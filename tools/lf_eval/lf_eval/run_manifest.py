# Copyright 2026 rcllf Contributors
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
"""Run manifest `trace_manifest.json` (schema `rcllf.manifest/1`).

Written by a run driver (a benchmark driver or the traced runtime tests of
colcon_lf) after `lttng stop`, from the `lttng list` text captured before `destroy`.
It lives in lf_eval, which reads it, so any driver can write it. It records what
only the driver can see: which events were
enabled per channel (with filters and exclusions), per-channel discard counts
(`unknown` when lttng printed no counter), contexts, the host's boot_id and
measured clock sync at run start and end, the expected producers and the
launcher exit status.
"""

from __future__ import annotations

import hashlib
import json
import re
import socket
import subprocess
from pathlib import Path
from typing import Any, Callable, Optional, Sequence

MANIFEST_SCHEMA = "rcllf.manifest/1"
RUN_MANIFEST_NAME = "trace_manifest.json"
UNKNOWN = "unknown"
UNAVAILABLE = "unavailable"

_CHANNEL_RE = re.compile(r"^- (\S+): \[(enabled|disabled)\]", re.M)
_RULE_RE = re.compile(r"^\s+(\S+) \(type: (\w+)\) \[(enabled|disabled)\](.*)$")
_FILTER_RE = re.compile(r"\[filter: '(.*)'\]")
_EXCL_RE = re.compile(r"\[exclusions: ([^\]]*)\]")


def parse_lttng_list(text: str) -> dict[str, dict[str, Any]]:
    """Per-channel attributes, statistics and event rules of `lttng list <session>`.

    A channel block without a `Discarded events` (or overwrite-mode `Lost
    packets`) line reports `unknown`, never 0: absence of the counter is not
    evidence of zero loss.
    """
    channels: dict[str, dict[str, Any]] = {}
    matches = list(_CHANNEL_RE.finditer(text))
    for i, m in enumerate(matches):
        block = text[m.end(): matches[i + 1].start() if i + 1 < len(matches) else len(text)]
        info: dict[str, Any] = {"enabled": m.group(2) == "enabled"}
        for key, label in (("loss_mode", "Event-loss mode"), ("subbuf_size", "Sub-buffer size"),
                           ("num_subbuf", "Sub-buffer count")):
            lm = re.search(rf"{label}:\s*(\S+)", block)
            info[key] = lm.group(1) if lm else UNKNOWN
        de = re.search(r"Discarded events:\s*(\d+)", block)
        lp = re.search(r"Lost packets:\s*(\d+)", block)
        info["discarded_events"] = int(de.group(1)) if de else UNKNOWN
        info["lost_packets"] = int(lp.group(1)) if lp else (UNKNOWN if info["loss_mode"] == "overwrite" else 0)
        if de is None and lp is not None:
            info["discarded_events"] = 0
        rules = []
        in_rules = False
        for line in block.splitlines():
            if "Recording event rules:" in line or "Event rules:" in line:
                in_rules = True
                continue
            if in_rules:
                rm = _RULE_RE.match(line)
                if rm:
                    fm, xm = _FILTER_RE.search(rm.group(4)), _EXCL_RE.search(rm.group(4))
                    rules.append({
                        "pattern": rm.group(1),
                        "type": rm.group(2),
                        "enabled": rm.group(3) == "enabled",
                        "filter": fm.group(1) if fm else None,
                        "exclusions": [x.strip() for x in xm.group(1).split(",")] if xm else [],
                    })
        info["rules"] = rules
        channels[m.group(1)] = info
    return channels


def channel_discards(info: dict[str, Any]) -> Any:
    """Events plus packets lost on a channel, or `unknown` if either count is."""
    ev, pk = info.get("discarded_events", UNKNOWN), info.get("lost_packets", UNKNOWN)
    if ev == UNKNOWN or pk == UNKNOWN:
        return UNKNOWN
    return int(ev) + int(pk)


# --- clock sync ------------------------------------------------------------------

_UNIT_S = {"s": 1.0, "ms": 1e-3, "us": 1e-6, "ns": 1e-9}


def _seconds(value: str, unit: str) -> float:
    return float(value) * _UNIT_S[unit]


def parse_chrony_tracking(text: str) -> Optional[dict[str, Any]]:
    """`chronyc tracking`: offset of the system clock and a max error bound.

    error_s = root_delay / 2 + root_dispersion (chrony's documented bound on
    the distance to the reference), offset_s signed (positive: local fast).
    """
    st = re.search(r"System time\s*:\s*([\d.]+) seconds (fast|slow)", text)
    rd = re.search(r"Root delay\s*:\s*([\d.]+) seconds", text)
    rdisp = re.search(r"Root dispersion\s*:\s*([\d.]+) seconds", text)
    if not (st and rd and rdisp):
        return None
    offset = float(st.group(1)) * (1 if st.group(2) == "fast" else -1)
    return {
        "source": "chrony",
        "offset_s": offset,
        "error_s": float(rd.group(1)) / 2 + float(rdisp.group(1)),
    }


def parse_timesync_status(text: str) -> Optional[dict[str, Any]]:
    """`timedatectl timesync-status` (systemd-timesyncd): Offset, Root distance."""
    off = re.search(r"Offset:\s*([+-]?[\d.]+)(ns|us|ms|s)\b", text)
    rdist = re.search(r"Root distance:\s*([\d.]+)(ns|us|ms|s)\b", text)
    if not (off and rdist):
        return None
    return {
        "source": "timesyncd",
        "offset_s": _seconds(off.group(1), off.group(2)),
        "error_s": _seconds(rdist.group(1), rdist.group(2)),
    }


def parse_pmc_current_data_set(text: str) -> Optional[dict[str, Any]]:
    """`pmc -u -b 0 'GET CURRENT_DATA_SET'` (linuxptp): offsetFromMaster, meanPathDelay in ns.

    Parser written against the linuxptp output format, no recorded fixture yet
    (the dev hosts run no ptp4l).
    """
    off = re.search(r"offsetFromMaster\s+([+-]?[\d.]+)", text)
    mpd = re.search(r"meanPathDelay\s+([+-]?[\d.]+)", text)
    if not (off and mpd):
        return None
    return {"source": "ptp", "offset_s": float(off.group(1)) * 1e-9,
            "error_s": abs(float(mpd.group(1))) * 1e-9}


Probe = Callable[[Sequence[str]], Optional[str]]


def _probe(argv: Sequence[str]) -> Optional[str]:
    try:
        r = subprocess.run(list(argv), stdout=subprocess.PIPE, stderr=subprocess.DEVNULL,
                           text=True, timeout=5)
    except (OSError, subprocess.TimeoutExpired):
        return None
    return r.stdout if r.returncode == 0 else None


def capture_clock_sync(probe: Probe = _probe) -> dict[str, Any]:
    """First available of chrony, PTP (pmc), timesyncd, else `unavailable`."""
    for argv, parser in (
        (["chronyc", "tracking"], parse_chrony_tracking),
        (["pmc", "-u", "-b", "0", "GET CURRENT_DATA_SET"], parse_pmc_current_data_set),
        (["timedatectl", "timesync-status"], parse_timesync_status),
    ):
        out = probe(argv)
        if out:
            parsed = parser(out)
            if parsed:
                return parsed
    return {"source": UNAVAILABLE}


def read_boot_id(path: Path = Path("/proc/sys/kernel/random/boot_id")) -> str:
    try:
        return path.read_text().strip()
    except OSError:
        return UNKNOWN


def tool_version(argv: Sequence[str], probe: Probe = _probe) -> str:
    out = probe(argv)
    return out.splitlines()[0].strip() if out else UNKNOWN


def file_sha256(path: Optional[Path]) -> Optional[str]:
    if path is None or not Path(path).is_file():
        return None
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


_LAUNCH_EXIT_RE = re.compile(
    r"\[(?:INFO|ERROR)\] \[(?P<name>[^\]]+)\]: process (?:has finished cleanly \[pid (?P<pid_ok>\d+)\]"
    r"|has died \[pid (?P<pid>\d+), exit code (?P<code>-?\d+))")


def parse_launch_exits(text: str) -> list[dict[str, Any]]:
    """Per-process exit status from `ros2 launch` output.

    `process has finished cleanly [pid N]` is returncode 0; `process has died
    [pid N, exit code C, ...]` is C (negative: killed by signal -C).
    """
    out = []
    for m in _LAUNCH_EXIT_RE.finditer(text):
        if m.group("pid_ok"):
            out.append({"role": "node_process", "name": m.group("name"),
                        "pid": int(m.group("pid_ok")), "returncode": 0, "signal": None})
        else:
            code = int(m.group("code"))
            out.append({"role": "node_process", "name": m.group("name"), "pid": int(m.group("pid")),
                        "returncode": code if code >= 0 else None,
                        "signal": -code if code < 0 else None})
    return out


def build_run_manifest(
    *,
    list_output: str,
    contexts: Sequence[str],
    clock_start: dict[str, Any],
    clock_end: dict[str, Any],
    launcher: dict[str, Any],
    launch_log: str = "",
    static_manifest: Optional[Path] = None,
    resolved_model_hash: Optional[str] = None,
    lttng_version: str = UNKNOWN,
    babeltrace_version: str = UNKNOWN,
    host: Optional[str] = None,
    boot_id: Optional[str] = None,
    readback: Optional[dict[str, Any]] = None,
) -> dict[str, Any]:
    """Assemble the `rcllf.manifest/1` run manifest document.

    ``readback`` is the configured-vs-read-back block of the manifest
    (produced by the benchmark driver); its ``valid`` is copied to the top level.
    """
    channels = parse_lttng_list(list_output)
    expected: list[dict[str, Any]] = []
    static_doc: dict[str, Any] = {}
    if static_manifest is not None and Path(static_manifest).is_file():
        static_doc = json.loads(Path(static_manifest).read_text())
        expected = [{"executable": e} for e in static_doc.get("executables", [])]
    doc: dict[str, Any] = {
        "schema": MANIFEST_SCHEMA,
        "kind": "run",
        "lttng_version": lttng_version,
        "babeltrace_version": babeltrace_version,
        "channels": {
            name: {
                "enabled": info["enabled"],
                "loss_mode": info["loss_mode"],
                "subbuf_size": info["subbuf_size"],
                "num_subbuf": info["num_subbuf"],
                "discarded_events": info["discarded_events"],
                "lost_packets": info["lost_packets"],
                "discarded": channel_discards(info),
                "rules": info["rules"],
                "contexts": list(contexts),
            }
            for name, info in channels.items()
        },
        "hosts": [{
            "host": host or socket.gethostname(),
            "boot_id": boot_id or read_boot_id(),
            "clock_sync_start": clock_start,
            "clock_sync_end": clock_end,
        }],
        "expected_producers": expected,
        "provider_marker": static_doc.get("provider_marker"),
        "processes": [launcher] + parse_launch_exits(launch_log),
        "static_manifest_hash": file_sha256(static_manifest),
        "resolved_model_hash": resolved_model_hash,
    }
    if readback is not None:
        doc["readback"] = readback
        doc["valid"] = readback.get("valid")
    return doc


def write_run_manifest(doc: dict[str, Any], run_dir: Path) -> Path:
    out = Path(run_dir) / RUN_MANIFEST_NAME
    out.write_text(json.dumps(doc, indent=2, sort_keys=True) + "\n")
    return out
