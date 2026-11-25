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
"""Read `rcllf:*` events out of a CTF trace (bt2), or a JSONL dump of one (D5).

The JSONL form is produced *from recorded CTF* by :func:`dump_jsonl` and is used
for scripted mutations in the unit tests (T7). It never replaces the CTF read.
"""

from __future__ import annotations

import json
from dataclasses import asdict, dataclass, field
from pathlib import Path
from typing import Any, Iterable, Optional

from lf_eval.semantic import schema

PROVIDER = "rcllf"


@dataclass
class SemEvent:
    """One rcllf event: name without provider prefix, common fields split out."""

    name: str
    ts: int
    epoch: int
    cat: int
    seq: int
    fields: dict[str, Any]
    pid: Optional[int] = None
    procname: Optional[str] = None
    channel: str = ""

    def gid(self) -> str:
        g = self.fields.get("gid")
        return bytes(g).hex() if isinstance(g, (list, tuple, bytes)) else ""


@dataclass
class ReadStats:
    """What the reader saw besides events: per-channel losses and decode errors."""

    discarded_events: dict[str, int] = field(default_factory=dict)
    discarded_packets: dict[str, int] = field(default_factory=dict)
    read_error: Optional[str] = None
    ros2_events: int = 0
    ros2_callback_events: int = 0


def _channel_of(stream: Any) -> str:
    """LTTng stream files are named <channel>_<cpu>; bt2 exposes the path as the stream name."""
    name = getattr(stream, "name", None) or ""
    base = Path(str(name)).name
    return base.rsplit("_", 1)[0] if "_" in base else base


def read_ctf(tracing_dir: Path) -> tuple[list[SemEvent], ReadStats]:
    """All rcllf events of *tracing_dir* in trace order, plus loss and error stats."""
    import bt2

    events: list[SemEvent] = []
    stats = ReadStats()
    try:
        for msg in bt2.TraceCollectionMessageIterator(str(tracing_dir)):
            if isinstance(msg, bt2._DiscardedEventsMessageConst):
                ch = _channel_of(msg.stream)
                c = msg.count if isinstance(getattr(msg, "count", None), int) else 1
                stats.discarded_events[ch] = stats.discarded_events.get(ch, 0) + c
                continue
            if isinstance(msg, bt2._DiscardedPacketsMessageConst):
                ch = _channel_of(msg.stream)
                c = msg.count if isinstance(getattr(msg, "count", None), int) else 1
                stats.discarded_packets[ch] = stats.discarded_packets.get(ch, 0) + c
                continue
            if type(msg) is not bt2._EventMessageConst:
                continue
            ev = msg.event
            provider, _, name = ev.name.partition(":")
            if provider != PROVIDER:
                if provider == "ros2":
                    stats.ros2_events += 1
                    stats.ros2_callback_events += name.startswith("callback_")
                continue
            payload = {k: _field(ev.payload_field[k]) for k in ev.payload_field}
            ctx = {}
            for attr in ("common_context_field", "specific_context_field"):
                f = getattr(ev, attr, None)
                if f is not None:
                    ctx.update({k: _field(f[k]) for k in f})
            events.append(SemEvent(
                name=name,
                ts=msg.default_clock_snapshot.ns_from_origin,
                epoch=int(payload.pop("epoch")),
                cat=int(payload.pop("cat")),
                seq=int(payload.pop("seq")),
                fields=payload,
                pid=ctx.get("vpid"),
                procname=ctx.get("procname"),
                channel=_channel_of(ev.packet.stream) if getattr(ev, "packet", None) is not None else "",
            ))
    except Exception as exc:  # truncated packet, bad metadata: integrity read_error
        stats.read_error = f"{type(exc).__name__}: {exc}"
    return events, stats


def _field(f: Any) -> Any:
    """bt2 field -> int, str or list of ints."""
    import bt2

    if isinstance(f, bt2._StringFieldConst):
        return str(f)
    if isinstance(f, (bt2._ArrayFieldConst,)):
        return [_field(x) for x in f]
    try:
        return int(f)
    except (TypeError, ValueError):
        return str(f)


def dump_jsonl(events: Iterable[SemEvent], stats: ReadStats, path: Path) -> Path:
    """First line: stats; then one event per line."""
    path = Path(path)
    with path.open("w") as fh:
        fh.write(json.dumps({"stats": asdict(stats)}, sort_keys=True) + "\n")
        for e in events:
            fh.write(json.dumps(asdict(e), sort_keys=True) + "\n")
    return path


def load_jsonl(path: Path) -> tuple[list[SemEvent], ReadStats]:
    lines = Path(path).read_text().splitlines()
    stats = ReadStats(**json.loads(lines[0])["stats"])
    return [SemEvent(**json.loads(line)) for line in lines[1:] if line.strip()], stats


def read_run(run_dir: Path) -> tuple[list[SemEvent], ReadStats]:
    """Events of a run directory: `tracing/` CTF if present, else `semantic_events.jsonl`."""
    run_dir = Path(run_dir)
    if (run_dir / "tracing").is_dir():
        return read_ctf(run_dir / "tracing")
    return load_jsonl(run_dir / "semantic_events.jsonl")


def known_event(name: str) -> bool:
    return name in schema.EVENTS
