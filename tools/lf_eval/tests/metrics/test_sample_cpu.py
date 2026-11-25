# Copyright 2026 rcllf Contributors
# Licensed under the Apache License, Version 2.0
"""Tests for the /proc CPU sampler.

The assertions pin down the tricky ``/proc/<pid>/stat`` comm-with-parens
parsing, the ``machine_cores`` detection and the injectable clock that makes :func:`sample` testable without real sleeps.
"""

from __future__ import annotations

import pytest

from lf_eval.metrics.sample_cpu import (
    aggregate_cores,
    build_report,
    cores_used,
    discover_pids,
    format_pid_list,
    machine_cores,
    read_proc_stat,
    sample,
    write_report,
)


def _write_fake_stat(tmp_path, pid, utime, stime):
    directory = tmp_path / str(pid)
    directory.mkdir(exist_ok=True)
    # /proc/<pid>/stat: fields 14 (utime) and 15 (stime) are 1-indexed.
    fields = ["0"] * 52
    fields[0] = str(pid)
    fields[1] = "(ros2 node)"
    fields[13] = str(utime)
    fields[14] = str(stime)
    (directory / "stat").write_text(" ".join(fields) + "\n")


def _write_cmdline(tmp_path, pid, cmdline):
    directory = tmp_path / str(pid)
    directory.mkdir(exist_ok=True)
    # /proc/<pid>/cmdline is NUL-separated.
    (directory / "cmdline").write_bytes(cmdline.replace(" ", "\0").encode() + b"\0")


# ── read_proc_stat ───────────────────────────────────────────────────────


def test_read_proc_stat_parses_utime_stime(tmp_path):
    _write_fake_stat(tmp_path, 1234, utime=500, stime=120)
    assert read_proc_stat(1234, proc_root=str(tmp_path)) == (500, 120)


def test_read_proc_stat_handles_comm_with_spaces_and_parens(tmp_path):
    """comm may contain spaces and parens; the numeric tail starts after the LAST ')'."""
    directory = tmp_path / "77"
    directory.mkdir()
    fields = ["0"] * 52
    fields[0] = "77"
    fields[13] = "10"
    fields[14] = "3"
    line = "77 (weird (name) 9) " + " ".join(fields[2:]) + "\n"
    (directory / "stat").write_text(line)
    assert read_proc_stat(77, proc_root=str(tmp_path)) == (10, 3)


def test_read_proc_stat_missing_pid_returns_none(tmp_path):
    assert read_proc_stat(999, proc_root=str(tmp_path)) is None


def test_read_proc_stat_malformed_returns_none(tmp_path):
    directory = tmp_path / "5"
    directory.mkdir()
    (directory / "stat").write_text("no parenthesis here\n")
    assert read_proc_stat(5, proc_root=str(tmp_path)) is None


def test_read_proc_stat_truncated_fields_returns_none(tmp_path):
    directory = tmp_path / "6"
    directory.mkdir()
    (directory / "stat").write_text("6 (x) S 1 2 3\n")
    assert read_proc_stat(6, proc_root=str(tmp_path)) is None


# ── cores_used ───────────────────────────────────────────────────────────


def test_cores_used_full_core():
    # 1000 ticks over 10 s at 100 Hz = 10 CPU-s / 10 wall-s = 1.0 core
    assert cores_used(0, 0, 1000, 0, wall_s=10.0, clk_tck=100) == 1.0


def test_cores_used_splits_user_and_system():
    # (400 user + 100 sys) / 100 Hz = 5 CPU-s over 10 wall-s = 0.5 core
    assert cores_used(0, 0, 400, 100, wall_s=10.0, clk_tck=100) == 0.5


def test_cores_used_multiple_cores():
    # 3000 ticks / 100 Hz = 30 CPU-s over 10 wall-s = 3 cores
    assert cores_used(0, 0, 3000, 0, wall_s=10.0, clk_tck=100) == 3.0


def test_cores_used_zero_wall_returns_zero():
    assert cores_used(0, 0, 100, 0, wall_s=0.0, clk_tck=100) == 0.0


def test_cores_used_zero_clk_tck_returns_zero():
    assert cores_used(0, 0, 100, 0, wall_s=1.0, clk_tck=0) == 0.0


# ── discovery / aggregation ──────────────────────────────────────────────


def test_discover_pids_matches_substring(tmp_path):
    _write_cmdline(tmp_path, 10, "ros2 launch running_example")
    _write_cmdline(tmp_path, 11, "AutowareReferenceSystem")
    _write_cmdline(tmp_path, 12, "sshd -D")
    found = discover_pids(
        ["running_example", "AutowareReferenceSystem"], proc_root=str(tmp_path)
    )
    assert set(found) == {10, 11}


def test_discover_pids_skips_non_numeric_entries(tmp_path):
    (tmp_path / "self").mkdir()
    _write_cmdline(tmp_path, 20, "target_node")
    assert set(discover_pids(["target_node"], proc_root=str(tmp_path))) == {20}


def test_discover_pids_ignores_empty_cmdline(tmp_path):
    _write_cmdline(tmp_path, 30, "")
    assert discover_pids(["anything"], proc_root=str(tmp_path)) == {}


def test_discover_pids_missing_root_returns_empty(tmp_path):
    assert discover_pids(["x"], proc_root=str(tmp_path / "nope")) == {}


def test_aggregate_cores_sums_per_pid():
    assert aggregate_cores({10: {"cores": 0.5}, 11: {"cores": 0.25}}) == 0.75


def test_format_pid_list_is_bare_newline_separated():
    """--list-pids output must parse as bare ints: no JSON, no summary line."""
    out = format_pid_list({11: "b", 10: "a"}.keys())
    assert out.split() == ["10", "11"]
    for token in out.split():
        int(token)


# ── machine_cores / report ───────────────────────────────────────────────


def test_machine_cores_detects_by_default():
    assert machine_cores() >= 1


def test_machine_cores_honours_override():
    assert machine_cores(12) == 12


def test_machine_cores_ignores_nonpositive_override():
    assert machine_cores(0) == machine_cores()
    assert machine_cores(-4) == machine_cores()


def test_build_report_normalises_percentages():
    report = build_report(
        {1: {"cores": 0.5}, 2: {"cores": 0.1}},
        wall_s=50.0,
        clk_tck=100,
        warmup_s=10.0,
        window_s=50.0,
        cores=12,
    )
    assert report["aggregate_cores"] == 0.6
    assert report["aggregate_pct_one_core"] == 60.0
    assert report["aggregate_pct_machine"] == pytest.approx(5.0)
    assert report["n_cores_machine"] == 12


def test_build_report_with_no_processes():
    report = build_report({}, wall_s=1.0, clk_tck=100, warmup_s=0.0, window_s=1.0)
    assert report["aggregate_cores"] == 0.0
    assert report["aggregate_pct_machine"] == 0.0


# ── sample() with injected clock ─────────────────────────────────────────


def test_sample_computes_cores_without_sleeping(tmp_path, monkeypatch):
    """A fake clock/sleep lets the whole sampler run instantly."""
    monkeypatch.setattr("os.sysconf", lambda _name: 100)
    _write_cmdline(tmp_path, 42, "my_target_node --ros-args")
    _write_fake_stat(tmp_path, 42, utime=0, stime=0)

    times = iter([0.0, 10.0])  # start, end => 10 s window
    calls = {"n": 0}

    def fake_sleep(_seconds):
        # sample() sleeps twice: warmup, then the window. Only during the
        # window (the second call) does the process accumulate CPU, so the
        # baseline snapshot taken between the two sleeps reads zero.
        calls["n"] += 1
        if calls["n"] == 2:
            _write_fake_stat(tmp_path, 42, utime=400, stime=100)

    report = sample(
        ["my_target_node"],
        warmup_s=0.0,
        window_s=10.0,
        proc_root=str(tmp_path),
        cores=4,
        sleep=fake_sleep,
        clock=lambda: next(times),
    )
    # 500 ticks / 100 Hz = 5 CPU-s over 10 wall-s = 0.5 cores
    assert report["aggregate_cores"] == pytest.approx(0.5)
    assert report["aggregate_pct_machine"] == pytest.approx(12.5)
    assert report["per_pid"][42]["cores"] == pytest.approx(0.5)


def test_sample_raises_when_no_pid_matches(tmp_path):
    with pytest.raises(LookupError, match="no pids matched"):
        sample(
            ["ghost"],
            warmup_s=0.0,
            window_s=0.0,
            proc_root=str(tmp_path),
            sleep=lambda _s: None,
            clock=lambda: 0.0,
        )


def test_write_report_roundtrips(tmp_path):
    import json

    path = tmp_path / "cpu_proc.json"
    write_report({"aggregate_cores": 1.25}, str(path))
    assert json.loads(path.read_text())["aggregate_cores"] == 1.25


# --- per-CPU utilisation -----------------------------------------

from lf_eval.metrics.sample_cpu import per_cpu_busy, read_cpu_times  # noqa: E402


def _write_proc_stat(tmp_path, cpus):
    lines = ["cpu  1 1 1 1 1 1 1 1 0 0"]
    for i, (user, idle) in enumerate(cpus):
        lines.append(f"cpu{i} {user} 0 0 {idle} 0 0 0 0 0 0")
    lines.append("intr 5 0 0")
    (tmp_path / "stat").write_text("\n".join(lines) + "\n")


def test_read_cpu_times_per_cpu_only(tmp_path):
    _write_proc_stat(tmp_path, [(30, 70), (5, 95)])
    assert read_cpu_times(str(tmp_path)) == {0: (30, 100), 1: (5, 100)}


def test_per_cpu_busy_between_two_samples(tmp_path):
    _write_proc_stat(tmp_path, [(0, 0), (0, 0)])
    first = read_cpu_times(str(tmp_path))
    # cpu0 fully busy, cpu1 one quarter busy over the window.
    _write_proc_stat(tmp_path, [(100, 0), (25, 75)])
    assert per_cpu_busy(first, read_cpu_times(str(tmp_path))) == {0: 1.0, 1: 0.25}


def test_read_cpu_times_missing_proc(tmp_path):
    assert read_cpu_times(str(tmp_path / "nope")) == {}
