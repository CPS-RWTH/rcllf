# Copyright 2026 rcllf Contributors
# Licensed under the Apache License, Version 2.0
"""BNA oracle, written from the BNA specification and nothing else.

Input is a list of event records (dicts with ``ev`` = ``params``, ``admit``,
``missing``, ``wake`` or ``close``). Output is the list of ``decision`` and
``sample`` records in emission order. Tags are ``(time, microstep)`` pairs and
compare lexicographically. Stamps and times are integer nanoseconds.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Iterable, Optional

ANCHOR = 0
OTHER = 1


class OracleError(ValueError):
    """The event stream itself is malformed (not a BNA outcome)."""


def split_key(key: str) -> tuple[int, str, int]:
    """``"<epoch>:<gid>:<ordinal>"`` as (epoch, gid, ordinal)."""
    epoch, _, rest = key.partition(":")
    gid, _, ordinal = rest.rpartition(":")
    if not gid:
        raise OracleError(f"malformed key {key!r}")
    return int(epoch), gid, int(ordinal)


@dataclass(frozen=True)
class Params:
    epoch: int
    delta: int
    lateness: tuple[int, int]
    capacity: tuple[int, int]
    retention: int
    timeout: int

    @classmethod
    def from_record(cls, r: dict) -> "Params":
        p = cls(int(r["epoch"]), int(r["delta"]), tuple(int(x) for x in r["lateness"]),
                tuple(int(x) for x in r["capacity"]), int(r["retention"]), int(r["timeout"]))
        if (p.delta < 0 or min(p.lateness) < 0 or min(p.capacity) < 1 or p.retention <= 0
                or p.timeout <= 0 or len(p.lateness) != 2 or len(p.capacity) != 2):
            raise OracleError(f"parameters out of range: {r}")
        return p


@dataclass
class Sample:
    input: int
    stamp: int
    key: str
    order: tuple[int, str, int]
    deadline: tuple[int, int]

    def sk(self) -> tuple:
        return (self.stamp, self.order)


class Oracle:
    """One synchronizer. Feed events with :meth:`feed`, read :attr:`out`."""

    def __init__(self, params: Params, sync: int = 0):
        self.p = params
        self.sync = sync
        self.anchors: list[Sample] = []
        self.cands: list[Sample] = []
        self.max_stamp: list[Optional[int]] = [None, None]
        self.last_ordinal: dict[tuple[int, str], int] = {}
        self.closed = False
        self.last_tag: tuple[int, int] = (-(1 << 62), 0)
        self.out: list[dict] = []
        self.missing_count = [0, 0]

    # -- output records --------------------------------------------------
    def _decision(self, a: Sample, tag, outcome: str, reason: str = "",
                  partner: Optional[Sample] = None) -> None:
        self.out.append({
            "ev": "decision", "sync": self.sync, "anchor": a.key, "stamp": a.stamp,
            "outcome": outcome, "reason": reason,
            "partner": partner.key if partner else None,
            "partner_stamp": partner.stamp if partner else None,
            "tag": list(tag),
        })

    def _sample(self, x: Sample, tag, outcome: str, reason: str = "") -> None:
        self.out.append({
            "ev": "sample", "sync": self.sync, "input": x.input, "key": x.key,
            "stamp": x.stamp, "outcome": outcome, "reason": reason, "tag": list(tag),
        })

    def _reject(self, x: Sample, tag, reason: str) -> None:
        """Terminal outcome of a sample refused at admission (F1 to F4, R4)."""
        if x.input == ANCHOR:
            self._decision(x, tag, "REJECTED", reason)
        elif reason == "late":
            self._sample(x, tag, "LATE")
        else:
            self._sample(x, tag, "FAULT", reason)

    # -- state helpers ---------------------------------------------------
    def watermark(self, i: int) -> Optional[int]:
        m = self.max_stamp[i]
        return None if m is None else m - self.p.lateness[i]

    def _in_window(self, a: Sample, y: Sample) -> bool:
        return abs(y.stamp - a.stamp) <= self.p.delta

    def _window_anchors(self, y: Sample) -> list[Sample]:
        return sorted((a for a in self.anchors if self._in_window(a, y)), key=Sample.sk)

    # -- rules -----------------------------------------------------------
    def _catch_up(self, t, inclusive: bool) -> None:
        while True:
            live = [x.deadline for x in self.anchors] + [x.deadline for x in self.cands]
            if not live:
                return
            d = min(live)
            if d < t or (inclusive and d == t):
                self._deadline_step(d)
            else:
                return

    def _deadline_step(self, d) -> None:
        for y in sorted((y for y in self.cands if y.deadline == d), key=Sample.sk):
            self.cands.remove(y)
            self._sample(y, d, "EXPIRED")
            for a in self._window_anchors(y):
                self.anchors.remove(a)
                self._decision(a, d, "REJECTED", "candidate_expired")
        for a in sorted((a for a in self.anchors if a.deadline == d), key=Sample.sk):
            self.anchors.remove(a)
            self._decision(a, d, "REJECTED", "timeout")
        self._decide(d)

    def _supersede(self, tag) -> None:
        w0 = self.watermark(ANCHOR)
        if w0 is None:
            return  # before any anchor A_lb is minus infinity
        a_lb = w0 + 1
        if self.anchors:
            a_lb = min(a_lb, min(self.anchors, key=Sample.sk).stamp)
        for y in sorted(self.cands, key=Sample.sk):
            if y.stamp + self.p.delta < a_lb:
                self.cands.remove(y)
                self._sample(y, tag, "SUPERSEDED")

    def _complete(self, a: Sample) -> bool:
        bound = a.stamp + self.p.delta
        for i in (ANCHOR, OTHER):
            w = self.watermark(i)
            if w is None or not w > bound:
                return False
        return True

    def _decide(self, tag) -> None:
        self._supersede(tag)
        while self.anchors:
            a = min(self.anchors, key=Sample.sk)
            if not self._complete(a):
                return
            self.anchors.remove(a)
            window = [c for c in self.cands if self._in_window(a, c)]
            if not window:
                self._decision(a, tag, "REJECTED", "missing")
            else:
                c = min(window, key=lambda c: (abs(c.stamp - a.stamp), c.stamp, c.order))
                self.cands.remove(c)
                self._decision(a, tag, "MATCHED", partner=c)
                self._sample(c, tag, "SELECTED")
            self._supersede(tag)

    def _capacity(self, i: int, tag) -> None:
        buf = self.anchors if i == ANCHOR else self.cands
        if len(buf) <= self.p.capacity[i]:
            return
        v = min(buf, key=Sample.sk)
        buf.remove(v)
        if i == ANCHOR:
            self._decision(v, tag, "REJECTED", "anchor_evicted")
            return
        self._sample(v, tag, "OVERFLOW")
        for a in self._window_anchors(v):
            self.anchors.remove(a)
            self._decision(a, tag, "REJECTED", "candidate_evicted")

    # -- events ----------------------------------------------------------
    def _check_tag(self, t) -> tuple[int, int]:
        t = (int(t[0]), int(t[1]))
        if t < self.last_tag:
            raise OracleError(f"tag {list(t)} before {list(self.last_tag)}")
        self.last_tag = t
        return t

    def admit(self, i: int, stamp: int, key: str, t) -> None:
        t = self._check_tag(t)
        if i not in (ANCHOR, OTHER):
            raise OracleError(f"input {i} out of range")
        self._catch_up(t, inclusive=False)
        epoch, gid, ordinal = split_key(key)
        x = Sample(i, int(stamp), key, (epoch, gid, ordinal), (0, 0))
        if self.closed:
            return self._reject(x, t, "closed")
        if epoch != self.p.epoch:
            return self._reject(x, t, "identity")
        last = self.last_ordinal.get((i, gid))
        if last is not None and ordinal == last:
            return self._reject(x, t, "duplicate")
        if last is not None and ordinal < last:
            return self._reject(x, t, "order")
        self.last_ordinal[(i, gid)] = ordinal
        if x.stamp < 0:
            return self._reject(x, t, "stamp")
        w = self.watermark(i)
        if w is not None and x.stamp <= w:
            return self._reject(x, t, "late")
        span = self.p.timeout if i == ANCHOR else self.p.retention
        x.deadline = (t[0] + span, 0)
        (self.anchors if i == ANCHOR else self.cands).append(x)
        m = self.max_stamp[i]
        self.max_stamp[i] = x.stamp if m is None else max(m, x.stamp)
        self._supersede(t)
        self._capacity(i, t)
        self._decide(t)

    def missing(self, i: int, key: str, t, stamp: Optional[int] = None) -> None:
        """A lost sample. With *stamp* (its publish tag minus the
        input's declared ``max_stamp_lag``, a lower bound of its header stamp)
        the watermark rises as on admit and pending anchors may decide now."""
        t = self._check_tag(t)
        self._catch_up(t, inclusive=False)
        split_key(key)
        self.missing_count[i] += 1
        if stamp is None or stamp < 0 or self.closed:
            return
        m = self.max_stamp[i]
        self.max_stamp[i] = stamp if m is None else max(m, stamp)
        self._decide(t)

    def wake(self, t) -> None:
        t = self._check_tag(t)
        self._catch_up(t, inclusive=True)

    def close(self, reason: str, t) -> None:
        if reason not in ("shutdown", "epoch", "cancelled"):
            raise OracleError(f"close reason {reason!r}")
        t = self._check_tag(t)
        self._catch_up(t, inclusive=True)
        if self.closed:
            return
        for a in sorted(self.anchors, key=Sample.sk):
            self._decision(a, t, "REJECTED", reason)
        for y in sorted(self.cands, key=Sample.sk):
            self._sample(y, t, "FLUSHED")
        self.anchors.clear()
        self.cands.clear()
        self.closed = True

    def feed(self, r: dict) -> None:
        ev = r.get("ev")
        if ev == "admit":
            self.admit(int(r["input"]), int(r["stamp"]), r["key"], r["tag"])
        elif ev == "missing":
            stamp = r.get("stamp")
            self.missing(int(r["input"]), r["key"], r["tag"],
                         None if stamp is None else int(stamp))
        elif ev == "wake":
            self.wake(r["tag"])
        elif ev == "close":
            self.close(r["reason"], r["tag"])
        elif ev == "params":
            raise OracleError("second params record")
        else:
            raise OracleError(f"unknown input event {ev!r}")


INPUT_EVENTS = ("params", "admit", "missing", "wake", "close")
OUTPUT_EVENTS = ("decision", "sample")


def run(events: Iterable[dict]) -> list[dict]:
    """Outputs of the oracle for one synchronizer's input events."""
    events = [e for e in events if e.get("ev") in INPUT_EVENTS]
    if not events or events[0]["ev"] != "params":
        raise OracleError("the first event must be params")
    o = Oracle(Params.from_record(events[0]), int(events[0].get("sync", 0)))
    for r in events[1:]:
        o.feed(r)
    return o.out
