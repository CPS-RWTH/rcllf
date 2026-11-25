# Copyright 2026 rcllf Contributors
# Licensed under the Apache License, Version 2.0
"""Counterexample corpus of the claim matrix, CE1.

Executes the system recorded in
``fixtures/counterexamples/shared_state_two_callbacks.json`` under every order
of its schedule domain and checks the recorded expected outputs. It is the
falsification case of row U1: deterministic callbacks that share state and have
two permitted orders give different outputs, so schedule invariance is not
implied by determinism of the callbacks. It runs no ROS 2 code.
"""

from __future__ import annotations

import json
from itertools import permutations
from pathlib import Path

CORPUS = Path(__file__).parent / "fixtures" / "counterexamples" / "shared_state_two_callbacks.json"
INPUTS = {"on_imu": 7, "on_scan": 100}


class SharedState:
    """CE1: ``on_scan`` publishes the IMU value ``on_imu`` last stored."""

    def __init__(self):
        self.last_imu = 0
        self.out = []

    def on_imu(self, v):
        self.last_imu = v

    def on_scan(self, r):
        self.out.append([r, self.last_imu])


class CommutingWrite:
    """Control: the shared write commutes, the observation inside ``on_scan`` does not."""

    def __init__(self):
        self.total = 0
        self.out = []

    def on_imu(self, v):
        self.total += v

    def on_scan(self, r):
        self.total += r
        self.out.append([self.total])


class NoSharedState:
    """Control: independent callbacks, each publishing its own input."""

    def __init__(self):
        self.out = []

    def on_imu(self, v):
        self.out.append([v])

    def on_scan(self, r):
        self.out.append([r])


def run(system_cls, order):
    system = system_cls()
    for callback in order:
        getattr(system, callback)(INPUTS[callback])
    return system


def outputs(system_cls, domain):
    return {",".join(order): run(system_cls, order).out for order in domain}


def corpus():
    return json.loads(CORPUS.read_text())


def test_the_unconstrained_domain_is_every_order_of_the_two_callbacks():
    domain = corpus()["schedule_domain"]["unconstrained"]
    assert sorted(map(tuple, domain)) == sorted(permutations(INPUTS))


def test_two_permitted_orders_give_different_outputs():
    c = corpus()
    actual = outputs(SharedState, c["schedule_domain"]["unconstrained"])
    assert actual == c["expected_outputs"]
    assert len({json.dumps(v) for v in actual.values()}) == 2


def test_an_order_constraint_leaves_one_schedule_and_one_output():
    c = corpus()
    actual = outputs(SharedState, c["schedule_domain"]["lf_reaction_priority"])
    assert actual == {"on_imu,on_scan": c["expected_outputs"]["on_imu,on_scan"]}


def test_commuting_state_updates_alone_are_not_confluence():
    control = corpus()["confluent_control"]
    domain = corpus()["schedule_domain"]["unconstrained"]
    assert outputs(CommutingWrite, domain) == control["expected_outputs"]
    assert {run(CommutingWrite, o).total for o in domain} == {control["final_state"]}


def test_without_shared_state_only_the_output_sequence_depends_on_the_order():
    domain = corpus()["schedule_domain"]["unconstrained"]
    actual = outputs(NoSharedState, domain)
    assert len({json.dumps(v) for v in actual.values()}) == 2
    expected = corpus()["commuting_control"]["expected_output_multisets"]
    assert all(sorted(v) == expected for v in actual.values())
