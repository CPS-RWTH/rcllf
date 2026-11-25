# Copyright 2026 rcllf Contributors
# SPDX-License-Identifier: Apache-2.0
"""The yaml ``stp_offset`` governs the ingress STP of an admitted xcorr edge.

The ingress end of an admitted decentralized
cross-federate edge needs an STP reaction, else it waits 0 for the records and
every record is tardy. The xcorr link's ``xcorr_g<G>`` ports name no
``after_overrides`` entry, so the offset is looked up by the link ports of the
user's edge (``edge.link_ports``), not read from the xcorr link (which would
give every ingress the 10 msec default).
"""

from __future__ import annotations

import re

from colcon_lf.federation.partition import AfterOverride

from .test_xfed_correspondence import _pubsub


def _ingress_stp(lf: str) -> list[str]:
    return re.findall(r"STP\(([^)]*)\)", lf)


def test_yaml_stp_offset_reaches_the_xcorr_ingress(tmp_path):
    out = _pubsub(
        tmp_path, coordination="decentralized",
        overrides=[AfterOverride("pub.data", "sub.data", 20_000_000, 3_000_000)])
    assert _ingress_stp(out["sub"]["lf"]) == ["3 msec"], (
        f"expected STP(3 msec) from the yaml stp_offset, got STP({', '.join(_ingress_stp(out['sub']['lf']))})")


def test_override_without_stp_offset_keeps_the_default(tmp_path):
    out = _pubsub(
        tmp_path, coordination="decentralized",
        overrides=[AfterOverride("pub.data", "sub.data", 20_000_000)])
    assert _ingress_stp(out["sub"]["lf"]) == ["10 msec"]


def test_ten_msec_yaml_stp_offset_is_ten_msec(tmp_path):
    out = _pubsub(
        tmp_path, coordination="decentralized",
        overrides=[AfterOverride("pub.data", "sub.data", 20_000_000, 10_000_000)])
    assert _ingress_stp(out["sub"]["lf"]) == ["10 msec"]


def test_centralized_xcorr_ingress_has_no_stp(tmp_path):
    out = _pubsub(
        tmp_path, overrides=[AfterOverride("pub.data", "sub.data", 20_000_000, 3_000_000)])
    assert _ingress_stp(out["sub"]["lf"]) == []
