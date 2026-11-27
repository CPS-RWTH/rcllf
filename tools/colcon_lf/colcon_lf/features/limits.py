# Copyright 2026 rcllf Contributors
# Licensed under the Apache License, Version 2.0
"""The stated limits of the support matrix, L1 to Ln, in one shipped source.

Each limit bounds what is claimed for the features that cite it in ``evidence.EVIDENCE``
(by slug). ``docs_table`` renders them into ``docs/features/support-matrix.md``. The
id of a limit (L1, L2, ...) is its position in ``LIMIT_ROWS``, never typed by hand, so
a new limit goes at the end unless it belongs between two rows.

This module imports nothing, so code that ships and the development tooling can both
read it. The evaluation pages take their limit rows from here too and add only their
evidence, so no two pages state a limit differently.
"""

from __future__ import annotations

# (slug, statement), in page order. A statement is one table cell: no line break and no
# unescaped '|'.
LIMIT_ROWS: tuple[tuple[str, str], ...] = (
    (
        'cross-federate-loss-detection',
        'Detected loss holds within one process. An admitted cross-federate edge between two '
        'federates on one host detects loss too (shown in the evaluation of the rcllf paper). '
        'A relay publisher is admitted too. Federates on two hosts are '
        'admitted when the deployment declares a bound on the offset of the two clocks below the '
        'edge timeout. Cross-federate edges that are not admitted stay '
        'undetected. A decentralized edge is '
        'admitted too, except into a BNA input. '
        'A cross-federate link has `after 0` by default and is admitted at every '
        '`after` of 0 or more, because the sender sends at the publishing tag. The generation '
        'switch that chose `after 0` is gone. Only the flush sender, used under '
        'decentralized coordination or with `RCLLF_XFED_FLUSH=1`, still refuses an `after` of 1 '
        'ns (`xfed_flush_min`). An earlier version refused every flush edge '
        '(`xfed_flush_no_complete`), because the flush carried no COMPLETE record. '
        'The flush message carries one COMPLETE number per '
        'time, a lost flush message is reported as CONTROL_FAILURE, and flush edges, among them '
        'a decentralized edge with an `after` above 0, are admitted again. Federates on two '
        'hosts without a declared bound are refused at the first record. A legacy cross-federate '
        'link written at a third microstep of one time exits instead of merging. '
        'Under decentralized coordination such a link exits at its second microstep of one time, '
        'since the receiver judges that second message tardy and moves or drops it. '
        'A cross-federate edge detects a lost sample at S(a), the physical start of '
        "the publishing activation on the publisher's clock, plus d plus its own timeout. A "
        'sample counts only by its ingress stamp, and every publication of '
        'one activation shares the cutoff S(a) + d + T, so a publish delayed by an upstream join '
        "keeps its full d + T and no timeout is chained over the publisher's inputs. The timeout chain, "
        'its loop slack rule and the join hold limit are gone. A '
        "lost sample's join still holds its tag until its cutoff, since LF runs the reactions of "
        'a tag level by level, which costs latency but no spurious loss. '
        'A subscription can set `join: deferred`: its drain '
        'runs at the cutoff tag, publish tag + d + T, a deferred edge never blocks the LF worker '
        'waiting for its sample, and its dispatch tag is fixed. Only a sample taken in strictly '
        'before the cutoff is delivered there, and the eligible input set can still differ '
        'across runs when an arrival races the cutoff. Blocking stays the default, and LF still '
        'waits for physical time to reach each tag. '
        'An edge inside one federate has no cutoff: the publishing thread takes the '
        "sample from the subscriber's in-process reader before the publish returns, a sample not "
        'found there ends `LOST` (`local_undelivered`, a reconcile failure), the drain never '
        'waits, and `join: deferred` on such an edge is a generation error. '
        'A cross-federate cutoff counts from the publication '
        'tag instead of S(a), t + max(0, d) + T + eps, so a sender behind its tags makes its '
        'outputs late at the receiver, and the `publisher_late` counter is gone.',
    ),
    (
        'nav2-lifecycle-actions',
        'Running Nav2 under lfc is future work. Gazebo `/clock` against logical time is '
        'not supported. A tf2 lookup is '
        'supported only through `rcllf_tf2::AcquisitionLookup`, which decides each request by '
        'the logical deadline request tag + wait bound, with a `TransformListener` whose `/tf` '
        'and `/tf_static` are published by nodes of the system; a direct or blocking `Buffer` '
        'lookup and `/tf` from outside the system are refused with RCLLF-R002. Action servers '
        'are supported when they send feedback and results only from callbacks that LF reactions '
        'run. A server that sends from its own thread is refused with RCLLF-H011. A '
        'server whose only clients are outside the LF system runs at tags set by request '
        'arrival. Client feedback is not a correspondence edge. An action with several client '
        'nodes, and two actions on one node where one name is a path suffix of the other (`move` '
        'and `/right/move`), are refused at generation. A server whose action also has a client '
        'inside the LF system takes no goal from a client outside it. '
        'For a lifecycle node, a transition made at tag T holds for every reaction after T, an inactive '
        'node runs no callback, its entities are re-bound after each change, and a handle a node '
        'drops at tag T stays valid for the other reactions at T. A `LifecyclePublisher` holds '
        'its activation for the tag as well, so a publish by another reaction at T does not '
        'depend on the reaction order. A transition a reaction makes holds from the '
        'same tag in every run. A transition completed outside LF, including `ros2 lifecycle '
        'set`, enters LF as a physical action and holds from the tag after that action, which '
        'follows the wall clock and so differs between runs. A node built without the lifecycle '
        "communication interface, or an executable without the binding's publication gate, falls "
        'back to reading the state when a callback returns or at a boundary, so such a '
        'transition carries no timestamp and can apply at a logical tag earlier than its '
        'physical time. A topic edge of a '
        'lifecycle node is admitted to correspondence, also across federates, and a publisher '
        'the node makes after startup is bound to the edge. A subscription that a lifecycle hook '
        'makes or resets keeps its edge legacy, which L6 bounds. '
        'A node that leaves the active state has the pending requests of its '
        'service clients pruned and the goals of its action clients cancelled at the first tag '
        'after the deactivation, so a reactivated node starts with none. This holds for a client '
        'type the analysis names in full (`pkg::srv::Name`, `pkg::action::Name`). A client of '
        'another type keeps its work in flight, and the binding warns once.',
    ),
    (
        'self-loop-and-startup-qos',
        'A self-loop edge that correspondence admission refuses stays on the stub path without a '
        'logical delivery guarantee. The reasons include `self_loop_split`, a '
        'subscription in another callback group than its publishers, and `burst_unbounded`, a '
        'publisher without analyzed publish bounds. RCLLF-H001 and RCLLF-H002 are also checked '
        'at startup on the effective QoS of the reader of every admitted and legacy edge. '
        'Every other subscription, such as an input from outside the analyzed graph, gets '
        'RCLLF-H001 at startup by its own reaction, so twin subscriptions on one topic are '
        'judged apart, and a subscription whose reliability the analysis did not see logs '
        'RCLLF-H000 with the effective value. A publisher whose reliability the analysis '
        'did not see logs RCLLF-H000 at startup with the effective value too. Two kinds '
        'of publisher are not checked at startup and keep only the static RCLLF-H000. One is a '
        'publisher whose topic is computed at run time. The other is a user publisher with the '
        'message type of a publisher the node creates for itself on the same topic, which is '
        '`/rosout`, `/parameter_events`, the `transition_event` topic of a lifecycle node, and '
        'the `_action/feedback` and `_action/status` topics of an action server.',
    ),
    (
        'rq4-failed-refsys-runs',
        'The reference-system evaluation of the rcllf paper (its data is not part of this '
        'repository) reports 5 of 160 runs as failed, all '
        '`stock-launch-per-node-depth8` (3 idle, 2 under `cpu-mem`), carrying 6 counted losses '
        '(an earlier evaluation: 12 runs, 15 losses, same class). The per-reader accounting puts each '
        'counted loss inside the match-resolution window, a gap between match-probe polls. '
        'They are reported, not rerun. That only depth 8 fails is an artefact of the '
        'classifier: a depth-1 reader counted the same startup losses as '
        'overwritten. With the fix, a 20-run stock depth-1 control fails 4 runs, so the stock '
        'depth-1 rows await a re-summary. CPU is whole-tree rusage. The trace column is callback '
        'wall time, and the host had no kernel traces.',
    ),
    (
        'rq1-loss-row-overload',
        'The rcllf row of the loss evaluation is "detected loss under sustained overload", not a determinism '
        'row. The per-ID gate detects every missing ID. With `take_timeout_ms` 5000 every ID is '
        'delivered, at a latency cost.',
    ),
    (
        'legacy-edge-trigger-only',
        'A legacy edge carries only a trigger, and the subscriber takes whatever sample its '
        'reader holds when the reaction runs. Its value at a tag therefore depends on the '
        'wall-clock arrival of samples. A receiver that runs behind the sender in wall time can '
        'take a later sample at an earlier tag, and a sender that runs behind the receiver can '
        'leave an earlier sample in the reader. Every topic edge that '
        'correspondence admission refuses with a reason is such an edge, for example '
        "`transient_local`, a fan-in group whose members' delays differ "
        '(`fan_in_release_mismatch`) or `burst_unbounded`. Samples are not filtered by stamp.',
    ),
    (
        'fan-in-groups',
        'Several publishing nodes into one subscription are delivered in one order: by release '
        "tag, then by the rank of the publisher's LF instance name, then in publication order "
        '. A fan-in group is admitted or stays legacy as a whole. It stays legacy, '
        'every member with a trigger-only edge, when a member is refused for any reason (for '
        "example `transient_local` or `burst_unbounded`), when the members' delays differ "
        '(`fan_in_release_mismatch`, for a deferred group also their chained timeouts), when '
        'some members cross federates and others do not (`fan_in_mixed_locality`), when a '
        "member's publisher is the subscribing node (`fan_in_self_loop`), when a member rebinds "
        'with a lifecycle node (`fan_in_rebind`) or when one node publishes the topic from two '
        'publisher objects (`fan_in_shared_publisher`). A group into a BoundedNearestToAnchor '
        'input is refused (`fan_in_bna_input`) and generation stops with `RCLLF-H003`. A lost '
        "sample holds the group's drain until its cutoff, so the later samples of the other "
        'publishers wait behind it in physical time, at unchanged tags. A writer outside the '
        'analyzed graph, such as `ros2 topic pub`, is never delivered (`UNRESOLVED`). Under '
        'decentralized coordination the order holds only while every record arrives within its '
        "STP window, and a tardy record's missing dispatch can count as an order violation that "
        'ends the run with exit code 3. Known limitation: this case has no experiment.',
    ),
    (
        'message-filters-sync-domain',
        'A two-input `message_filters` ExactTime or ApproximateTime synchronizer is claimed on '
        'the domain `mf-sync/1` only: both inputs are admitted correspondence edges with one '
        'link delay, at most the declared burst per tag, and `message_filters` 4.11.17 with '
        '`rclcpp` 28.1.22. On it the converted synchronizer emits the same signal and drop '
        'stream as the stock header run off-line on the LF arrival order (tick, then input 0 '
        'before input 1, then publish order), which is not the arrival order of a stock ROS run, '
        'so the claim is "matches message_filters 4.11.17 for the LF arrival order", never '
        '"equivalent to stock ROS". An input that is a legacy edge, including an '
        'input from outside the analyzed graph (`external`), refuses generation with `RCLLF-H003 '
        'sync_input_legacy:<reason>`, unequal input delays with `sync_input_delay_unequal`. A '
        'lost or overflowing sample is reported `out_of_domain`, not compared. Rate bounds '
        '(`setInterMessageLowerBound`, `setMaxIntervalDuration`), three or more inputs and '
        '`LatestTime` stay refused.',
    ),
)


def limits() -> dict[str, str]:
    """Limit id (L1, L2, ...) to its statement."""
    return {f"L{index}": text for index, (_, text) in enumerate(LIMIT_ROWS, start=1)}


def limit_ids() -> dict[str, str]:
    """Limit slug to its id."""
    return {slug: f"L{index}" for index, (slug, _) in enumerate(LIMIT_ROWS, start=1)}


def problems() -> list[str]:
    """Every row of ``LIMIT_ROWS`` that cannot be rendered as one table row."""
    out: list[str] = []
    seen: set[str] = set()
    for index, (slug, text) in enumerate(LIMIT_ROWS, start=1):
        if slug in seen:
            out.append(f"limit slug '{slug}' appears twice in LIMIT_ROWS (second at L{index})")
        seen.add(slug)
        if not text.strip():
            out.append(f"limit '{slug}' (L{index}) has an empty statement")
        if "\n" in text:
            out.append(f"limit '{slug}' (L{index}) has a line break in its statement")
        if any(text[i] == "|" and (i == 0 or text[i - 1] != "\\") for i in range(len(text))):
            out.append(f"limit '{slug}' (L{index}) has an unescaped '|' in its statement")
    return out
