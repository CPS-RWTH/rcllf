# correspondence_fixture

Publisher/subscriber fixture used by the runtime tests of
(publication correspondence: which topics a reaction actually publishes to,
under fan-out, fan-in and out-of-band publication).

## Nodes

- `CorrPublisher` (`corr_publisher`): one 200ms timer (`on_tick`) driving
  publishes on `/corr/a` (`pub_a_`) and `/corr/b` (`pub_b_`), both
  `std_msgs::msg::Header`. Behavior is selected by the `CORR_SCENARIO` env
  var (default `one`):
  - `zero`: never publishes.
  - `one`: `pub_a_` publishes once per tick.
  - `many`: `pub_a_` publishes 3 identical messages per tick.
  - `conditional`: `pub_a_` publishes on odd ticks only.
  - `two_publishers`: both `pub_a_` and `pub_b_` publish once per tick.
  - `stale`: like `one`, plus a helper thread that publishes one message
    (`frame_id="H"`) on `pub_a_` as soon as a subscriber is discovered,
    before tick 1 fires.
  - `fanout`: same as `one` (fan-out comes from the launch topology).

  Publishing stops after tick 10 (the timer keeps running).

- `CorrSubscriber` (`corr_sub_1`): subscribes `/corr/a` and `/corr/b`.
- `CorrSubscriberA` (`corr_sub_2`): subscribes `/corr/a` only.
- `CorrFaninRelay` (`corr_fanin_relay`): subscribes `/corr/a`, then `/corr/b`, and
  republishes every `/corr/b` sample as `j-<payload>` on `/corr/j`.
- `CorrTwoGroupPublisher` (`corr_two_group_publisher`): two MutuallyExclusive
  callback groups publish `/corr/a`. `on_a` (200ms) publishes `A1`..`A6`,
  `on_b` (310ms) publishes `B1`..`B4`, both skip their first tick.
- `CorrReentrantPublisher` (`corr_reentrant_publisher`): the same two timers in one
  Reentrant callback group.
- `CorrStopPublisher` (`corr_stop_publisher`): publishes `/corr/a` payloads `t1k1`..`t5k1`
  every 200 ms and calls `rclcpp::shutdown()` right after `t5k1` in the same callback.
  The LF stop is requested from the reaction that published last.

## Launch files

- `corr_bringup.launch.py`: all three nodes, no remaps. `/corr/a` fans out
  to `corr_sub_1` and `corr_sub_2`; `/corr/b` goes to `corr_sub_1` only.
- `corr_remap.launch.py`: corr_publisher and corr_sub_2, both remapping `/corr/a` to
  `/corr/a_remapped`. Port names stay the declared names, the edge follows
  the remapped topic.
- `corr_loop.launch.py`: corr_ping's timer publishes `/corr/ping`, corr_pong answers
  on `/corr/pong`, corr_ping's `/corr/pong` callback only logs. A zero-delay loop with
  no causality loop: both links are admitted and deliver at the publication tag.
- `corr_loop_echo.launch.py`: corr_ping_echo's `/corr/pong` callback publishes one more
  ping, so every subscription on the loop publishes onward into it. A causality loop:
  `colcon lf-generate` refuses it and names both links.
- `corr_loop_echo_delayed.launch.py`: the same loop with the `/corr/pong` link at
  d = 1 msec (`lf/corr_loop_echo_delayed.execution.yaml`), which breaks it, so it runs.
- `corr_shared_join.launch.py`: corr_publisher (`CORR_SCENARIO=two_publishers`), corr_sub_2
  with a 150 ms take timeout, corr_fanin_relay and corr_relay_sink on `/corr/j`.
  A lost `/corr/a` sample of corr_sub_2 holds the tag of the `/corr/j` publish.
- `corr_two_group.launch.py`, `corr_reentrant.launch.py`: the matching publisher and
  corr_sub_2.

## Run

```bash
CORR_SCENARIO=many ros2 launch correspondence_fixture corr_bringup.launch.py
```
