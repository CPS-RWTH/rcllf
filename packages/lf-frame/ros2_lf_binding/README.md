# ros2_lf_binding

The runtime binding that connects Lingua Franca reactors to ROS 2 nodes. The
generated code of `colcon lf` links against it. It also ships the publication,
clock and action gate sources that every generated executable compiles in.

## Optional diagnostics

Both diagnostics are off by default and change no behaviour while off.

### Read-back of the effective settings (`RCLLF_READBACK_DIR`)

Set `RCLLF_READBACK_DIR=<dir>` before starting a generated program. At its
first startup reaction the binding writes `rcllf_readback.<pid>.json` into
that directory (schema `rcllf.readback/1`). The file holds the RID table as
registered, the timers reactor-c fires, the LF worker count, the reactor-c
patch markers compiled in, and the scheduling policy of every thread. Use it to
check that a run uses the settings you configured, from the live process and
not from its configuration. The directory must exist.

### DDS match probe (`librcllf_match_probe.so`)

A `VOLATILE` sample published before DDS matched the reader to the writer is
never delivered, and the ROS 2 tracepoints do not show when matching happens.
The match probe is an `LD_PRELOAD` library that polls every reader and writer
and emits `rcllf_dds:reader_matched` and `rcllf_dds:writer_matched` LTTng
events. It works in any ROS 2 process, rcllf or stock. It is built only when
lttng-ust is found at configure time.

```bash
LD_PRELOAD=$(ros2 pkg prefix ros2_lf_binding)/lib/librcllf_match_probe.so \
  ros2 run <package> <executable>
```

Record the events with an LTTng session that enables the `rcllf_dds:*`
tracepoints. Set `RCLLF_MATCH_PROBE_EVENTS=0` to use the poll alone instead of
the rmw matched event callback that rmw_fastrtps offers.
