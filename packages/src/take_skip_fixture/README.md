# take_skip_fixture

CI fixture for take-mode skip subscriptions. Writers publish `std_msgs/Int64`
IDs on `take` every 100 ms (from tick 5 on); one reader takes them through a
skip-mode subscription. Runtime tests drive it to check what rcllf does with a
sample that arrives late, never arrives, or arrives in a burst.

| Launch file | System |
|---|---|
| `take_one.launch.py` | one writer, one reader |
| `take_fan.launch.py` | two writers (`id_base` 0 and 1000) fan in to one reader |

## Fault hooks

The writer reads these environment variables. They apply only to the node
named in `TAKE_SKIP_INJECT_NODE`; unset means no fault.

| Variable | Effect |
|---|---|
| `TAKE_SKIP_INJECT_NODE` | name of the writer node that applies the hooks below |
| `TAKE_SKIP_LATE_IDS` | IDs published `TAKE_SKIP_DEFER_MS` later from another thread |
| `TAKE_SKIP_DEFER_MS` | delay of a late ID in ms (default 50) |
| `TAKE_SKIP_LOST_IDS` | IDs that are never published |
| `TAKE_SKIP_BURST_IDS` | IDs published `TAKE_SKIP_BURST_N` times at their tick |
| `TAKE_SKIP_BURST_N` | burst size (default 15, above the reader's KeepLast depth) |

A late or lost ID still counts as a write at its tick, as a sample late or lost
in transit does.

```bash
colcon lf-analyze --package take_skip_fixture --launch-file take_one.launch.py
```
