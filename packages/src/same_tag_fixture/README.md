# same_tag_fixture

Chain test: a zero-delay connection must deliver the first record of a
tag at the publication tag (t, m), as an LF connection without `after` does. Then one
node can be replaced by a chain of two nodes without moving any downstream tag.

## The graph (one process, every connection d = 0, no execution config)

| Node | Executable | Does |
|---|---|---|
| `tick` | `st_source` | 200 ms timer, ticks 1 to 10: publishes `/a` = tick, `/c` = 2000 + tick, `/d` = 1000 + tick, and three records on `/m` = 3000 + 10 tick + k (k = 0, 1, 2) |
| `relay` (ONE) | `st_stage` | `/a` to `/c`, value 3 a + 7 |
| `pre` + `relay` (CHAIN) | `st_stage_in`, `st_stage_out` | `/a` to `/b` (3 a), `/b` to `/c` (b + 7): the same computation split in two |
| `sink` | `st_sink` | subscribes `/c` (fan-in of `relay`, rank 0, and `tick`, rank 1) and `/d` |
| `msink` | `st_multi_sink` | subscribes `/m` (three records per tag) |

- `launch/st_one.launch.py`: ONE variant. `launch/st_chain.launch.py`: CHAIN variant.
- The fan-in ranks follow the lexicographic order of the LF instance names, so
  `relay` (rank 0) is delivered before `tick` (rank 1) at an equal release tag.

## Expected tags (same-tag delivery, tick t at elapsed (t - 1) 200 ms = T)

| Subscription | Records | Delivery tags |
|---|---|---|
| stage(s) `/a`, `/b` | one | (T, 0) |
| `sink` `/c` | `relay` then `tick` | (T, 0), (T, 1) |
| `sink` `/d` | one | (T, 0) |
| `msink` `/m` | three | (T, 0), (T, 1), (T, 2) |

ONE and CHAIN must give the sink identical tags and values. In earlier versions every first
record lands one microstep after its publication, so the stage chain moves `/c`
(ONE (T, 2), CHAIN (T, 3)) and the fan-in delivers `tick` before `relay`.

## Run

```bash
cd packages
LF_LAUNCH=st_one bash ../docker/lf_build.sh same_tag_fixture
LF_LAUNCH=st_chain bash ../docker/lf_build.sh same_tag_fixture
source install/setup.bash
cd ../tools/colcon_lf && python3 -m pytest tests/runtime -v -m runtime -k same_tag
```

The test is `tools/colcon_lf/tests/runtime/test_runtime_same_tag.py`.
