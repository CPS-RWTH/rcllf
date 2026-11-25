# sync_oracle_showcase

Showcase and oracle for the `message_filters` synchronizer domain **mf-sync/1**.
A converted `ExactTime` or `ApproximateTime` synchronizer is
compatible with stock `message_filters` when, fed the same rows in the
lf_arrival_order, it emits the same signal and drop stream as the stock header
run off-line (oracle O1). User docs:
`docs/features/message-filters.md`.

## Contents

| Path | What |
|---|---|
| `corpora/*.yaml` | Input corpora with hand-derived `expect` blocks (format in `include/sync_oracle_showcase/corpus.hpp`) |
| `src/corpus_player.cpp` | Publishes a corpus, one tick per 50 ms timer firing, all rows of a tick in one callback |
| `src/exact_sync_node.cpp`, `src/approx_sync_node.cpp` | One synchronizer each (param `queue_size`), log `RCLLF_SYNC {json}` per signal and drop |
| `src/branch_worker.cpp` | P3 fork/join branch: forwards each row after `work_ms` of busy work (`SYNC_WORK_SWAP=1` swaps the two branches' work) |
| `test/sync_oracle.cpp` | O1: the stock header run off-line on the lf_arrival_order (`--impl ref` and `--mut <name>` for the mutants) |
| `launch/corpus.launch.py` | Player plus five instances: `exact_q1/2/3`, `approx_q2/3` |
| `launch/forkjoin.launch.py` | P3: `fork_source` -> `branch_0`, `branch_1` -> `join` (ExactTime, q 3) |

| Corpus | Purpose |
|---|---|
| P1 | Paced pairs, one per tick: the stock-ROS reference (S6) and the drop-sample LOST check |
| P2 | Timer schedule (a every 30 ms, b every 50 ms), expected pairs derived by the ApproximateTime port |
| B1-B8 | ExactTime boundaries: queue overflow, overwrite, out-of-order stamps, clearOld, same-tick bursts |
| A1-A4 | ApproximateTime boundaries (pivot, max interval, inter-message bound) |
| F1 | Falsification corpus: each mutant of O1 differs on it |
| N1 | Out of domain (more publishes per invocation than declared): must report `out_of_domain` |
| P3 | Fork/join: the join output must not depend on branch work |

## Capture commands (dev container)

```bash
colcon lf --package sync_oracle_showcase       # analyze, generate, build
source install/setup.bash
O1=install/sync_oracle_showcase/lib/sync_oracle_showcase/sync_oracle
C=src/sync_oracle_showcase/corpora/b8.yaml

# Converted run: every instance logs RCLLF_SYNC records, the drains log the arrival order.
SYNC_CORPUS=$PWD/$C RCLLF_TRACE_CALLBACKS=1 timeout -s INT 12 \
  ros2 launch lf_generated_sync_oracle_showcase_corpus lf_corpus.launch.py > run.log 2>&1

# Oracle O1 per instance, then the comparison (exit 1 on any report).
$O1 --corpus $C --policy exact --queue 1 > o1_exact_q1.jsonl
python3 -m lf_eval.sync_oracle --corpus $C --log run.log --o1 exact_q1=o1_exact_q1.jsonl

# Fork/join (P3), with and without swapped branch work.
SYNC_CORPUS=$PWD/src/sync_oracle_showcase/corpora/p3.yaml SYNC_WORK_SWAP=1 \
  RCLLF_TRACE_CALLBACKS=1 timeout -s INT 12 \
  ros2 launch lf_generated_sync_oracle_showcase_forkjoin lf_forkjoin.launch.py > p3.log 2>&1
```

The comparator prints `<node>: compatible` or the first report:
`version_unpinned`, `config_mismatch`, `out_of_domain`, `arrival_order_mismatch`,
`tuple_mismatch` or `classification_mismatch`. The automated version is
`tools/colcon_lf/tests/runtime/test_runtime_sync_oracle.py`.
