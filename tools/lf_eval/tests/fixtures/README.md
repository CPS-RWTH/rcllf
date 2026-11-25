# lf_eval test fixtures: provenance

Every fixture listed here was cut from a real run's LTTng trace. None holds a
message payload: the events keep only trace event names, timestamps, process and thread ids, CPU numbers,
handle and pointer values, queue depths and QoS settings, and topic, node and
callback names. Tests read them and never change them.

| Fixture | Source run | What it keeps | Used by |
|---|---|---|---|
| `overwrites/refsys_stock_euclidean_cluster_settings.json.gz` | `20261009T194016Z_stock-launch-per-node_refsys_default_r0`: stock ROS 2, one process per node, of the Autoware reference system ([ros-realtime/reference-system](https://github.com/ros-realtime/reference-system), Apache-2.0) | the 2328 events of topic `/EuclideanClusterSettings`: node, publisher and subscription inits, `rcl_publish`, `rmw_publish`, `rmw_take` | `test_output_check.py` |
| `overwrites/refsys_stock_euclidean_cluster_detector_gap.json.gz` | `20261014T092620Z_stock-launch-per-node_refsys_default_r17`, same system and launch | the 749 events of topic `/EuclideanClusterDetector`, as above plus the match probe's `writer_matched` and `reader_matched` | `test_output_check.py` |
| `rslcpp_intra_648.json` | `20261010T113448Z_rslcpp_default_r1` of campaign `w6f-full/rq4-sentry-pipeline_20261010T111651Z`: rcllf's own `sentry_pipeline` nodes under the rslcpp executor ([TUMFTM/rslcpp](https://github.com/TUMFTM/rslcpp), Apache-2.0) | 396 events in a window with 6 sink callbacks (`provenance.window_ns`), plus the analyzed `system` description of the pipeline | `test_chain_latency.py` |

`overwrites/extract.py` cut both `overwrites/` files from their run directories
(`python3 extract.py <run_dir> <topic> <out.json.gz>`). `rslcpp_intra_648.json` was cut
by a one-off script that is not kept; its `provenance` block names the
run and the window.

The reference-system fixtures carry that project's node and topic names, nothing of
its code. The source runs themselves are not part of the repository.
