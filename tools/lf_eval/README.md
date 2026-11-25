# lf_eval

Trace analysis for rcllf: reads LTTng traces and computes latency, CPU,
determinism and chain metrics. Entry points are the `lf-eval` command (also `python -m lf_eval`) and the
`ros2 lf eval` and `ros2 lf report` verbs.

Some modules (`runset`, `output_check`, `chain_latency`, `results_report`) serve
the benchmark orchestrator that lives in the development repository. They work
on any run directory with the documented layout. Tests that need recorded
fixtures from that orchestrator skip themselves when the fixtures are absent
(see `tests/dev_only.py`).
