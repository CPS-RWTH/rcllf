# action_full_example

Full action lifecycle exercise: decoupled feedback timer, goal rejection via parameter,
mid-flight cancellation, and multiple sequential goals.

## Nodes

- **long_running_action_server** — 50-step execution loop at 10 Hz (5 s total).
  A separate 5 Hz feedback timer publishes progress, decoupled from the execution rate.
  Rejects goals when the `reject_goals` parameter is `true`.
- **action_full_client** — sends three sequential goals:
  - **Goal A** (`analysis_type="complete"`) — runs to completion.
  - **Goal B** (`analysis_type="cancel_me"`) — client cancels after 2 s.
  - **Goal C** (`analysis_type="rejected"`) — client sets `reject_goals=true` via
    parameter service before sending; server rejects immediately.

## Purpose

Targets converter gaps in `lf_generator.py` action paths (no `cancel_callback` reaction,
no rate-aware feedback) and the binding's single `entity_id` per action waitable
(`reaction_dispatch.cpp:235`).

## Usage

```bash
ros2 launch action_full_example bringup.launch.py
```

Expected log sequence:
1. Goal A: feedback at ~5 Hz for ~5 s, then `SUCCEEDED`.
2. Goal B: feedback for ~2 s, then `CANCELED`.
3. Goal C: immediate `goal_response_callback` with null handle → `REJECTED` logged.

## LF pipeline

```bash
colcon lf-analyze --package action_full_example
colcon lf-generate --target-lfc
```

The analyzer should produce `system.json`. The generator may warn about the decoupled
feedback timer and cancel path — these document known converter gaps.
