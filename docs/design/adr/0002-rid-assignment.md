# ADR 0002, Reaction ID (RID) Assignment and Stability

**Date:** 2026-04-21  
**Status:** Accepted

## Context

The lfc path requires a shared vocabulary between two independently-compiled
artefacts:

1. **The `.lf` source** (and the C++ it compiles into): reaction bodies call
   `ros2_lf_invoke_timer(self, RID_X)` where `RID_X` is a compile-time integer.
2. **`ros2_lf_binding`**: at runtime, `ros2_lf_bind_node` populates
   `g_handle_table[RID_X]` with the live ROS 2 timer or subscription handle.

Both artefacts must agree on which integer refers to which (node, callback) pair.
If they disagree, for example because `system.json` was regenerated with a different
node order, the reaction fires the wrong callback or a null handle.

## Decision

**RIDs are assigned once, deterministically, from `system.json`, and must not
change unless `system.json` changes.**

### Assignment algorithm

1. Iterate `system.json` nodes in the order they appear in the file
   (extraction preserves the launch-file declaration order).
2. Within each node, iterate callbacks in C++ source-file definition order
   (libclang AST pre-order, the order `clang_parser.py` visits declarations).
3. Assign the next available integer, starting from 0.

Note (2026-10-21): step 2 describes the plan of this record. The generator
assigns RIDs in the declaration order of each node's reactor, which puts subscription
callbacks before timer callbacks (source order within each group), and
`execution.yaml` `reaction_order: timers_first` puts timer callbacks first. So the RIDs
of a node depend on `system.json` and on that key.
`timers_first` is the default, so timer callbacks come first unless a node sets
`reaction_order: subscriptions_first`.

The resulting mapping is emitted as `#define RID_<NodeClass>_<callback>` macros
in `ros2_lf_reaction_ids.h` and as `RidEntry` registrations in
`ros2_lf_reaction_ids_init.cpp`.

### Stability guarantee

Given the same `system.json` (same node list, same order, same callback lists),
re-running `colcon lf-generate` produces identical RID values. Stability breaks
if and only if:

- A node is added, removed, or reordered in `system.json`.
- A callback is added, removed, or reordered within a node's C++ source.

When stability breaks, regenerate both `ros2_lf_reaction_ids.h` and
`ros2_lf_reaction_ids_init.cpp` and rebuild `lf_generated_<package>_<stem>`. The binding
library itself (`ros2_lf_binding.so`) does not embed RIDs and does not need
rebuilding.

### `RID_COUNT`

`RID_COUNT` is the total number of (node, callback) pairs. It is used to size
`g_handle_table`:

```c
#define RID_COUNT  3   // example: 1 timer + 2 subscriptions
```

`resize_handle_table(RID_COUNT)` is called during static init (before `main()`).

## Consequences

**Accepted:**

- RIDs are small integers, so `g_handle_table` is a flat vector with O(1) lookup,
  no hashing, no string comparison at reaction-dispatch time.
- Regeneration is cheap and deterministic. Tools that diff `system.json` before
  regenerating (future work) can skip regeneration when nothing changed.
- The macro names (`RID_<NodeClass>_<callback>`) serve as human-readable
  documentation in generated reaction bodies.

**Known limitations:**

- RID stability depends on extraction determinism. If `clang_parser.py`
  iterates declarations in a platform-dependent order, RIDs could differ
  between machines. In practice, libclang's AST pre-order is deterministic
  for a given source file.
- There is no versioning of the RID table. If a binary built against an old
  `ros2_lf_reaction_ids.h` is run with a binding library populated from a new
  `ros2_lf_reaction_ids_init.cpp`, dispatch will silently invoke the wrong
  callbacks. The diagnostic is unambiguous (`[WARN] ros2_lf_bind_node: no
  loaded component matches class ...`) but not fatal. Future work: embed a
  hash of `system.json` in both artefacts and assert equality at startup.
