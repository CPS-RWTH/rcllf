# A full Lingua Franca tag, captured at a single instant by the LF runtime.
#
# `logical` and `microstep` together form the LF tag (g, m). Two reactions
# at the same `logical` but different `microstep` are logically simultaneous
# but ordered by microstep within the same instant -- what /clock alone
# cannot represent.
#
# `wall` is the wall-clock instant (RCL_SYSTEM_TIME) at which the runtime
# emitted this tag. Use it to correlate LF logical time with physical
# execution time for debugging, profiling, and recordings.

builtin_interfaces/Time logical
uint32                   microstep
builtin_interfaces/Time  wall
