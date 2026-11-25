# brake_fusion /brake_cmd: one brake command with its identity.
#
# level is the FIRST field on purpose: a drop-injection shim's f64_first_ge rule
# reads the leading float64 of the taken
# C++ message, as it did for std_msgs/Float64, so the brake loss cells keep
# naming the same command.
float64 level
# 1-based count of the commands the controller decided, injected drops
# included, so a missing seq at the simulator is a command that never arrived.
uint64 seq
# The controller tick that decided it (ControllerNode tick_ or the grid index).
int64 tick
# The controller's clock at the decision: the tick's logical tag under rcllf,
# the node's ROS clock under stock ROS 2.
int64 stamp_ns
