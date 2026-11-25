# The end-of-run COMPLETE count of one cross-federate edge.
#
# A COMPLETE message lost after the receiver's last one leaves no sequence gap,
# so the receiver alone cannot see it. After Lingua Franca returned, the
# sending federate publishes this on /rcllf/corr_end (reliable, transient
# local), and the receiving federate's reconcile fails unless it arrived and
# every completion number up to `completes_due` arrived before it.

# Run epoch of the sending process, for the trace.
uint64 sender_epoch
# The system-wide id of the edge (the generator's global edge id).
int32 global_edge
# Completion numbers the sender gave out.
uint64 completes_sent
# Of those, the ones that land at or before the stop tag (sent minus at stop).
uint64 completes_due
