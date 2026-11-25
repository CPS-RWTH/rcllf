# sync_oracle_showcase corpus sample: one row of a sync corpus.
#
# header.stamp is the row's stamp_ns, the key message_filters synchronizes on.
std_msgs/Header header
# Synchronizer input the row is addressed to (0 or 1).
uint32 input
# 0-based row index in the corpus file: the sample's identity.
uint64 row
# Opaque value; equal payloads on distinct rows test identity by row (B6).
uint64 payload
