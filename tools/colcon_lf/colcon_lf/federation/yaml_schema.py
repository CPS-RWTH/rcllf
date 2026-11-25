# Copyright 2026 rcllf Contributors
# Licensed under the Apache License, Version 2.0

"""Constants for the federation YAML schema."""

# Top-level keys
KEY_SYSTEM = "system"
KEY_FEDERATES = "federates"
KEY_AFTER_OVERRIDES = "after_overrides"
KEY_COORDINATION = "coordination"

# Federate sub-keys
KEY_NODES = "nodes"

# after_overrides sub-keys
KEY_FROM = "from"
KEY_TO = "to"
KEY_AFTER = "after"
KEY_STP_OFFSET = "stp_offset"
# Decentralized consistency budget (item5 piece D): d + W >= L + eps + X + W_send
# per link, checked at generation. Decentralized coordination only.
KEY_ABSENT_AFTER = "absent_after"  # alias of stp_offset
KEY_MAX_LATENCY = "max_latency"  # L
KEY_CLOCK_ERROR = "clock_error"  # eps
KEY_MAX_EXEC = "max_exec"  # X
BUDGET_KEYS = (KEY_MAX_LATENCY, KEY_CLOCK_ERROR, KEY_MAX_EXEC)

# Sentinel mode values for --federate flag
MODE_PER_PROCESS = "per-process"
MODE_PER_NODE = "per-node"

# Valid values for the `coordination` key / --coordination flag
COORDINATION_CENTRALIZED = "centralized"
COORDINATION_DECENTRALIZED = "decentralized"
VALID_COORDINATION_MODES = (COORDINATION_CENTRALIZED, COORDINATION_DECENTRALIZED)
