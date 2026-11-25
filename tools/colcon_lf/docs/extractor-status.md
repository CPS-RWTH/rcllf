# Extractor Gap Status

Which C++ constructs the extractor reads, and which it does not yet.

Snapshot of 2026-04-21. Later releases added extractor features that this table
does not track; the authoritative list is the support matrix
(`docs/features/support-matrix.md`) and the module registry
(`colcon_lf/features/`).

Legend: ✓ present | ± partial | ✗ missing

## TimerInfo

| Field | Status | Notes |
|-------|--------|-------|
| `period_ms` | ✓ | |
| `offset_ms` | ✓ | Extracted from 3rd arg of `create_wall_timer`/`create_timer` |
| `callback` | ✓ | |
| `callback_group` | ± | Name captured; kind via CallbackGroupInfo lookup |
| `created_in` | ✓ | |
| `destroyed_in` | ✓ | |

## PublisherInfo

| Field | Status | Notes |
|-------|--------|-------|
| `topic` | ✓ | |
| `message_type` | ✓ | |
| `qos_depth` | ✓ | Integer only |
| `qos_reliability` | ✓ | Extracted from chained `rclcpp::QoS` calls |
| `qos_durability` | ✓ | Extracted from chained `rclcpp::QoS` calls |
| `qos_history` | ✓ | Extracted from chained `rclcpp::QoS` calls |
| `qos_deadline_ms` | ✓ | Extracted from `.deadline(...)` chained call |

## SubscriberInfo

Same QoS fields as PublisherInfo — all ✓. `callback_group` name present via field.

## ServiceClientInfo

| Field | Status | Notes |
|-------|--------|-------|
| `service_name` | ✓ | |
| `service_type` | ✓ | |
| `response_callback` | ± | Captured when async pattern used |
| `call_style` | ✓ | Classified as `async_callback`/`blocking_spin`/`blocking_future`/`None` |

## TopicConnection

| Field | Status | Notes |
|-------|--------|-------|
| `topic` | ✓ | |
| `message_type` | ✓ | |
| `publishers` | ✓ | |
| `subscribers` | ✓ | |
| `after_delay_ns` | ✓ | Carried on model; populated by generator (default 0, no logical delay, when not set) |

## CallbackGroupInfo

| Field | Status | Notes |
|-------|--------|-------|
| `name` | ✓ | |
| `type` (MutuallyExclusive/Reentrant) | ✓ | Populated by clang_parser |
| Per-callback membership | ± | Timer/sub/service carry `callback_group` name; requires join with `CallbackGroupInfo` list by name |

## Out of scope (confirmed not needed for Phase 1-3)

- Lifecycle callbacks — deferred
- Dynamic sub/pub creation — deferred
- Intra-process eligibility — deferred
