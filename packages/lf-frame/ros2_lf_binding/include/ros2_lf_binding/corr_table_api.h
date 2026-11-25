// Copyright 2026 rcllf Contributors
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/// @file corr_table_api.h
/// @brief Generated-code API for the correspondence edge table.
///
/// The generated ros2_lf_reaction_ids_init.cpp registers every topic edge
/// before ros2_lf_runtime_init(): admitted edges get the correspondence path,
/// legacy edges are listed with the generation-stage reason.

#ifndef ROS2_LF_BINDING__CORR_TABLE_API_H_
#define ROS2_LF_BINDING__CORR_TABLE_API_H_

#include <cstdint>
#include <string>
#include <vector>

namespace ros2_lf_binding
{

/// Where the edge's other end runs. kEgress: the subscriber
/// is in another federate (no sub_key node here, sub_rid -1). kIngress: the
/// publisher is (no publishing RIDs here).
  enum class CorrEdgeRole : uint8_t {kLocal = 0, kEgress = 1, kIngress = 2};

  struct CorrEdgeSpec
  {
    int edge_id;               ///< Dense from 0, the `edge` argument of ros2_lf_corr_*.
    std::string pub_key;       ///< Binding key of the publishing node.
    std::vector < int > pub_rids;  ///< RIDs of every publishing callback, non-empty.
    std::string pub_topic;     ///< Node-local topic name at the publisher (analyzer).
    std::string sub_key;       ///< Binding key of the subscribing node.
    int sub_rid;               ///< RID of the subscription callback.
    std::string sub_topic;     ///< Node-local topic name at the subscriber (analyzer).
    int64_t delay_ns;          ///< Resolved after-delay d of the LF link it replaces.
    // S2 fills these from the schedule; -1 / 0 select the defaults.
    int64_t timeout_ns = -1;   ///< T_e; -1: ROS2_LF_MAXWAIT_MS, else the subscription policy
    int64_t max_lag_ns = -1;   ///< L_e; -1: 2 * max T_e
    uint32_t burst = 0;        ///< B_e, records per reaction; 0: not checked
    uint32_t retain = 0;       ///< Retention bound N_e(d_e + L) + 1; 0: kRetainPerEdge
    uint32_t carrier = 0;      ///< Carrier bound N_e(d_e); 0: kCarrierCap
    /// KEEP_LAST depth the edge's reader gets at least (exec-config
    /// reader_depth); 0: kRetainPerEdge.
    uint32_t reader_depth = 0;
    CorrEdgeRole role = CorrEdgeRole::kLocal;
    int global_id = -1;        ///< system edge id, names the xcorr_g<G> connection
    std::string peer_node {};   ///< fully qualified name of the node at the other end
    /// Reliability the analysis saw on the subscription ("reliable",
    /// "best_effort"); empty when the analysis has no value.
    std::string sub_reliability {};
    /// KEEP_LAST depth the analysis saw on the subscription; -1 when
    /// the analysis has no value.
    int sub_depth = -1;
    /// The publishing node is a lifecycle node. Startup records the
    /// edge's publisher instead of refusing a missing one, and the gate binds
    /// every publisher the node creates on the topic, also after startup.
    bool pub_rebind = false;
    /// The subscribing node is a lifecycle node that makes or resets
    /// the subscription in a hook. Startup defers the reader; each reader the
    /// node creates is checked and admitted when its RID binds it.
    bool sub_rebind = false;
    /// exec-config `join: deferred`, set by set_corr_edge_deferred.
    bool deferred = false;
    /// An in-process d = 0 edge whose drain is triggered by the record
    /// port itself, set by set_corr_edge_same_tag.
    bool same_tag = false;
    /// A decentralized ingress edge, set by set_corr_edge_tardy_drop.
    bool tardy_drop = false;
    /// exec-config `control_failure_policy: stop`. A missing COMPLETE
    /// record on this ingress edge requests an LF stop after the report.
    /// Set by set_corr_edge_control_stop.
    bool control_stop = false;
    /// The lowest edge id of the edge's fan-in group (every admitted
    /// edge into one subscription callback on one topic), -1 outside a group.
    /// Set by set_corr_edge_fan_in.
    int group_lead = -1;
    /// The publisher's position in the group's instance-name order,
    /// which orders the members' messages at one delivery tag.
    uint32_t fan_in_rank = 0;
  };

  void register_corr_edge(CorrEdgeSpec spec);
/// The subscriber of admitted edge `edge_id` joins deferred: its drain runs
/// at the cutoff tag and never waits for an absent sample. Call after
/// register_corr_edge of that edge.
  void set_corr_edge_deferred(int edge_id);
/// Admitted local edge `edge_id` (two nodes in this process, d = 0, an
/// immediate join) is drained by a reaction its record port triggers, so a
/// record releases at its publication tag (t, m), not (t, m + 1). Refused at
/// startup on a self, cross-federate, delayed or deferred edge. Call after
/// register_corr_edge of that edge.
  void set_corr_edge_same_tag(int edge_id);
/// Admitted ingress edge `edge_id` runs over a decentralized link. A
/// record that arrives after its intended tag (publish tag + d) is dropped and
/// its message reported lost, not fatal. Call after register_corr_edge.
  void set_corr_edge_tardy_drop(int edge_id);
/// A CONTROL_FAILURE (a sender activation without a COMPLETE record) on
/// admitted ingress edge `edge_id` requests an LF stop, not only a report.
/// Call after register_corr_edge of that edge.
  void set_corr_edge_control_stop(int edge_id);
/// Admitted edge `edge_id` is the member of rank `rank` in the fan-in group
/// led by edge `lead`. The members share one reader, one drain and one delivery
/// order. Call after register_corr_edge of that edge, for every member of a
/// group of two or more; a lone edge has no call.
  void set_corr_edge_fan_in(int edge_id, int lead, uint32_t rank);

/// The spec of every registered edge, as the runtime schedules it
/// (index = edge id; an id never registered gives an entry with an empty pub_key).
  std::vector < CorrEdgeSpec > registered_corr_edges();

/// A topic edge the generator left on the legacy path, with its reason:
/// the subscription's node-local topic and the reliability and depth the
/// analysis saw on it (empty, -1: no value), for the startup QoS check.
  void register_corr_legacy_edge(
    const std::string & topic, const std::string & pub_key, const std::string & sub_key,
    const std::string & reason, const std::string & sub_topic = "",
    const std::string & sub_reliability = "", int sub_depth = -1);

/// A legacy cross-federate edge read in this federate. The publisher's
/// federate lists it; here it only feeds the startup QoS check of its reader
/// (RCLLF-H001 names the publisher, RCLLF-H002), not the legacy edge count.
  void register_corr_legacy_ingress(
    const std::string & topic, const std::string & pub_key, const std::string & sub_key,
    const std::string & reason, const std::string & sub_topic = "",
    const std::string & sub_reliability = "", int sub_depth = -1);

/// One publisher of node `key` on its node-local `topic` with the
/// reliability the analysis saw ("reliable", "best_effort", empty: no value).
/// Startup logs RCLLF-H000 with the effective reliability where it is empty.
  void register_publisher_qos(
    const std::string & key, const std::string & topic, const std::string & reliability);

}  // namespace ros2_lf_binding

#endif  // ROS2_LF_BINDING__CORR_TABLE_API_H_
