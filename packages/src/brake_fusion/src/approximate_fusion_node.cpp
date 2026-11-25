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

#include "brake_fusion/approximate_fusion_node.hpp"

#include <chrono>
#include <cinttypes>
#include <cstdlib>
#include <functional>
#include <memory>
#include <string>
#include <thread>
#include <rclcpp_components/register_node_macro.hpp>

#include "brake_fusion/loss_events.hpp"

namespace brake_fusion
{

namespace
{
int64_t stamp_ns(const sensor_msgs::msg::Range::ConstSharedPtr & msg)
{
  return msg ? rclcpp::Time(msg->header.stamp).nanoseconds() : -1;
}
}  // namespace

ApproximateFusionNode::ApproximateFusionNode(const rclcpp::NodeOptions & options)
: Node("fusion", options)
{
  // Queue size: tracks arrive at 100 /s (two radars through
  // the tracker), camera samples at 33 /s, so about 3 tracks wait per camera
  // period (ideal-timing oracle, t2approx: at most 3 queued). Sized to ride through
  // one lost camera message without the overflow path (which drops the
  // oldest message and restarts the pivot search): ApproximateTime then
  // holds 7 tracks, ExactTime 9 open stamps (one lost camera sample at a
  // 60 ms match stamp). 9 covers both; one shared launch argument. Below 3
  // the approximate queue overflows every camera period.
  // Start-up is the only recorded overflow (5 of the 47
  // stock runs with a sample ledger, 2 to 9 drops each, none in the 42 rcllf
  // runs): one input's first samples came late, so the other input's
  // samples piled up (one Orin NX stock run with a camera fault, camera
  // 112 ms after the first track, peak 10). Replaying the arrivals at queue
  // 1000 (never overflows) gives the same pairs in 4 of the 5. In one
  // stock run with a camera fault it adds the 2 earliest pairs,
  // so there the overflow cost the first two fused values, yet the partial
  // stage acted at plant step 249, and the stop (4.081 m short) is inside
  // the spread of the stock runs.
  // No finite queue covers an unbounded start delay, which is why the sample
  // ledger counts these drops instead of a larger queue hiding them.
  // The queue size is not a freshness bound. In a
  // track stall of 100 ms or more, ApproximateTime holds its candidate pair
  // and publishes it when tracks resume, 122 ms (100 ms stall) to 222 ms
  // (200 ms stall) after its older stamp, at queue 9 and at an unbounded
  // queue alike (only a queue of 3 evicts it, and 3 overflows on one lost
  // camera sample). In stock ROS 2 the stall shows as a /fused/range gap
  // past kFusedApproxDeadline, which is what reports it (test_retune.py).
  // rcllf runs deadline handlers per missing message only, so there the lost
  // tracks report it at this node's input, not the fused gap downstream.
  // Pairing parameters (a review finding), set explicitly so a
  // message_filters default change cannot move them silently:
  //   age penalty 0.1 and no maximum interval. These are the values every
  //   recorded t2 run used. Replaying the recorded arrivals of 50 runs
  //   through the k7 port (scripts/replay_sync.py)
  //   reproduces all their pairs. For rcllf (27 runs, logical stamps on a
  //   10 ms track grid) every pair is at most 10 ms apart and the pairs are
  //   the same for an age penalty 0..5 and any maximum interval of at least
  //   10 ms. For stock (23 runs, node-clock stamps) that is false: pairs reach
  //   12.9 ms apart (start-up bursts, a lost camera sample), and 18 of the 23
  //   runs pair differently under another age penalty or a 10..20 ms maximum
  //   interval, moving a fused range by up to 0.226 m. A maximum interval of
  //   10 ms also leaves 7 camera samples unpaired in those runs. So no
  //   alternative is better and the defaults are kept. The 0.226 m is a
  //   difference between parameter choices, not an error against the plant:
  //   the range over-read that enters the stop margin is kRangeOverRead
  //   (stop_margin.hpp), checked on the recorded runs.
  //   A finite maximum interval would not bound freshness either: the stall
  //   pair above is 10 ms apart.
  //   Inter-message lower bound 0 on both inputs. A bound must not exceed the
  //   smallest stamp gap an input ever sees, or the policy may publish a pair
  //   it has not proven best. The radar input carries both radars, so two
  //   samples share a stamp (rcllf: gap 0) or nearly (stock: 128 ns), and
  //   node-clock camera stamps come as close as 29.085 ms (smallest gaps in
  //   the 50 recorded runs). 0 is the one radar bound
  //   valid everywhere, and a camera bound under 29 ms would prune too
  //   little to matter, so both are 0. A valid bound only moves the moment a
  //   pair is proven best, never which pair is chosen.
  constexpr double kAgePenalty = 0.1;
  const int64_t queue_size = this->declare_parameter<int64_t>("queue_size", 9);
  log_.open(this->declare_parameter<std::string>("log_dir", ""), this->get_name());
  // Fault injection: BF_FUSION_STALL="N,MS" blocks
  // this node's executor for MS ms of wall time inside camera arrival N, a
  // stalled reader whose inputs keep arriving. The run then shows which input
  // ids are taken after it, which pairs come out, what the ledger drops and
  // which deadline and lost callbacks fire. An environment variable like the
  // BF_DROP_* variables, so stock and rcllf runs get it without a rebuild.
  if (const char * stall_env = std::getenv("BF_FUSION_STALL")) {
    char * rest = nullptr;
    stall_at_ = std::strtoll(stall_env, &rest, 10);
    stall_ms_ = (rest && *rest == ',') ? std::strtoll(rest + 1, nullptr, 10) : 0;
    log_.line(
      "\"ev\":\"fault_config\",\"kind\":\"stall\",\"camera_n\":%" PRId64
      ",\"ms\":%" PRId64, stall_at_, stall_ms_);
  }

  // Offered deadline, so the controller's requested one matches.
  // depth 10, DERIVED (rule in loss_events.hpp:82): pairs come
  // every 20 to 40 ms (loss_events.hpp:72), so ten cover at least 200 ms of
  // traffic, more than kFusedApproxDeadline 50 ms and the largest deadline,
  // 90 ms. rcllf burst B = 9 on /fused/range, DERIVED: publish_bounds 1 (the
  // one publish() in sync_callback) x the ApproximateTime queue size Q = 9
  // (queue_size below; t2/t3 launch files pass 9), since one add() signals at
  // most Q pairs (lf_generator.py _sync_calls_per_input). The binding deepens
  // the controller's reader to max(8, B) = 9 itself (_reader_depth; no
  // brake_fusion config declares reader_depth), so this
  // literal is not what holds the burst. Generated: colcon lf-generate puts
  // burst 9, reader_depth 9 on this edge in every t2/t3 approx system's
  // rcllf_trace_manifest.json ("proven x ApproximateTime Q=9"); the k0-k2
  // pilots launch with queue_size 10 and get 10. Measured: at most 1 pair per input
  // callback in the recorded runs.
  auto fused_qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable().durability_volatile().deadline(
    rclcpp::Duration(kFusedApproxDeadline));
  fused_pub_ = this->create_publisher<sensor_msgs::msg::Range>("/fused/range", fused_qos);
  // The same input QoS, deadlines and loss handlers as the
  // ExactTime sibling, so the policies differ only in pairing.
  // The radar input (remapped to /radar/track in t2/t3) stays
  // best effort and the camera input reliable, both a design choice. Depth 10
  // DERIVED by the same rule: ten camera samples cover 300 ms, ten
  // /radar/track samples (one per sample of two 20 ms radars) 100 ms, each
  // more than 90 ms.
  // Measured: no sample of either input lost after startup in the recorded runs.
  // The analyzer resolves this message_filters QoS (best effort, KeepLast 10,
  // volatile in the t2/t3 system.json), and rcllf binds the reader with it.
  // Its RCLLF-H001 warning (best effort input) names the source topic
  // /radar/range, not the remapped /radar/track.
  auto radar_qos = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort().durability_volatile().deadline(
    rclcpp::Duration(kRadarDeadline));
  auto camera_qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable().durability_volatile().deadline(
    rclcpp::Duration(kCameraDeadline));
  rclcpp::SubscriptionOptions radar_options = loss_options(log_, this->get_clock(), "/radar/range");
  rclcpp::SubscriptionOptions camera_options =
    loss_options(log_, this->get_clock(), "/camera/range");
  radar_sub_.subscribe(this, "/radar/range", radar_qos.get_rmw_qos_profile(), radar_options);
  camera_sub_.subscribe(this, "/camera/range", camera_qos.get_rmw_qos_profile(), camera_options);
  radar_sub_.registerCallback(
    std::bind(&ApproximateFusionNode::radar_arrival, this, std::placeholders::_1));
  camera_sub_.registerCallback(
    std::bind(&ApproximateFusionNode::camera_arrival, this, std::placeholders::_1));
  sync_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(
    SyncPolicy(static_cast<uint32_t>(queue_size)), radar_sub_, camera_sub_);
  sync_->registerCallback(
    std::bind(
      &ApproximateFusionNode::sync_callback, this, std::placeholders::_1, std::placeholders::_2));
  sync_->setAgePenalty(kAgePenalty);
  sync_->setInterMessageLowerBound(0, rclcpp::Duration(0, 0));
  sync_->setInterMessageLowerBound(1, rclcpp::Duration(0, 0));
  // The ledger settles at the next arrival and at shutdown (sample_ledger.hpp):
  // rcllf admits one observer per Subscriber, the arrival callbacks above.
  ledger_ = std::make_unique<SampleLedger>(static_cast<std::size_t>(queue_size));
  log_.line(
    "\"ev\":\"config\",\"sync_policy\":\"approximate\",\"queue_size\":%" PRId64
    ",\"age_penalty\":%.3g,\"max_interval\":\"none\",\"inter_message_lower_bound_ns\":0",
    queue_size, kAgePenalty);
}

ApproximateFusionNode::~ApproximateFusionNode()
{
  // Samples the policy still holds when the node stops, then the totals per
  // input (0 = /radar/track, 1 = /camera/range). peak is the most samples of
  // one input the policy held at a push (its queue demand), held_max the most
  // it kept after an add. The last add's discards first.
  settle();
  for (const auto & f : ledger_->pending()) {
    log_.line(
      "\"ev\":\"sample_pending\",\"input\":%d,\"n\":%" PRId64 ",\"stamp_ns\":%" PRId64,
      f.input, f.n, f.stamp_ns);
  }
  for (int i = 0; i < 2; ++i) {
    log_.line(
      "\"ev\":\"ledger\",\"input\":%d,\"arrivals\":%" PRId64 ",\"paired\":%" PRId64
      ",\"superseded\":%" PRId64 ",\"queue_overflow\":%" PRId64 ",\"pending\":%" PRId64
      ",\"duplicates\":%" PRId64 ",\"peak\":%" PRId64 ",\"held_max\":%" PRId64,
      i, ledger_->arrivals(i), ledger_->paired_count(i), ledger_->superseded(i),
      ledger_->overflowed(i), ledger_->held(i), ledger_->duplicates(i), ledger_->peak(i),
      ledger_->held_max(i));
  }
}

void ApproximateFusionNode::settle()
{
  for (const auto & f : ledger_->settle()) {
    log_.line(
      "\"ev\":\"sample_drop\",\"input\":%d,\"n\":%" PRId64 ",\"stamp_ns\":%" PRId64
      ",\"reason\":\"%s\",\"t_ns\":%" PRId64,
      f.input, f.n, f.stamp_ns, f.reason, this->now().nanoseconds());
  }
}

void ApproximateFusionNode::note_duplicate(int input, int64_t n, int64_t stamp, int64_t earlier)
{
  if (earlier >= 0) {
    log_.line(
      "\"ev\":\"duplicate_stamp\",\"input\":%d,\"n\":%" PRId64 ",\"earlier_n\":%" PRId64
      ",\"stamp_ns\":%" PRId64, input, n, earlier, stamp);
  }
}

void ApproximateFusionNode::radar_arrival(const sensor_msgs::msg::Range::ConstSharedPtr & msg)
{
  settle();  // the discards of the previous add (t_ns is this arrival's)
  // Same fields as FirstRadarFusionNode's radar_in, without its own pairing
  // verdict (taken / why): the synchronizer decides after this line.
  log_.line(
    "\"ev\":\"radar_in\",\"n\":%" PRId64 ",\"src\":\"%s\",\"stamp_ns\":%" PRId64
    ",\"range\":%.9g,\"t_ns\":%" PRId64,
    radar_in_, msg->header.frame_id.c_str(), stamp_ns(msg), msg->range,
    this->now().nanoseconds());
  note_duplicate(0, radar_in_, stamp_ns(msg), ledger_->arrive(0, msg, radar_in_, stamp_ns(msg)));
  ++radar_in_;
}

void ApproximateFusionNode::camera_arrival(const sensor_msgs::msg::Range::ConstSharedPtr & msg)
{
  settle();
  log_.line(
    "\"ev\":\"camera_in\",\"n\":%" PRId64 ",\"stamp_ns\":%" PRId64 ",\"range\":%.9g"
    ",\"t_ns\":%" PRId64,
    camera_in_, stamp_ns(msg), msg->range, this->now().nanoseconds());
  if (camera_in_ == stall_at_) {
    const auto mono = [] {
        return std::chrono::duration_cast<std::chrono::nanoseconds>(
          std::chrono::steady_clock::now().time_since_epoch()).count();
      };
    const int64_t from = mono();
    std::this_thread::sleep_for(std::chrono::milliseconds(stall_ms_));
    log_.line(
      "\"ev\":\"fault\",\"kind\":\"stall\",\"camera_n\":%" PRId64 ",\"mono_from_ns\":%"
      PRId64 ",\"mono_to_ns\":%" PRId64 ",\"t_ns\":%" PRId64,
      camera_in_, from, static_cast<int64_t>(mono()), this->now().nanoseconds());
  }
  note_duplicate(1, camera_in_, stamp_ns(msg), ledger_->arrive(1, msg, camera_in_, stamp_ns(msg)));
  ++camera_in_;
}

void ApproximateFusionNode::sync_callback(
  const sensor_msgs::msg::Range::ConstSharedPtr & radar,
  const sensor_msgs::msg::Range::ConstSharedPtr & camera)
{
  // The fused range is the mean of the two ranges, so its stamp is the mean
  // of the two stamps (equal under ExactTime).
  const int64_t radar_ns = stamp_ns(radar);
  const int64_t camera_ns = stamp_ns(camera);
  const int64_t fused_ns = radar_ns + (camera_ns - radar_ns) / 2;
  sensor_msgs::msg::Range fused;
  fused.header.stamp = rclcpp::Time(fused_ns, this->get_clock()->get_clock_type());
  fused.header.frame_id = "fused";
  fused.radiation_type = radar->radiation_type;
  fused.min_range = radar->min_range;
  fused.max_range = radar->max_range;
  fused.range = 0.5F * (radar->range + camera->range);
  const auto members = ledger_->paired(radar.get(), camera.get());
  log_.line(
    "\"ev\":\"pair\",\"n\":%" PRId64 ",\"radar_stamp_ns\":%" PRId64 ",\"camera_stamp_ns\":%" PRId64
    ",\"fused_stamp_ns\":%" PRId64 ",\"radar_range\":%.9g,\"camera_range\":%.9g"
    ",\"fused_range\":%.9g,\"radar_src\":\"%s\",\"radar_n\":%" PRId64 ",\"camera_n\":%" PRId64
    ",\"t_ns\":%" PRId64,
    pairs_, radar_ns, camera_ns, fused_ns, radar->range, camera->range, fused.range,
    radar->header.frame_id.c_str(), members[0], members[1], this->now().nanoseconds());
  ++pairs_;
  fused_pub_->publish(fused);
}

}  // namespace brake_fusion

RCLCPP_COMPONENTS_REGISTER_NODE(brake_fusion::ApproximateFusionNode)
