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

#include "handle_matching.hpp"

#include <algorithm>
#include <cstdio>
#include <cstdlib>
#include <limits>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"

namespace ros2_lf_binding
{

namespace
{

std::string describe_entry(const RidEntry & e)
{
  return "RID " + std::to_string(e.reaction_id) + " (" + e.node_class + "::" + e.callback_name +
         ", " + e.kind + (e.kind == "sub" ? ", topic '" + e.topic + "'" :
         ", period " + std::to_string(e.period_ns) + " ns") + ")";
}

std::string describe_group(int group)
{
  if (group == kGroupDefault) {
    return "the default callback group";
  }
  if (group == kGroupForeign) {
    return "a callback group another executor spins";
  }
  if (group == kGroupTf2Listener) {
    return "the tf2_ros::TransformListener callback group LF owns";
  }
  return "callback group #" + std::to_string(group);
}

}  // namespace

bool timer_fits(int64_t period_ns, const RidEntry & e)
{
  return e.period_ns > 0 && period_ns == e.period_ns;
}

HandleMatch match_handles(
  const std::string & node_class, const std::string & family,
  const std::vector<const RidEntry *> & entries,
  const std::vector<HandleCandidate> & handles,
  const std::function<bool(const RidEntry &, std::size_t)> & fits,
  const MatchOptions & options)
{
  HandleMatch result;

  // Analyzer creation order; entries without an index go last, in RID order.
  std::vector<std::size_t> order(entries.size());
  for (std::size_t i = 0; i < order.size(); ++i) {
    order[i] = i;
  }
  auto rank = [&entries](std::size_t i) {
      const int idx = entries[i]->creation_index;
      return idx < 0 ? std::numeric_limits<int>::max() : idx;
    };
  std::stable_sort(
    order.begin(), order.end(), [&](std::size_t a, std::size_t b) {
      if (rank(a) != rank(b)) {
        return rank(a) < rank(b);
      }
      return entries[a]->reaction_id < entries[b]->reaction_id;
    });

  // A handle another executor spins is never bound to a RID, not even by an
  // entry whose group is unresolved: both executors would run its callback. It
  // stays unbound and is exempted under the acknowledgement or refused below.
  // A kGroupTf2Listener entry fits only a handle of the detached
  // TransformListener group, and such a handle only that entry.
  auto fits_own = [&](const RidEntry & e, std::size_t h) {
      if (e.group_index == kGroupTf2Listener || handles[h].tf2_listener) {
        return e.group_index == kGroupTf2Listener && handles[h].tf2_listener && fits(e, h);
      }
      return handles[h].group != kGroupForeign && fits(e, h);
    };
  auto fits_in_group = [&](const RidEntry & e, std::size_t h) {
      return (e.group_index == kGroupUnknown || e.group_index == kGroupTf2Listener ||
             e.group_index == handles[h].group) && fits_own(e, h);
    };

  // An entry without a known group may only pick among handles of one group:
  // discovery order across groups is group order, not creation order.
  for (const std::size_t i : order) {
    if (entries[i]->group_index != kGroupUnknown) {
      continue;
    }
    std::vector<std::size_t> fitting;
    for (std::size_t h = 0; h < handles.size(); ++h) {
      if (fits_own(*entries[i], h)) {
        fitting.push_back(h);
      }
    }
    for (const std::size_t h : fitting) {
      if (handles[h].group != handles[fitting.front()].group) {
        std::string list;
        for (const std::size_t f : fitting) {
          list += (list.empty() ? "" : ", ") + std::string("#") + std::to_string(f) + " in " +
            describe_group(handles[f].group);
        }
        result.ok = false;
        result.error = describe_entry(*entries[i]) + " fits " + std::to_string(fitting.size()) +
          " " + family + " handles of node '" + node_class + "' in different callback groups (" +
          list + ") and the analyzer could not resolve its callback group. Creation order " +
          "across callback groups is not visible at bind time, so binding it would be a guess. " +
          "Create the callback group in the constructor, store it in a member and pass that " +
          "member when creating the " + family;
        return result;
      }
    }
  }

  constexpr std::size_t kNone = std::numeric_limits<std::size_t>::max();
  result.handle_of.assign(entries.size(), kNone);
  std::vector<bool> used(handles.size(), false);
  for (const std::size_t i : order) {
    for (std::size_t h = 0; h < handles.size(); ++h) {
      if (!used[h] && fits_in_group(*entries[i], h)) {
        used[h] = true;
        result.handle_of[i] = h;
        break;
      }
    }
  }

  const std::string twice = check_bound_once(node_class, family, entries, handles,
      result.handle_of);
  if (!twice.empty()) {
    result.ok = false;
    result.error = twice;
    return result;
  }
  for (const std::size_t i : order) {
    if (result.handle_of[i] == kNone && options.defer_missing) {
      result.deferred.push_back(i);
    } else if (result.handle_of[i] == kNone) {
      result.ok = false;
      result.error = describe_entry(*entries[i]) + " has no unbound " + family +
        " handle in node '" + node_class + "' that fits it" +
        (entries[i]->group_index == kGroupUnknown ? std::string() :
        " in " + describe_group(entries[i]->group_index)) + " (" +
        std::to_string(handles.size()) + " " + family + " handle(s) found, " +
        std::to_string(entries.size()) + " expected by the analyzer)";
      return result;
    }
  }
  std::vector<bool> bound(handles.size(), false);
  for (const std::size_t h : result.handle_of) {
    if (h != kNone) {
      bound[h] = true;
    }
  }
  for (std::size_t h = 0; h < handles.size(); ++h) {
    if (!bound[h] && options.allow_unclaimed) {
      continue;
    }
    if (!bound[h] && handles[h].tf2_listener) {
      result.ok = false;
      result.error = family + " handle #" + std::to_string(h) + " (" + handles[h].description +
        ") of node '" + node_class + "' sits in " + describe_group(kGroupTf2Listener) +
        " but no reaction ID fits it. tf2_ros no longer runs that group, so the transforms " +
        "it carries would never reach the Buffer";
      return result;
    }
    if (!bound[h] && handles[h].group == kGroupForeign &&
      !handles[h].acknowledged_hazard.empty())
    {
      result.exempted.push_back(h);
    } else if (!bound[h] && handles[h].group == kGroupForeign) {
      result.ok = false;
      result.error = family + " handle #" + std::to_string(h) + " (" + handles[h].description +
        ") of node '" + node_class + "' has no reaction ID and sits in " +
        describe_group(kGroupForeign) + " (a library thread such as " +
        "tf2_ros::TransformListener), outside the LF schedule. Acknowledge RCLLF-R002 in " +
        "execution.yaml to run it as unsupported, or create it without its own executor";
      return result;
    } else if (!bound[h]) {
      result.ok = false;
      result.error = family + " handle #" + std::to_string(h) + " (" + handles[h].description +
        ", " + describe_group(handles[h].group) + ") of node '" + node_class +
        "' has no reaction ID. The analyzer did not " +
        "extract it (a callback shared by two " + family + "s, or a " +
        family + " created outside the constructor), so its callback would never run";
      return result;
    }
  }
  return result;
}

std::string check_bound_once(
  const std::string & node_class, const std::string & family,
  const std::vector<const RidEntry *> & entries,
  const std::vector<HandleCandidate> & handles,
  const std::vector<std::size_t> & handle_of)
{
  for (std::size_t a = 0; a < handle_of.size(); ++a) {
    for (std::size_t b = a + 1; b < handle_of.size(); ++b) {
      if (handle_of[a] == handle_of[b] && handle_of[a] < handles.size()) {
        return family + " handle #" + std::to_string(handle_of[a]) + " (" +
               handles[handle_of[a]].description + ") of node '" + node_class +
               "' bound twice: to " + describe_entry(*entries[a]) + " and to " +
               describe_entry(*entries[b]);
      }
    }
  }
  return "";
}

void fail_bind(const std::string & message)
{
  RCLCPP_FATAL(
    rclcpp::get_logger("ros2_lf_binding"),
    "ros2_lf_bind_node: %s. Refusing to start.", message.c_str());
  std::fflush(stdout);
  std::fflush(stderr);
  // _Exit, not exit: this runs on an LF worker thread while other threads are
  // live, and static destructors racing them would mask the message.
  std::_Exit(EXIT_FAILURE);
}

}  // namespace ros2_lf_binding
