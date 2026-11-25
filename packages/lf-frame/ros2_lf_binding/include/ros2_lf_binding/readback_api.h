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

/// @file readback_api.h
/// @brief Link identities the read-back names delays with.
///
/// Included by generated ros2_lf_reaction_ids_init.cpp. The generator
/// registers which carrier realises each link of the system: an LF connection
/// into an input port, or a correspondence edge. It also registers the offset of
/// every reaction-triggering input port in lfc's self struct (`offsetof` on
/// lfc's own header, compiled into lfc's target), so that the read-back can
/// name the port a delay reactor writes. The delay itself is read from the
/// running program, never from these registrations.

#ifndef ROS2_LF_BINDING__READBACK_API_H_
#define ROS2_LF_BINDING__READBACK_API_H_

#include <cstddef>
#include <string>
#include <vector>

namespace ros2_lf_binding
{

  struct LfLinkSpec
  {
    std::string kind;          ///< "topic", "service request", "action goal_req", ...
    std::string src;           ///< Node key of the sending end (resolved model `src`).
    std::string dst;           ///< Node key of the receiving end.
    std::string name;          ///< Topic, service or action name (resolved model `name`).
    std::string carrier;       ///< "lf": an LF connection; "corr": a correspondence edge.
    std::string src_instance;  ///< "lf": LF instance of the sending node.
    std::string dst_instance;  ///< "lf": LF instance of the receiving node.
    std::string dst_port;      ///< "lf": input port of the receiving node.
    int edge = -1;             ///< "corr": the edge id.
  };

  void register_lf_link(const LfLinkSpec & link);

  /// The generated program has `instances` instances of `reactor_class` (lfc's
  /// C name), each with `reactions` reactions. The read-back's reaction scan
  /// must find exactly these, or the run is INVALID.
  void register_lf_class(const std::string & reactor_class, int reactions, int instances);

  /// `reactor_class` is lfc's C name of the class (`_<lowercase class>`).
  void register_lf_port(
    const std::string & reactor_class, std::size_t offset,
    const std::string & port);

}  // namespace ros2_lf_binding

#endif  // ROS2_LF_BINDING__READBACK_API_H_
