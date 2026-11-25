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

#include "readback.hpp"

#include <dlfcn.h>
#include <malloc.h>
#include <sys/syscall.h>
#include <unistd.h>

#include <cctype>
#include <cstddef>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <map>
#include <mutex>
#include <set>
#include <sstream>
#include <string>
#include <system_error>
#include <utility>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rid_table.hpp"
#include "ros2_lf_binding/corr_table_api.h"
#include "ros2_lf_binding/readback_api.h"

extern "C" {
#include "core/environment.h"
#include "core/lf_types.h"
}

// Bounds of the patch-marker section of the generated executable, emitted by
// colcon_lf's rcllf_trace_build_info.cpp. Weak: an older program,
// or the unit tests, leave it null and the read-back says so.
extern "C" __attribute__((weak)) const char * const rcllf_reactor_c_patch_markers[2];

namespace ros2_lf_binding
{
namespace
{

/// A symbol of the EXECUTABLE. This library links its own single-threaded
/// reactor-c, whose copy of a symbol must not be read as the program's.
void * executable_symbol(const char * name)
{
  void * sym = dlsym(RTLD_DEFAULT, name);
  if (sym == nullptr) {
    return nullptr;
  }
  Dl_info self{};
  Dl_info found{};
  if (dladdr(reinterpret_cast<void *>(&executable_symbol), &self) != 0 &&
    dladdr(sym, &found) != 0 && self.dli_fbase == found.dli_fbase)
  {
    return nullptr;
  }
  return sym;
}

nlohmann::json rid_entries_json()
{
  nlohmann::json out = nlohmann::json::array();
  for (const auto & e : get_rid_entries()) {
    out.push_back(
        {
          {"reaction_id", e.reaction_id},
          {"node_class", e.node_class},
          {"callback_name", e.callback_name},
          {"kind", e.kind},
          {"topic", e.topic},
          {"period_ns", e.period_ns},
    });
  }
  return out;
}

/// Timers reactor-c fires, from the first environment. Only the fields
/// before any configuration #if of environment_t and trigger_t are read.
nlohmann::json lf_timers_json()
{
  using EnvsFn = int (*)(environment_t **);
  auto fn = reinterpret_cast<EnvsFn>(executable_symbol("_lf_get_environments"));
  nlohmann::json out = {{"available", false}, {"timers", nlohmann::json::array()}};
  if (fn == nullptr) {
    return out;
  }
  environment_t * envs = nullptr;
  const int n = fn(&envs);
  if (n <= 0 || envs == nullptr) {
    return out;
  }
  out["available"] = true;
  out["environments"] = n;
  out["worker_thread_count"] = envs[0].worker_thread_count;
  for (int i = 0; i < envs[0].timer_triggers_size; ++i) {
    const trigger_t * t = envs[0].timer_triggers[i];
    if (t == nullptr) {
      continue;
    }
    out["timers"].push_back(
        {
          {"offset_ns", t->offset},
          {"period_ns", t->period},
          {"reactions", t->number_of_reactions},
    });
  }
  return out;
}

/// Every admitted correspondence edge as the runtime schedules it.
/// Its delay is the edge's after-delay: lf_schedule(corr_drain, w) uses it,
/// no LF `after` does.
nlohmann::json corr_edges_json()
{
  nlohmann::json out = nlohmann::json::array();
  for (const auto & e : registered_corr_edges()) {
    if (e.pub_key.empty()) {
      continue;
    }
    const char * role = e.role == CorrEdgeRole::kEgress ? "egress" :
      e.role == CorrEdgeRole::kIngress ? "ingress" : "local";
    out.push_back(
        {
          {"edge", e.edge_id},
          {"pub_key", e.pub_key},
          {"pub_topic", e.pub_topic},
          {"sub_key", e.sub_key},
          {"sub_topic", e.sub_topic},
          {"delay_ns", e.delay_ns},
          {"role", role},
          {"deferred", e.deferred},
    });
  }
  return out;
}

/// "Main.child.grandchild" from the base.name chain (full_name is set lazily).
std::string reactor_path(const self_base_t * self)
{
  std::vector<std::string> names;
  for (int depth = 0; self != nullptr && depth < 64; ++depth, self = self->parent) {
    names.emplace_back(self->name != nullptr ? self->name : "?");
  }
  std::string out;
  for (auto it = names.rbegin(); it != names.rend(); ++it) {
    out += (out.empty() ? "" : ".") + *it;
  }
  return out;
}

struct LinkRegistry
{
  std::mutex mutex;
  std::vector<LfLinkSpec> links;
  /// lfc class -> offset in its self struct -> input port.
  std::map<std::string, std::map<std::size_t, std::string>> ports;
  /// lfc class -> (reactions per instance, instances) the generated program declares.
  std::map<std::string, std::pair<int, int>> classes;
};

LinkRegistry & link_registry()
{
  static LinkRegistry registry;
  return registry;
}

/// lfc's C class of a reaction, from the exported symbol of its function:
/// `_Z<len><class>reaction_function_<number>Pv` (lfc compiles reaction
/// `number` of class `<class>` as `<class>reaction_function_<number>`).
/// Empty unless the symbol starts exactly at the function and names the
/// reaction's own number.
std::string reaction_class(const reaction_t * r)
{
  Dl_info info{};
  void * fn = reinterpret_cast<void *>(r->function);
  if (fn == nullptr || dladdr(fn, &info) == 0 || info.dli_sname == nullptr ||
    info.dli_saddr != fn)
  {
    return "";
  }
  const std::string sym = info.dli_sname;
  std::size_t i = 2;
  std::size_t len = 0;
  if (sym.rfind("_Z", 0) != 0) {
    return "";
  }
  for (; i < sym.size() && std::isdigit(static_cast<unsigned char>(sym[i])); ++i) {
    len = len * 10 + static_cast<std::size_t>(sym[i] - '0');
  }
  const std::string tail = "reaction_function_" + std::to_string(r->number);
  if (i + len + 2 != sym.size() || sym.compare(i + len, 2, "Pv") != 0 || len <= tail.size() ||
    sym.compare(i + len - tail.size(), tail.size(), tail) != 0)
  {
    return "";
  }
  return sym.substr(i, len - tail.size());
}

/// True when the executable exports reaction `number` of `cls`.
bool exports_reaction(const std::string & cls, int number)
{
  const std::string id = cls + "reaction_function_" + std::to_string(number);
  const std::string sym = "_Z" + std::to_string(id.size()) + id + "Pv";
  return executable_symbol(sym.c_str()) != nullptr;
}

/// Every reaction of the executable's reactor-c, its deadlines
/// and lfc's delay reactors with the link they realise. Not a walk from the
/// startup and timer reactions (it misses action- and input-only reactions):
/// lfc allocates every reactor's self struct with lf_new_reactor, which
/// records it in `_lf_reactors_to_free`, and declares every reaction as a
/// `reaction_t _lf__reaction_<n>` field of that struct. So each allocation is
/// scanned for reaction_t whose `self` is the allocation and whose `function`
/// is the exported `<class>reaction_function_<number>`. A class is complete
/// when an instance holds reactions 0..n-1 and the executable exports no
/// reaction n; anything else is listed under `incomplete`.
nlohmann::json lf_reaction_graph_json()
{
  nlohmann::json out = {
    {"available", false},
    {"delayed_connections", nlohmann::json::array()},
    {"deadlines", nlohmann::json::array()},
  };
  using EnvsFn = int (*)(environment_t **);
  auto fn = reinterpret_cast<EnvsFn>(executable_symbol("_lf_get_environments"));
  auto * to_free = static_cast<allocation_record_t **>(executable_symbol("_lf_reactors_to_free"));
  if (fn == nullptr || to_free == nullptr) {
    return out;
  }
  environment_t * envs = nullptr;
  const int n_envs = fn(&envs);

  struct Reactor
  {
    const char * base;
    std::size_t size;
    std::string cls;
    std::map<int, const reaction_t *> reactions;
  };
  std::map<const void *, Reactor> reactors;
  std::vector<const void *> order;
  for (const allocation_record_t * a = *to_free; a != nullptr; a = a->next) {
    const auto * base = static_cast<const char *>(a->allocated);
    Reactor & rc = reactors[a->allocated];
    rc.base = base;
    rc.size = base != nullptr ? malloc_usable_size(a->allocated) : 0;
    order.push_back(a->allocated);
    for (std::size_t off = 0; off + sizeof(reaction_t) <= rc.size; off += alignof(reaction_t)) {
      void * self = nullptr;
      std::memcpy(&self, base + off + offsetof(reaction_t, self), sizeof(self));
      if (self != a->allocated) {
        continue;
      }
      const auto * r = reinterpret_cast<const reaction_t *>(base + off);
      const std::string cls = r->number >= 0 ? reaction_class(r) : "";
      if (cls.empty() || (!rc.cls.empty() && cls != rc.cls)) {
        continue;
      }
      rc.cls = cls;
      rc.reactions[r->number] = r;
    }
  }

  std::map<std::string, int> class_reactions;
  nlohmann::json incomplete = nlohmann::json::array();
  nlohmann::json without = nlohmann::json::array();
  std::size_t found = 0;
  for (const void * self : order) {
    const Reactor & rc = reactors[self];
    const std::string path = reactor_path(static_cast<const self_base_t *>(self));
    found += rc.reactions.size();
    if (rc.reactions.empty()) {
      without.push_back(path);
      continue;
    }
    const int n = static_cast<int>(rc.reactions.size());
    if (rc.reactions.rbegin()->first != n - 1 || exports_reaction(rc.cls, n)) {
      incomplete.push_back(
          path + " (" + rc.cls + "): found " + std::to_string(n) +
          " reactions up to number " + std::to_string(rc.reactions.rbegin()->first) +
        (exports_reaction(rc.cls, n) ? ", the executable has reaction " + std::to_string(n) : ""));
    }
    auto known = class_reactions.emplace(rc.cls, n);
    if (!known.second && known.first->second != n) {
      incomplete.push_back(
          path + " (" + rc.cls + "): " + std::to_string(n) + " reactions, another instance " +
          std::to_string(known.first->second));
    }
  }

  // Who writes what: reactor-c flattens port hierarchy, so an output trigger
  // is the final input port trigger, held in the self struct of its reactions.
  std::map<const reaction_t *, std::set<const reaction_t *>> upstream;
  std::map<const reaction_t *, const Reactor *> owner;
  for (const void * self : order) {
    for (const auto & kv : reactors[self].reactions) {
      owner[kv.second] = &reactors[self];
    }
  }
  auto for_each_output = [](const reaction_t * r, auto && visit) {
      for (std::size_t i = 0; r->triggers != nullptr && i < r->num_outputs; ++i) {
        const int width = r->triggered_sizes != nullptr ? r->triggered_sizes[i] : 0;
        for (int j = 0; r->triggers[i] != nullptr && j < width; ++j) {
          if (r->triggers[i][j] != nullptr) {
            visit(r->triggers[i][j]);
          }
        }
      }
    };
  for (const auto & kv : owner) {
    for_each_output(
      kv.first, [&](const trigger_t * t) {
        for (int k = 0; t->reactions != nullptr && k < t->number_of_reactions; ++k) {
          upstream[t->reactions[k]].insert(kv.first);
        }
      });
  }
  LinkRegistry & reg = link_registry();
  std::lock_guard<std::mutex> lock(reg.mutex);

  // A reactor whose reaction symbols do not resolve has no class and no
  // reactions here; the declared counts make it visible.
  std::map<std::string, int> class_instances;
  for (const void * self : order) {
    if (!reactors[self].cls.empty()) {
      ++class_instances[reactors[self].cls];
    }
  }
  if (reg.classes.empty()) {
    incomplete.push_back(
      "the program declares no reactor classes (generated by an older colcon lf-generate)");
  }
  nlohmann::json declared = nlohmann::json::object();
  for (const auto & kv : reg.classes) {
    const int want_reactions = kv.second.first;
    const int want_instances = kv.second.second;
    declared[kv.first] = {{"reactions", want_reactions}, {"instances", want_instances}};
    const int got_instances = class_instances[kv.first];
    const auto got = class_reactions.find(kv.first);
    const int got_reactions = got != class_reactions.end() ? got->second : 0;
    if (got_instances != want_instances || got_reactions != want_reactions) {
      incomplete.push_back(
        kv.first + ": the program declares " + std::to_string(want_instances) +
        " instance(s) of " + std::to_string(want_reactions) + " reactions, the scan found " +
        std::to_string(got_instances) + " of " + std::to_string(got_reactions));
    }
  }

  // The input port a trigger is: its offset in the self struct of the
  // reactions it triggers, named by the generator's offsetof table.
  auto port_of = [&](const trigger_t * t) -> nlohmann::json {
      if (t->reactions == nullptr || t->number_of_reactions < 1) {
        return {{"reactor", nullptr}, {"port", nullptr}};
      }
      const void * self = t->reactions[0]->self;
      const auto it = reactors.find(self);
      const std::string path = reactor_path(static_cast<const self_base_t *>(self));
      const auto * p = reinterpret_cast<const char *>(t);
      if (it == reactors.end() || p < it->second.base || p >= it->second.base + it->second.size) {
        return {{"reactor", path}, {"port", nullptr}};
      }
      const auto cls = reg.ports.find(it->second.cls);
      const std::size_t off = static_cast<std::size_t>(p - it->second.base);
      if (cls == reg.ports.end() || cls->second.count(off) == 0) {
        return {{"reactor", path}, {"port", nullptr}};
      }
      return {{"reactor", path}, {"port", cls->second.at(off)}};
    };

  for (const void * self : order) {
    const Reactor & rc = reactors[self];
    for (const auto & kv : rc.reactions) {
      const reaction_t * r = kv.second;
      if (r->deadline >= 0) {
        // reactor-c treats a negative deadline (NEVER) as none (reactor_common.c).
        out["deadlines"].push_back(
            {
              {"reactor", reactor_path(static_cast<const self_base_t *>(self))},
              {"class", rc.cls},
              {"reaction", r->number},
              {"deadline_ns", r->deadline},
        });
      }
    }
    // lfc 0.12's generic delay reactor of a connection `after d` is class
    // `_lf_GenDelay_<hash>`, C name `__lf_gendelay_<hash>`: reaction 0 is
    // `reaction(act) -> out`, reaction 1 `reaction(inp) -> act`, and its self
    // struct declares `trigger_t _lf__act` right after `_lf__reaction_1`
    // (lfc's __lf_gendelay_<hash>.h). User classes are `_<lowercase name>`,
    // and lf-generate refuses a reactor named `_lf...`, so the prefix names
    // only lfc's class. The trigger is read only inside the allocation and
    // only if it points back at reaction 0.
    if (rc.cls.rfind("__lf_gendelay_", 0) != 0 || rc.reactions.size() != 2) {
      continue;
    }
    const reaction_t * r0 = rc.reactions.at(0);
    const reaction_t * r1 = rc.reactions.at(1);
    const auto * act_at = reinterpret_cast<const char *>(r1 + 1);
    if (act_at + offsetof(trigger_t, is_timer) + sizeof(bool) > rc.base + rc.size) {
      continue;
    }
    const auto * act = reinterpret_cast<const trigger_t *>(act_at);
    if (act->is_timer || act->number_of_reactions != 1 || act->reactions == nullptr ||
      act->reactions[0] != r0)
    {
      continue;
    }
    std::set<std::string> sources;
    for (const reaction_t * u : upstream[r1]) {
      sources.insert(reactor_path(static_cast<const self_base_t *>(u->self)));
    }
    nlohmann::json destinations = nlohmann::json::array();
    for_each_output(r0, [&](const trigger_t * t) {destinations.push_back(port_of(t));});
    out["delayed_connections"].push_back(
        {
          {"delay_reactor", reactor_path(static_cast<const self_base_t *>(self))},
          {"class", rc.cls},
          {"after_ns", act->offset},
          {"sources", sources},
          {"destinations", destinations},
    });
  }

  // lfc 0.12 gives a federate one network input action per
  // inbound cross-federate connection, `_lf_action_table[i]`, and that
  // connection's `after` at the same index of `_lf_action_delay_table`
  // (initialize_triggers_for_federate); reactor-c's federate.c reads both at
  // runtime. The action triggers the NetworkReceiver's reaction, whose output
  // reaches the input port the link lands on. A program that is not a
  // federate defines neither table.
  nlohmann::json network = nlohmann::json::array();
  auto * actions = static_cast<lf_action_base_t **>(executable_symbol("_lf_action_table"));
  auto * delays = static_cast<const interval_t *>(executable_symbol("_lf_action_delay_table"));
  auto * n_actions = static_cast<const std::size_t *>(executable_symbol("_lf_action_table_size"));
  out["network_table"] = actions != nullptr && delays != nullptr && n_actions != nullptr;
  for (std::size_t i = 0; out["network_table"].get<bool>() && i < *n_actions; ++i) {
    nlohmann::json destinations = nlohmann::json::array();
    auto add = [&](const trigger_t * d) {destinations.push_back(port_of(d));};
    const trigger_t * t = actions[i] != nullptr ? actions[i]->trigger : nullptr;
    for (int k = 0; t != nullptr && t->reactions != nullptr && k < t->number_of_reactions; ++k) {
      for_each_output(t->reactions[k], add);
    }
    network.push_back({{"index", i}, {"after_ns", delays[i]}, {"destinations", destinations}});
  }
  out["network_inputs"] = network;

  nlohmann::json links = nlohmann::json::array();
  for (const auto & l : reg.links) {
    links.push_back(
        {
          {"kind", l.kind}, {"src", l.src}, {"dst", l.dst}, {"name", l.name},
          {"carrier", l.carrier}, {"src_instance", l.src_instance},
          {"dst_instance", l.dst_instance}, {"dst_port", l.dst_port}, {"edge", l.edge},
    });
  }
  out["available"] = true;
  out["environments"] = n_envs;
  out["reactors"] = order.size();
  out["reactions_found"] = found;
  out["classes"] = class_reactions;
  out["classes_declared"] = declared;
  out["incomplete"] = incomplete;
  out["reactors_without_reaction"] = without;
  out["links"] = links;
  return out;
}

nlohmann::json lf_workers_json()
{
  // reactor-c's resolved worker count (reactor_common.c), set before the
  // workers start from the `workers` target property or `--workers`.
  auto * workers = static_cast<const unsigned int *>(executable_symbol("_lf_number_of_workers"));
  if (workers == nullptr) {
    return nullptr;
  }
  return *workers;
}

nlohmann::json patches_json()
{
  if (&rcllf_reactor_c_patch_markers == nullptr) {
    return nullptr;
  }
  const char * begin = rcllf_reactor_c_patch_markers[0];
  const char * end = rcllf_reactor_c_patch_markers[1];
  if (begin == nullptr || end == nullptr) {
    // The build info exists but no patched source was compiled in.
    return nlohmann::json::array();
  }
  return split_patch_markers(begin, end);
}

nlohmann::json threads_json()
{
  nlohmann::json out = nlohmann::json::array();
  const long self_tid = static_cast<long>(syscall(SYS_gettid));  // NOLINT(runtime/int)
  std::error_code ec;
  for (const auto & dent : std::filesystem::directory_iterator("/proc/self/task", ec)) {
    std::ifstream f(dent.path() / "stat");
    std::string line;
    if (!std::getline(f, line)) {
      continue;
    }
    nlohmann::json t = thread_sched_from_stat(line);
    if (t.is_null()) {
      continue;
    }
    const long tid = std::strtol(dent.path().filename().c_str(), nullptr, 10);  // NOLINT
    t["tid"] = tid;
    // The first startup reaction runs on an LF worker: this thread.
    t["lf_worker"] = tid == self_tid;
    out.push_back(t);
  }
  return out;
}

}  // namespace

void register_lf_link(const LfLinkSpec & link)
{
  LinkRegistry & reg = link_registry();
  std::lock_guard<std::mutex> lock(reg.mutex);
  reg.links.push_back(link);
}

void register_lf_class(const std::string & reactor_class, int reactions, int instances)
{
  LinkRegistry & reg = link_registry();
  std::lock_guard<std::mutex> lock(reg.mutex);
  reg.classes[reactor_class] = {reactions, instances};
}

void register_lf_port(
  const std::string & reactor_class, std::size_t offset,
  const std::string & port)
{
  LinkRegistry & reg = link_registry();
  std::lock_guard<std::mutex> lock(reg.mutex);
  reg.ports[reactor_class][offset] = port;
}

std::vector<std::string> split_patch_markers(const char * begin, const char * end)
{
  std::vector<std::string> out;
  const char * p = begin;
  while (p != nullptr && p < end) {
    const char * nul = static_cast<const char *>(std::memchr(p, '\0', end - p));
    const char * stop = nul != nullptr ? nul : end;
    if (stop > p) {
      out.emplace_back(p, stop);
    }
    p = stop + 1;
  }
  return out;
}

nlohmann::json thread_sched_from_stat(const std::string & stat)
{
  // "<pid> (<comm>) <state> ..." where comm may hold spaces and ')'.
  const auto open = stat.find('(');
  const auto close = stat.rfind(')');
  if (open == std::string::npos || close == std::string::npos || close < open) {
    return nullptr;
  }
  std::istringstream rest(stat.substr(close + 1));
  std::vector<std::string> tail;
  for (std::string field; rest >> field; ) {
    tail.push_back(field);
  }
  // proc(5): field 40 is rt_priority, field 41 policy; tail[0] is field 3.
  if (tail.size() < 39) {
    return nullptr;
  }
  return {
    {"comm", stat.substr(open + 1, close - open - 1)},
    {"rt_priority", std::strtol(tail[37].c_str(), nullptr, 10)},
    {"policy", std::strtol(tail[38].c_str(), nullptr, 10)},
  };
}

nlohmann::json build_readback()
{
  char exe[4096] = {0};
  const ssize_t n = readlink("/proc/self/exe", exe, sizeof(exe) - 1);
  return {
    {"schema", kReadbackSchema},
    {"pid", static_cast<int64_t>(getpid())},
    {"executable", n > 0 ? std::string(exe, static_cast<size_t>(n)) : std::string()},
    {"rid_entries", rid_entries_json()},
    {"lf_timers", lf_timers_json()},
    {"lf_workers", lf_workers_json()},
    {"corr_edges", corr_edges_json()},
    {"lf_reaction_graph", lf_reaction_graph_json()},
    {"reactor_c_patches", patches_json()},
    {"threads", threads_json()},
  };
}

std::string write_readback(const std::string & dir)
{
  namespace fs = std::filesystem;
  const fs::path target = fs::path(dir) / ("rcllf_readback." + std::to_string(getpid()) + ".json");
  const fs::path tmp = fs::path(target.string() + ".tmp");
  {
    std::ofstream f(tmp);
    if (!f) {
      return "";
    }
    f << build_readback().dump(2) << "\n";
    if (!f) {
      return "";
    }
  }
  std::error_code ec;
  fs::rename(tmp, target, ec);
  return ec ? "" : target.string();
}

void write_readback_once()
{
  // call_once, not a flag: concurrent callers block until the file exists.
  static std::once_flag once;
  std::call_once(
    once, [] {
      const char * dir = std::getenv(kReadbackDirEnv);
      if (dir == nullptr || *dir == '\0') {
        return;
      }
      const std::string path = write_readback(dir);
      if (path.empty()) {
        // The benchmark harness marks a run without this file INVALID, so failing here is loud.
        RCLCPP_ERROR(
          rclcpp::get_logger("ros2_lf_binding"),
          "could not write the read-back file into %s=%s", kReadbackDirEnv, dir);
      } else {
        RCLCPP_INFO(
          rclcpp::get_logger("ros2_lf_binding"), "read-back written to %s", path.c_str());
      }
    });
}

}  // namespace ros2_lf_binding
