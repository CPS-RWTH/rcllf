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

#include "plant_replay.hpp"

#include <algorithm>
#include <cstdint>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <mutex>
#include <string>
#include <utility>
#include <vector>

#include <nlohmann/json.hpp>

#include "plant_record.hpp"

namespace ros2_lf_binding
{
namespace plant_replay
{
namespace
{

std::mutex g_mu;
bool g_read = false;

std::string & dir_locked()  // guarded by g_mu
{
  static std::string d;
  return d;
}

void load_locked()
{
  if (g_read) {
    return;
  }
  const char * v = std::getenv("RCLLF_PLANT_REPLAY");
  dir_locked() = v != nullptr ? v : "";
  g_read = true;
}

std::string at(const std::string & path, size_t line, const std::string & what)
{
  return path + ":" + std::to_string(line) + ": " + what;
}

bool is_int(const nlohmann::json & j, const char * key)
{
  return j.contains(key) && j[key].is_number_integer();
}

bool is_str(const nlohmann::json & j, const char * key)
{
  return j.contains(key) && j[key].is_string();
}

bool is_int_or_null(const nlohmann::json & j, const char * key)
{
  return j.contains(key) && (j[key].is_number_integer() || j[key].is_null());
}

/// The fields
/// plant_record::external() writes on every external entry, and their types.
/// A missing or wrong-typed one refuses the recording; it never reads as 0.
std::string external_fields_why(const nlohmann::json & j)
{
  for (const char * k : {"stamp_ns", "arrival_ns", "serialize_ns", "cdr_len"}) {
    if (!is_int(j, k)) {
      return std::string(k) + " is missing or not an integer";
    }
  }
  for (const char * k : {"insert_ns", "tag_ns"}) {
    if (!is_int_or_null(j, k)) {
      return std::string(k) + " is missing or not an integer or null";
    }
  }
  if (!is_str(j, "cdr")) {
    return "cdr is missing or not a string";
  }
  if (j.contains("replay_of") && !j["replay_of"].is_number_integer()) {
    return "replay_of is not an integer";  // optional: written by a replay only
  }
  return "";
}

/// The dispositions external_input.cpp records (disposition_name); any other
/// word is a malformed recording, never a silently skipped
/// record.
bool known_disposition(const std::string & d)
{
  return d == "admitted" || d == "late" || d == "overrun" || d == "collision" ||
         d == "after_stop" || d == "refused" || d == "tf_mixed_stamps";
}

bool header_ok(
  const nlohmann::json & j, const char * kind, const std::string & path, std::string * why)
{
  if (!is_str(j, "format") || j["format"].get<std::string>() != "rcllf_plant_record" ||
    !is_int(j, "version") || j["version"].get<int64_t>() != plant_record::kFormatVersion ||
    !is_str(j, "kind") || j["kind"].get<std::string>() != kind || !is_int(j, "lf_time_start"))
  {
    *why = at(
      path, 1, std::string("not an rcllf_plant_record version ") +
      std::to_string(plant_record::kFormatVersion) + " '" + kind + "' header");
    return false;
  }
  return true;
}

}  // namespace

bool enabled()
{
  std::lock_guard<std::mutex> lk(g_mu);
  load_locked();
  return !dir_locked().empty();
}

std::string dir()
{
  std::lock_guard<std::mutex> lk(g_mu);
  load_locked();
  return dir_locked();
}

bool base64_decode(const std::string & in, std::vector<uint8_t> * out)
{
  static const std::string abc =
    "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
  out->clear();
  if (in.size() % 4 != 0) {
    return false;
  }
  size_t pad = 0;
  while (pad < in.size() && pad < 2 && in[in.size() - 1 - pad] == '=') {
    ++pad;
  }
  uint32_t v = 0;
  int bits = 0;
  for (size_t i = 0; i + pad < in.size(); ++i) {
    const size_t p = abc.find(in[i]);
    if (p == std::string::npos) {
      return false;  // includes a '=' anywhere but the last two places
    }
    v = (v << 6) | static_cast<uint32_t>(p);
    bits += 6;
    if (bits >= 8) {
      bits -= 8;
      out->push_back(static_cast<uint8_t>((v >> bits) & 0xffu));
    }
  }
  return true;
}

static bool load_topic_impl(
  const std::string & path, TopicFile * out, std::string * why, size_t * n_out)
{
  std::ifstream in(path);
  if (!in) {
    *why = path + ": cannot open";
    return false;
  }
  why->clear();
  *out = TopicFile{};
  std::map<int64_t, size_t> index;  // id -> position in out->records
  bool header = false;
  size_t & n = *n_out;
  for (std::string line; std::getline(in, line); ) {
    ++n;
    if (line.empty()) {
      continue;
    }
    const nlohmann::json j = nlohmann::json::parse(line, nullptr, false);
    if (j.is_discarded() || !j.is_object() || !j.contains("ev") || !j["ev"].is_string()) {
      *why = at(path, n, "not a JSON object with \"ev\"");
      return false;
    }
    const std::string ev = j["ev"].get<std::string>();
    if (ev == "header") {
      if (header || n != 1) {
        *why = at(path, n, "a header that is not the first line");
        return false;
      }
      if (!header_ok(j, "external", path, why) || !j.contains("topic") ||
        !j["topic"].is_string() || !j.contains("type") || !j["type"].is_string() ||
        !is_int(j, "delay_ns") || !is_int(j, "ingress_reserve_ns") || !is_int(j, "grid_ns"))
      {
        if (why->empty()) {
          *why = at(path, n, "topic header lacks topic, type or the anchor constants");
        }
        return false;
      }
      if (j["delay_ns"].get<int64_t>() < 0 || j["ingress_reserve_ns"].get<int64_t>() < 0 ||
        j["grid_ns"].get<int64_t>() <= 0)
      {
        *why = at(path, n, "topic header with a negative constant or a grid that is not positive");
        return false;
      }
      out->topic = j["topic"].get<std::string>();
      out->type = j["type"].get<std::string>();
      out->start_ns = j["lf_time_start"].get<int64_t>();
      out->delay_ns = j["delay_ns"].get<int64_t>();
      out->reserve_ns = j["ingress_reserve_ns"].get<int64_t>();
      out->grid_ns = j["grid_ns"].get<int64_t>();
      header = true;
      continue;
    }
    if (!header) {
      *why = at(path, n, "an entry before the header");
      return false;
    }
    if (!is_int(j, "rid") || !is_int(j, "id") || !is_str(j, "disposition") ||
      j["rid"].get<int64_t>() < 0 || j["rid"].get<int64_t>() > INT32_MAX)
    {
      *why = at(path, n, "an entry without rid, id or disposition, or with a rid out of range");
      return false;
    }
    const int64_t id = j["id"].get<int64_t>();
    const std::string disposition = j["disposition"].get<std::string>();
    if (!known_disposition(disposition)) {
      *why = at(path, n, "record " + std::to_string(id) + ": unknown disposition '" +
          disposition + "'");
      return false;
    }
    if (ev == "correction") {
      auto it = index.find(id);
      if (it == index.end()) {
        *why = at(path, n, "a correction of record " + std::to_string(id) + " before its entry");
        return false;
      }
      Record & corrected = out->records[it->second];
      const int64_t rid = j["rid"].get<int64_t>();
      if (rid != corrected.rid) {
        // A record is (rid, id).
        *why = at(
          path, n, "a correction of record " + std::to_string(id) + " with rid " +
          std::to_string(rid) + ", but the record's entry has rid " +
          std::to_string(corrected.rid));
        return false;
      }
      corrected.disposition = disposition;
      continue;
    }
    if (ev != "external") {
      *why = at(path, n, "unknown entry '" + ev + "'");
      return false;
    }
    if (index.count(id) > 0) {
      *why = at(path, n, "a second external entry for record " + std::to_string(id));
      return false;
    }
    const std::string bad_field = external_fields_why(j);
    if (!bad_field.empty()) {
      *why = at(path, n, "record " + std::to_string(id) + ": " + bad_field);
      return false;
    }
    Record r;
    r.rid = static_cast<int>(j["rid"].get<int64_t>());
    r.id = id;
    r.stamp_ns = j["stamp_ns"].get<int64_t>();
    r.arrival_ns = j["arrival_ns"].get<int64_t>();
    if (!j["tag_ns"].is_null()) {
      r.tag_ns = j["tag_ns"].get<int64_t>();
    }
    r.disposition = disposition;
    if (!base64_decode(j["cdr"].get<std::string>(), &r.bytes)) {
      *why = at(path, n, "record " + std::to_string(id) + ": cdr is not base64");
      return false;
    }
    if (j["cdr_len"].get<int64_t>() != static_cast<int64_t>(r.bytes.size())) {
      *why = at(path, n, "record " + std::to_string(id) + ": cdr_len disagrees with cdr");
      return false;
    }
    index[id] = out->records.size();
    out->records.push_back(std::move(r));
  }
  if (!header) {
    *why = path + ": no header line";
    return false;
  }
  for (const Record & r : out->records) {
    if (r.disposition == "admitted" && (!r.tag_ns || r.bytes.empty())) {
      *why = path + ": admitted record " + std::to_string(r.id) +
        (r.tag_ns ? " has no message bytes" : " has no tag");
      return false;
    }
  }
  return true;
}

static bool load_stop_impl(const std::string & path, Stop * out, std::string * why, size_t * n_out)
{
  std::ifstream in(path);
  if (!in) {
    *why = path + ": cannot open";
    return false;
  }
  why->clear();
  size_t & n = *n_out;
  size_t stops = 0;
  bool header = false;
  for (std::string line; std::getline(in, line); ) {
    ++n;
    if (line.empty()) {
      continue;
    }
    const nlohmann::json j = nlohmann::json::parse(line, nullptr, false);
    if (j.is_discarded() || !j.is_object() || !j.contains("ev") || !j["ev"].is_string()) {
      *why = at(path, n, "not a JSON object with \"ev\"");
      return false;
    }
    const std::string ev = j["ev"].get<std::string>();
    if (ev == "header") {
      if (n != 1 || !header_ok(j, "run", path, why)) {
        if (why->empty()) {
          *why = at(path, n, "a header that is not the first line");
        }
        return false;
      }
      out->start_ns = j["lf_time_start"].get<int64_t>();
      header = true;
    } else if (ev == "lf_stop") {
      if (!header) {
        *why = at(path, n, "an lf_stop before the run header");
        return false;
      }
      if (!is_int(j, "tag_ns") || !is_int(j, "microstep") || j["microstep"].get<int64_t>() < 0 ||
        j["microstep"].get<int64_t>() > UINT32_MAX)
      {
        *why = at(path, n, "lf_stop without tag_ns or a microstep in range");
        return false;
      }
      out->tag_ns = j["tag_ns"].get<int64_t>();
      out->microstep = static_cast<uint32_t>(j["microstep"].get<int64_t>());
      if (j.contains("condition") && !j["condition"].is_string()) {
        *why = at(path, n, "lf_stop condition is not a string");
        return false;
      }
      out->condition = is_str(j, "condition") ? j["condition"].get<std::string>() : "";
      ++stops;
    }
  }
  if (!header) {
    *why = path + ": no run header";
    return false;
  }
  if (stops != 1) {
    *why = path + ": " + std::to_string(stops) +
      " lf_stop lines; a replay needs the live run's one stop tag";
    return false;
  }
  return true;
}

/// A json type or range error in a field the checks above do not cover is a
/// malformed line too, never an exception out of the startup reaction.
bool load_topic(const std::string & path, TopicFile * out, std::string * why)
{
  size_t n = 0;
  try {
    return load_topic_impl(path, out, why, &n);
  } catch (const nlohmann::json::exception & e) {
    *why = at(path, n, std::string("malformed value: ") + e.what());
    return false;
  }
}

bool load_stop(const std::string & path, Stop * out, std::string * why)
{
  size_t n = 0;
  try {
    return load_stop_impl(path, out, why, &n);
  } catch (const nlohmann::json::exception & e) {
    *why = at(path, n, std::string("malformed value: ") + e.what());
    return false;
  }
}

std::vector<std::string> topic_files(const std::string & d)
{
  namespace fs = std::filesystem;
  std::vector<std::string> out;
  std::error_code ec;
  for (fs::directory_iterator it(d, ec), end; !ec && it != end; it.increment(ec)) {
    const std::string name = it->path().filename().string();
    if (name.size() > 10 && name.rfind("ext.", 0) == 0 &&
      name.compare(name.size() - 6, 6, ".jsonl") == 0)
    {
      out.push_back(it->path().string());
    }
  }
  std::sort(out.begin(), out.end());
  return out;
}

void set_dir_for_test(const std::optional<std::string> & d)
{
  std::lock_guard<std::mutex> lk(g_mu);
  if (d) {
    dir_locked() = *d;
    g_read = true;
  } else {
    g_read = false;
    load_locked();
  }
}

}  // namespace plant_replay
}  // namespace ros2_lf_binding
