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

// Reading a recording for replay. The replay path itself is
// tested in test_external_input.cpp (StampReplayTest) on the binding's own
// recording; this file covers the reader's refusals.

#include <gtest/gtest.h>

#include <cstdint>
#include <filesystem>
#include <fstream>
#include <string>
#include <vector>

#include "plant_record.hpp"
#include "plant_replay.hpp"

namespace fs = std::filesystem;
namespace rec = ros2_lf_binding::plant_record;
namespace rep = ros2_lf_binding::plant_replay;

namespace
{

fs::path write(const std::string & name, const std::vector<std::string> & lines)
{
  const fs::path p = fs::temp_directory_path() / ("rcllf_replay_800_" + name);
  std::ofstream out(p);
  for (const auto & l : lines) {
    out << l << "\n";
  }
  return p;
}

std::string header()
{
  return "{\"ev\":\"header\",\"format\":\"rcllf_plant_record\",\"version\":" +
         std::to_string(rec::kFormatVersion) + ",\"kind\":\"external\",\"topic\":\"/a\","
         "\"type\":\"geometry_msgs/msg/PointStamped\",\"lf_time_start\":5,\"delay_ns\":1,"
         "\"ingress_reserve_ns\":2,\"grid_ns\":3}";
}

/// An external entry with every field plant_record::external() writes.
std::string entry(int id, const char * disp, const std::string & cdr)
{
  return "{\"ev\":\"external\",\"rid\":0,\"id\":" + std::to_string(id) +
         ",\"stamp_ns\":7,\"arrival_ns\":8,\"insert_ns\":" + (cdr.empty() ? "null" : "9") +
         ",\"tag_ns\":" + (cdr.empty() ? "null" : "100") + ",\"disposition\":\"" + disp +
         "\",\"serialize_ns\":4,\"cdr_len\":" + std::to_string(cdr.empty() ? 0 : 3) +
         ",\"cdr\":\"" + cdr + "\"}";
}

/// `line` with the json member `key` (and its value, up to the next ',' or
/// '}') replaced by `with`; an empty `with` drops the member.
std::string patch(std::string line, const std::string & key, const std::string & with)
{
  const std::string k = "\"" + key + "\":";
  const size_t at = line.find(k);
  EXPECT_NE(at, std::string::npos) << key << " in " << line;
  size_t end = at + k.size();
  if (line[end] == '"') {
    end = line.find('"', end + 1) + 1;
  } else {
    end = line.find_first_of(",}", end);
  }
  if (!with.empty()) {
    return line.replace(at, end - at, k + with);
  }
  // drop the member and the comma that joins it to its neighbour
  if (line[end] == ',') {
    return line.erase(at, end + 1 - at);
  }
  return line.erase(at - 1, end - at + 1);
}

}  // namespace

TEST(PlantReplay, Base64RoundTripsTheRecorderAndRefusesGarbage)
{
  for (size_t n = 0; n < 7; ++n) {
    std::vector<uint8_t> in(n);
    for (size_t i = 0; i < n; ++i) {
      in[i] = static_cast<uint8_t>(0xf0 + 7 * i);
    }
    std::vector<uint8_t> out;
    ASSERT_TRUE(rep::base64_decode(rec::base64(in.data(), in.size()), &out)) << n;
    EXPECT_EQ(out, in) << n;
  }
  std::vector<uint8_t> out;
  EXPECT_FALSE(rep::base64_decode("QUJD=", &out)) << "length not a multiple of 4";
  EXPECT_FALSE(rep::base64_decode("Q=JD", &out)) << "'=' inside";
  EXPECT_FALSE(rep::base64_decode("QU*D", &out)) << "not in the alphabet";
}

TEST(PlantReplay, LastEntryPerIdIsTheDisposition)
{
  rep::TopicFile tf;
  std::string why;
  const auto p = write(
    "ok.jsonl", {header(), entry(1, "admitted", "QUJD"), entry(2, "late", ""),
      "{\"ev\":\"correction\",\"rid\":0,\"id\":1,\"disposition\":\"collision\"}"});
  ASSERT_TRUE(rep::load_topic(p.string(), &tf, &why)) << why;
  EXPECT_EQ(tf.topic, "/a");
  EXPECT_EQ(tf.start_ns, 5);
  EXPECT_EQ(tf.grid_ns, 3);
  ASSERT_EQ(tf.records.size(), 2u);
  EXPECT_EQ(tf.records[0].disposition, "collision");
  EXPECT_EQ(tf.records[0].bytes, (std::vector<uint8_t>{'A', 'B', 'C'}));
  EXPECT_EQ(tf.records[1].disposition, "late");
  EXPECT_FALSE(tf.records[1].tag_ns.has_value());
}

TEST(PlantReplay, MalformedRecordingsAreRefusedWithTheLine)
{
  rep::TopicFile tf;
  std::string why;
  EXPECT_FALSE(
    rep::load_topic(
      write(
        "corr.jsonl",
        {header(), "{\"ev\":\"correction\",\"rid\":0,\"id\":9,\"disposition\":\"collision\"}"})
      .string(), &tf, &why));
  EXPECT_NE(why.find(":2: a correction of record 9 before its entry"), std::string::npos) << why;
  EXPECT_FALSE(
    rep::load_topic(
      write("dup.jsonl", {header(), entry(1, "admitted", "QUJD"), entry(1, "admitted", "QUJD")})
      .string(), &tf, &why));
  EXPECT_NE(why.find(":3: a second external entry for record 1"), std::string::npos) << why;
  EXPECT_FALSE(
    rep::load_topic(write("nohdr.jsonl", {entry(1, "admitted", "QUJD")}).string(), &tf, &why));
  EXPECT_NE(why.find(":1: an entry before the header"), std::string::npos) << why;
  std::string bad = entry(1, "admitted", "QUJD");
  bad.replace(bad.find("\"cdr_len\":3"), 11, "\"cdr_len\":4");
  EXPECT_FALSE(rep::load_topic(write("len.jsonl", {header(), bad}).string(), &tf, &why));
  EXPECT_NE(why.find("cdr_len disagrees with cdr"), std::string::npos) << why;
  std::string v2 = header();
  v2.replace(v2.find("\"version\":"), 11, "\"version\":9");
  EXPECT_FALSE(rep::load_topic(write("ver.jsonl", {v2}).string(), &tf, &why));
  EXPECT_NE(why.find("not an rcllf_plant_record version"), std::string::npos) << why;
}

// A misspelled disposition or a value
// of the wrong json type is a refused recording, never a skipped record or an
// exception out of the startup reaction.
TEST(PlantReplay, UnknownDispositionsAndWrongTypesAreRefused)
{
  rep::TopicFile tf;
  std::string why;
  EXPECT_FALSE(
    rep::load_topic(
      write("typo.jsonl", {header(), entry(1, "admittted", "QUJD")}).string(), &tf, &why));
  EXPECT_NE(why.find(":2: record 1: unknown disposition 'admittted'"), std::string::npos) << why;
  EXPECT_FALSE(
    rep::load_topic(
      write(
        "corrtypo.jsonl",
        {header(), entry(1, "admitted", "QUJD"),
          "{\"ev\":\"correction\",\"rid\":0,\"id\":1,\"disposition\":\"colision\"}"})
      .string(), &tf, &why));
  EXPECT_NE(why.find(":3: record 1: unknown disposition 'colision'"), std::string::npos) << why;
  std::string f42 = header();
  f42.replace(f42.find("\"rcllf_plant_record\""), 20, "42");
  EXPECT_FALSE(rep::load_topic(write("fmt.jsonl", {f42}).string(), &tf, &why));
  EXPECT_NE(why.find(":1: not an rcllf_plant_record version"), std::string::npos) << why;
  std::string tagstr = entry(1, "admitted", "QUJD");
  tagstr.replace(tagstr.find("\"tag_ns\":100"), 12, "\"tag_ns\":\"x\"");
  EXPECT_FALSE(rep::load_topic(write("tagstr.jsonl", {header(), tagstr}).string(), &tf, &why));
  EXPECT_NE(why.find(":2: record 1: tag_ns is missing or not an integer or null"),
    std::string::npos) << why;
  const std::string hdr = "{\"ev\":\"header\",\"format\":\"rcllf_plant_record\",\"version\":" +
    std::to_string(rec::kFormatVersion) + ",\"kind\":\"run\",\"lf_time_start\":5}";
  rep::Stop s;
  EXPECT_FALSE(
    rep::load_stop(
      write("runms.jsonl", {hdr, "{\"ev\":\"lf_stop\",\"tag_ns\":6,\"microstep\":-1}"})
      .string(), &s, &why));
  EXPECT_NE(why.find(":2: lf_stop without tag_ns or a microstep in range"), std::string::npos) <<
    why;
}

// A present `condition` that is not
// a string is a malformed run.jsonl, not a stop without a condition.
TEST(PlantReplay, NonStringStopConditionIsRefused)
{
  const std::string hdr = "{\"ev\":\"header\",\"format\":\"rcllf_plant_record\",\"version\":" +
    std::to_string(rec::kFormatVersion) + ",\"kind\":\"run\",\"lf_time_start\":5}";
  rep::Stop s;
  std::string why;
  EXPECT_FALSE(
    rep::load_stop(
      write("runcond.jsonl", {hdr, "{\"ev\":\"lf_stop\",\"tag_ns\":6,\"microstep\":0,"
        "\"condition\":7}"}).string(), &s, &why)) << "condition 7 read as '" << s.condition <<
    "'";
  EXPECT_NE(why.find(":2: lf_stop condition is not a string"), std::string::npos) << why;
  ASSERT_TRUE(
    rep::load_stop(
      write("runnocond.jsonl", {hdr, "{\"ev\":\"lf_stop\",\"tag_ns\":6,\"microstep\":0}"})
      .string(), &s, &why)) << why;
  EXPECT_EQ(s.condition, "") << "an absent condition stays optional";
}

// A correction names its record by
// rid and id; one whose rid is not its record's rid corrects nothing.
TEST(PlantReplay, CorrectionWithAnotherRidIsRefused)
{
  rep::TopicFile tf;
  std::string why;
  const bool ok = rep::load_topic(
    write(
      "corrrid.jsonl",
      {header(), entry(1, "admitted", "QUJD"),
        "{\"ev\":\"correction\",\"rid\":1,\"id\":1,\"disposition\":\"collision\"}"})
    .string(), &tf, &why);
  EXPECT_FALSE(ok) << "rid-1 correction applied to rid-0 record 1, disposition now '" <<
    (tf.records.empty() ? std::string("?") : tf.records[0].disposition) << "'";
  EXPECT_NE(
    why.find(":3: a correction of record 1 with rid 1, but the record's entry has rid 0"),
    std::string::npos) << why;
}

// Every field plant_record::external()
// always writes is required with its type; a present optional one must have
// its type; cdr_len must equal the decoded length.
TEST(PlantReplay, ExternalEntryFieldsAreRequiredAndTyped)
{
  rep::TopicFile tf;
  std::string why;
  const std::string good = entry(1, "admitted", "QUJD");
  ASSERT_TRUE(rep::load_topic(write("typed_ok.jsonl", {header(), good}).string(), &tf, &why)) <<
    why;
  ASSERT_EQ(tf.records.size(), 1u);
  EXPECT_EQ(tf.records[0].stamp_ns, 7);
  EXPECT_EQ(tf.records[0].arrival_ns, 8);
  ASSERT_TRUE(
    rep::load_topic(
      write("typed_ro.jsonl", {header(), patch(good, "serialize_ns", "4,\"replay_of\":3")})
      .string(), &tf, &why)) << why;
  struct Case
  {
    const char * key;
    const char * with;  ///< "" drops the field
    const char * says;
  };
  const Case cases[] = {
    {"stamp_ns", "\"7\"", "stamp_ns is missing or not an integer"},
    {"stamp_ns", "", "stamp_ns is missing or not an integer"},
    {"arrival_ns", "8.5", "arrival_ns is missing or not an integer"},
    {"arrival_ns", "", "arrival_ns is missing or not an integer"},
    {"serialize_ns", "null", "serialize_ns is missing or not an integer"},
    {"serialize_ns", "", "serialize_ns is missing or not an integer"},
    {"insert_ns", "\"x\"", "insert_ns is missing or not an integer or null"},
    {"insert_ns", "", "insert_ns is missing or not an integer or null"},
    {"tag_ns", "", "tag_ns is missing or not an integer or null"},
    {"cdr_len", "\"3\"", "cdr_len is missing or not an integer"},
    {"cdr_len", "", "cdr_len is missing or not an integer"},
    {"cdr", "7", "cdr is missing or not a string"},
    {"cdr", "", "cdr is missing or not a string"},
    {"serialize_ns", "4,\"replay_of\":\"3\"", "replay_of is not an integer"},
  };
  for (const Case & c : cases) {
    const std::string line = patch(good, c.key, c.with);
    const bool ok = rep::load_topic(write("typed.jsonl", {header(), line}).string(), &tf, &why);
    EXPECT_FALSE(ok) << c.key << "=" << c.with << " accepted: stamp_ns " <<
      (tf.records.empty() ? -1 : tf.records[0].stamp_ns) << ", arrival_ns " <<
      (tf.records.empty() ? -1 : tf.records[0].arrival_ns) << " in " << line;
    EXPECT_NE(why.find(std::string(":2: record 1: ") + c.says), std::string::npos) <<
      c.key << "=" << c.with << ": " << why;
  }
  // cdr_len is checked against the decoded bytes for every entry, not only admitted ones
  const std::string late = patch(entry(2, "late", ""), "cdr_len", "5");
  EXPECT_FALSE(rep::load_topic(write("typed_len.jsonl", {header(), late}).string(), &tf, &why));
  EXPECT_NE(why.find(":2: record 2: cdr_len disagrees with cdr"), std::string::npos) << why;
}

TEST(PlantReplay, StopNeedsExactlyOneLfStop)
{
  const std::string hdr = "{\"ev\":\"header\",\"format\":\"rcllf_plant_record\",\"version\":" +
    std::to_string(rec::kFormatVersion) + ",\"kind\":\"run\",\"lf_time_start\":5}";
  const std::string stop =
    "{\"ev\":\"lf_stop\",\"tag_ns\":6000000000,\"microstep\":1,\"condition\":\"request_stop\"}";
  rep::Stop s;
  std::string why;
  ASSERT_TRUE(rep::load_stop(write("run1.jsonl", {hdr, stop}).string(), &s, &why)) << why;
  EXPECT_EQ(s.tag_ns, 6000000000LL);
  EXPECT_EQ(s.microstep, 1u);
  EXPECT_EQ(s.condition, "request_stop");
  EXPECT_EQ(s.start_ns, 5);
  EXPECT_FALSE(rep::load_stop(write("run0.jsonl", {hdr}).string(), &s, &why));
  EXPECT_NE(why.find("0 lf_stop lines"), std::string::npos) << why;
  EXPECT_FALSE(rep::load_stop(write("run2.jsonl", {hdr, stop, stop}).string(), &s, &why));
  EXPECT_NE(why.find("2 lf_stop lines"), std::string::npos) << why;
}
