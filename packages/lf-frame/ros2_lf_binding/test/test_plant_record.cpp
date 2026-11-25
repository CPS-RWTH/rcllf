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

/// @file test_plant_record.cpp
/// @brief The recorder's encodings, stop condition and file
/// format, and that it writes nothing when off.

#include <gtest/gtest.h>

#include <cstdint>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <string>
#include <vector>

#include "plant_record.hpp"
#include "rcutils/allocator.h"
#include "rmw/ret_types.h"
#include "rmw/serialized_message.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "std_msgs/msg/multi_array_dimension.hpp"

namespace rec = ros2_lf_binding::plant_record;
namespace fs = std::filesystem;

namespace
{

std::vector<std::string> lines_of(const fs::path & p)
{
  std::vector<std::string> out;
  std::ifstream in(p);
  for (std::string l; std::getline(in, l); ) {
    out.push_back(l);
  }
  return out;
}

fs::path fresh_dir(const char * name)
{
  fs::path d = fs::temp_directory_path() / name;
  fs::remove_all(d);
  return d;
}

std::string b64(const std::string & s)
{
  return rec::base64(reinterpret_cast<const uint8_t *>(s.data()), s.size());
}

}  // namespace

// RFC 4648 section 10 test vectors.
TEST(PlantRecord, Base64MatchesRfc4648)
{
  EXPECT_EQ(b64(""), "");
  EXPECT_EQ(b64("f"), "Zg==");
  EXPECT_EQ(b64("fo"), "Zm8=");
  EXPECT_EQ(b64("foo"), "Zm9v");
  EXPECT_EQ(b64("foob"), "Zm9vYg==");
  EXPECT_EQ(b64("fooba"), "Zm9vYmE=");
  EXPECT_EQ(b64("foobar"), "Zm9vYmFy");
}

// FNV-1a 64 reference values (offset basis; "a"; "foobar").
TEST(PlantRecord, Fnv1a64MatchesReferenceValues)
{
  EXPECT_EQ(rec::fnv1a64(nullptr, 0), 0xcbf29ce484222325ULL);
  const std::string a = "a";
  EXPECT_EQ(rec::fnv1a64(reinterpret_cast<const uint8_t *>(a.data()), 1), 0xaf63dc4c8601ec8cULL);
  const std::string f = "foobar";
  EXPECT_EQ(
    rec::fnv1a64(reinterpret_cast<const uint8_t *>(f.data()), f.size()), 0x85944171f73967e8ULL);
}

// The publish lines hash the CDR bytes, and the CDR writer leaves the
// alignment padding unwritten. A buffer that held other bytes must still
// hash as if the padding were zero, or one message records two hashes (the
// a8 fixture test saw a /drive payload hash change between two calls).
TEST(PlantRecord, SerializedPaddingIsZeroWhateverTheBufferHeld)
{
  std_msgs::msg::MultiArrayDimension m;
  m.label = "ab";  // CDR: u32 length 3, "ab\0", one padding byte, u32 size, u32 stride
  m.size = 5;
  m.stride = 7;
  // Expected bytes derived by hand from the CDR rules, independent of rmw:
  // encapsulation CDR_LE (00 01 00 00), then the fields little endian.
  const std::vector<uint8_t> want{
    0x00, 0x01, 0x00, 0x00, 0x03, 0x00, 0x00, 0x00, 'a', 'b', 0x00, 0x00,
    0x05, 0x00, 0x00, 0x00, 0x07, 0x00, 0x00, 0x00};
  const auto * ts = rosidl_typesupport_cpp::get_message_type_support_handle<
    std_msgs::msg::MultiArrayDimension>();
  rmw_serialized_message_t buf = rmw_get_zero_initialized_serialized_message();
  rcutils_allocator_t alloc = rcutils_get_default_allocator();
  ASSERT_EQ(rmw_serialized_message_init(&buf, 64, &alloc), RMW_RET_OK);
  std::memset(buf.buffer, 0xab, buf.buffer_capacity);  // what a reused heap block may hold
  ASSERT_TRUE(rec::serialize_zero_padded(&m, ts, &buf));
  ASSERT_EQ(buf.buffer_length, want.size());
  EXPECT_EQ(static_cast<int>(buf.buffer[11]), 0) << "padding byte 11 is " << std::hex <<
    static_cast<int>(buf.buffer[11]) << ": the recorder hashed what the buffer held before";
  EXPECT_EQ(
    rec::fnv1a64(buf.buffer, buf.buffer_length), rec::fnv1a64(want.data(), want.size())) <<
    "the payload hash depends on the buffer's earlier contents";
  EXPECT_EQ(rmw_serialized_message_fini(&buf), RMW_RET_OK);
}

// The stop condition, from reactor-c: a timeout sets (start + duration, 0)
// (reactor_threaded.c:521), lf_request_stop the current tag plus one
// microstep (reactor_threaded.c:456-460).
TEST(PlantRecord, StopConditionTellsTimeoutFromRequestStop)
{
  EXPECT_STREQ(rec::stop_condition(1000, 6000, 7000, 0), "timeout");
  EXPECT_STREQ(rec::stop_condition(1000, 6000, 7000, 1), "request_stop");
  EXPECT_STREQ(rec::stop_condition(1000, 6000, 3000, 1), "request_stop");
  EXPECT_STREQ(rec::stop_condition(1000, -1, 3000, 1), "request_stop");
  EXPECT_STREQ(rec::stop_condition(1000, -1, 3000, 0), "other");
  EXPECT_STREQ(rec::stop_condition(1000, 6000, 3000, 0), "other");
}

TEST(PlantRecord, TopicFileNames)
{
  EXPECT_EQ(rec::topic_file("/stamped_in"), "ext.stamped_in.jsonl");
  EXPECT_EQ(rec::topic_file("/car/scan"), "ext.car.scan.jsonl");
}

// The file format a reader (and replay) depends on: header
// first, one external line per record, the correction last.
TEST(PlantRecord, WritesHeaderEntriesAndCorrectionInOrder)
{
  const fs::path d = fresh_dir("rcllf_plant_record_format");
  rec::set_dir_for_test(d.string());
  ASSERT_TRUE(rec::enabled());
  rec::open_topic("/in", "geometry_msgs/msg/PointStamped", "rmw_x", 1000, 200, 50, 10);
  rec::open_topic("/in", "geometry_msgs/msg/PointStamped", "rmw_x", 1000, 200, 50, 10);
  rec::External a;
  a.rid = 3;
  a.id = 7;
  a.stamp_ns = 1100;
  a.arrival_ns = 1150;
  a.insert_ns = 1160;
  a.tag_ns = 360;
  a.disposition = "admitted";
  a.serialize_ns = 42;
  a.bytes = {'f', 'o', 'o'};
  rec::external("/in", a);
  rec::External l;
  l.rid = 3;
  l.id = 8;
  l.stamp_ns = 900;
  l.arrival_ns = 1200;
  l.disposition = "late";
  rec::external("/in", l);
  rec::correction("/in", 3, 7, "collision");
  rec::close();
  rec::set_dir_for_test(std::string());

  const auto lines = lines_of(d / "ext.in.jsonl");
  ASSERT_EQ(lines.size(), 4u);
  EXPECT_EQ(
    lines[0], "{\"ev\":\"header\",\"format\":\"rcllf_plant_record\",\"version\":1,"
    "\"kind\":\"external\",\"topic\":\"/in\",\"type\":\"geometry_msgs/msg/PointStamped\","
    "\"serialization\":\"rmw_serialize\",\"rmw\":\"rmw_x\",\"lf_time_start\":1000,"
    "\"delay_ns\":200,\"ingress_reserve_ns\":50,\"grid_ns\":10}");
  EXPECT_EQ(
    lines[1], "{\"ev\":\"external\",\"rid\":3,\"id\":7,\"stamp_ns\":1100,\"arrival_ns\":1150,"
    "\"insert_ns\":1160,\"tag_ns\":360,\"disposition\":\"admitted\",\"serialize_ns\":42,"
    "\"cdr_len\":3,\"cdr\":\"Zm9v\"}");
  EXPECT_EQ(
    lines[2], "{\"ev\":\"external\",\"rid\":3,\"id\":8,\"stamp_ns\":900,\"arrival_ns\":1200,"
    "\"insert_ns\":null,\"tag_ns\":null,\"disposition\":\"late\",\"serialize_ns\":-1,"
    "\"cdr_len\":0,\"cdr\":\"\"}");
  EXPECT_EQ(lines[3], "{\"ev\":\"correction\",\"rid\":3,\"id\":7,\"disposition\":\"collision\"}");
  // Outside an LF program the teardown still records lf_stop (source teardown).
  const auto run = lines_of(d / "run.jsonl");
  ASSERT_EQ(run.size(), 2u);
  EXPECT_NE(run[1].find("\"ev\":\"lf_stop\""), std::string::npos) << run[1];
  EXPECT_NE(run[1].find("\"source\":\"teardown\""), std::string::npos) << run[1];
}

// Two ingress threads may enqueue a correction before the
// entry it corrects. The writer holds it back until the entry is written.
TEST(PlantRecord, CorrectionNeverPrecedesItsEntry)
{
  const fs::path d = fresh_dir("rcllf_plant_record_reorder");
  rec::set_dir_for_test(d.string());
  rec::open_topic("/in", "t", "r", 1000, 200, 50, 10);
  rec::correction("/in", 3, 7, "collision");  // queued first, as the race would
  rec::External a;
  a.rid = 3;
  a.id = 7;
  a.disposition = "admitted";
  rec::external("/in", a);
  rec::close();
  rec::set_dir_for_test(std::string());
  const auto lines = lines_of(d / "ext.in.jsonl");
  ASSERT_EQ(lines.size(), 3u);
  EXPECT_NE(lines[1].find("\"ev\":\"external\""), std::string::npos)
    << "a correction was written before the entry it corrects: " << lines[1];
  EXPECT_NE(lines[2].find("\"ev\":\"correction\""), std::string::npos) << lines[2];
}

// lf_stop is the run's last line even when a later
// shutdown reaction records after it.
TEST(PlantRecord, LfStopIsTheLastRunLine)
{
  const fs::path d = fresh_dir("rcllf_plant_record_stop_last");
  rec::set_dir_for_test(d.string());
  rec::lf_stop("shutdown_reaction");
  rec::actuation_late(0, "/a", 1, 0, 2, 3, 4);
  rec::close();
  rec::set_dir_for_test(std::string());
  const auto run = lines_of(d / "run.jsonl");
  ASSERT_EQ(run.size(), 3u);
  EXPECT_NE(run[2].find("\"ev\":\"lf_stop\""), std::string::npos)
    << "a line follows lf_stop: " << run[2];
  EXPECT_NE(run[2].find("\"source\":\"shutdown_reaction\""), std::string::npos) << run[2];
}

// Off: no directory, no file, nothing queued.
TEST(PlantRecord, OffWritesNothing)
{
  const fs::path d = fresh_dir("rcllf_plant_record_off");
  rec::set_dir_for_test(std::string());
  EXPECT_FALSE(rec::enabled());
  rec::open_topic("/in", "t", "r", 1, 2, 3, 4);
  rec::External e;
  e.disposition = "admitted";
  rec::external("/in", e);
  rec::correction("/in", 0, 1, "collision");
  rec::actuation_late(0, "/a", 1, 0, 2, 3, 4);
  rec::lf_stop("test");
  rec::close();
  EXPECT_FALSE(fs::exists(d));
}
