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

// Oracle O1 checks: the library output equals each corpus'
// independently derived expect block, the ref restatement equals the library
// on every ExactTime corpus and queue, and each single-flag mutant diverges on
// F1 at the hand-derived seq.

#include <gtest/gtest.h>

#include <algorithm>
#include <filesystem>
#include <map>
#include <string>
#include <vector>

#include "sync_oracle_showcase/oracle.hpp"

namespace sos = sync_oracle_showcase;

namespace
{

std::vector<std::string> corpus_files()
{
  std::vector<std::string> out;
  for (const auto & e : std::filesystem::directory_iterator(SYNC_CORPUS_DIR)) {
    if (e.path().extension() == ".yaml") {
      out.push_back(e.path().string());
    }
  }
  std::sort(out.begin(), out.end());
  return out;
}

std::vector<std::string> shorts(const std::vector<sos::Event> & events)
{
  std::vector<std::string> out;
  for (const auto & e : events) {
    out.push_back(sos::event_short(e));
  }
  return out;
}

sos::Corpus corpus(const std::string & name)
{
  return sos::load_corpus(std::string(SYNC_CORPUS_DIR) + "/" + name + ".yaml");
}

// First seq where two streams differ (a missing event counts), -1 if equal.
long first_divergence(const std::vector<sos::Event> & a, const std::vector<sos::Event> & b)  // NOLINT
{
  const size_t n = std::max(a.size(), b.size());
  for (size_t k = 0; k < n; ++k) {
    if (k >= a.size() || k >= b.size() || sos::event_json(a[k], k) != sos::event_json(b[k], k)) {
      return static_cast<long>(k);  // NOLINT(runtime/int)
    }
  }
  return -1;
}

}  // namespace

TEST(SyncOracle, CorpusDirectoryHoldsEveryCase)
{
  std::vector<std::string> names;
  for (const auto & f : corpus_files()) {
    names.push_back(sos::load_corpus(f).name);
  }
  const std::vector<std::string> want = {
    "A1", "A2", "A3", "A4", "B1", "B2", "B3", "B4", "B5", "B6", "B7", "B8", "F1", "N1", "P1",
    "P2", "P3"};
  std::sort(names.begin(), names.end());
  EXPECT_EQ(names, want);
}

TEST(SyncOracle, ArrivalOrderIsTickThenInputThenPublishOrder)
{
  const auto order = sos::arrival_order(corpus("b8").rows);
  std::vector<uint64_t> rows;
  for (const auto & r : order) {
    rows.push_back(r.row);
  }
  EXPECT_EQ(rows, (std::vector<uint64_t>{0, 1, 2, 3}));
}

TEST(SyncOracle, LibraryEqualsTheIndependentlyDerivedExpectBlocks)
{
  size_t checked = 0;
  for (const auto & f : corpus_files()) {
    const auto c = sos::load_corpus(f);
    if (c.kind == "forkjoin" || c.expect.empty()) {
      continue;  // P3 is the join's derivation, N1 is out of the domain
    }
    const auto got = shorts(sos::run_header(sos::arrival_order(c.rows), c.policy, c.queue));
    EXPECT_EQ(got, c.expect) << c.name;
    ++checked;
  }
  EXPECT_EQ(checked, 15u);
}

TEST(SyncOracle, RefEqualsLibraryOnEveryExactCorpusAndQueue)
{
  for (const auto & f : corpus_files()) {
    const auto c = sos::load_corpus(f);
    const auto order = sos::arrival_order(c.rows);
    for (uint32_t q : {1u, 2u, 3u}) {
      EXPECT_EQ(
        shorts(sos::run_ref(order, q, "")), shorts(sos::run_header(order, "exact", q)))
        << c.name << " q " << q;
    }
  }
}

TEST(SyncOracle, EachMutantDivergesOnF1AtTheHandDerivedSeq)
{
  const auto c = corpus("f1");
  const auto order = sos::arrival_order(c.rows);
  const auto lib = sos::run_header(order, "exact", c.queue);
  const std::map<std::string, long> want = {  // NOLINT(runtime/int)
    {"first_wins", 1}, {"evict_oldest_arrival", 0}, {"no_clear_old", 3}, {"q_plus1", 0}};
  for (const auto & [mutant, seq] : want) {
    EXPECT_EQ(first_divergence(lib, sos::run_ref(order, c.queue, mutant)), seq) << mutant;
  }
}
