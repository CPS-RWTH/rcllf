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


// Oracle O1. Usage:
//   sync_oracle --corpus FILE [--policy exact|approx] [--queue N]
//               [--impl header|ref] [--mut first_wins|evict_oldest_arrival|
//               no_clear_old|q_plus1] [--short]
// Policy and queue default to the corpus' own. Prints one config record and
// one record per signal or drop callback in the canonical format of
// corpus.hpp (--short: the expect-block form, one per line).

#include <cstdio>
#include <exception>
#include <string>
#include <vector>

#include "sync_oracle_showcase/oracle.hpp"

int main(int argc, char ** argv)
{
  using sync_oracle_showcase::Corpus;
  std::string corpus_path;
  std::string policy;
  std::string impl = "header";
  std::string mutant;
  long queue = -1;  // NOLINT(runtime/int)
  bool short_form = false;
  for (int i = 1; i < argc; ++i) {
    const std::string a = argv[i];
    auto next = [&]() -> std::string {
        if (i + 1 >= argc) {
          throw std::runtime_error("missing value for " + a);
        }
        return argv[++i];
      };
    try {
      if (a == "--corpus") {
        corpus_path = next();
      } else if (a == "--policy") {
        policy = next();
      } else if (a == "--queue") {
        queue = std::stol(next());
      } else if (a == "--impl") {
        impl = next();
      } else if (a == "--mut") {
        mutant = next();
      } else if (a == "--short") {
        short_form = true;
      } else {
        std::fprintf(stderr, "unknown argument %s\n", a.c_str());
        return 2;
      }
    } catch (const std::exception & e) {
      std::fprintf(stderr, "%s\n", e.what());
      return 2;
    }
  }
  if (corpus_path.empty()) {
    std::fprintf(stderr, "usage: sync_oracle --corpus FILE [options]\n");
    return 2;
  }
  try {
    const Corpus c = sync_oracle_showcase::load_corpus(corpus_path);
    if (policy.empty()) {
      policy = c.policy;
    }
    const uint32_t q = queue >= 0 ? static_cast<uint32_t>(queue) : c.queue;
    const auto order = sync_oracle_showcase::arrival_order(c.rows);
    std::vector<sync_oracle_showcase::Event> events;
    if (impl == "header") {
      if (!mutant.empty()) {
        throw std::runtime_error("mutants apply to --impl ref only");
      }
      events = sync_oracle_showcase::run_header(order, policy, q);
    } else if (impl == "ref") {
      if (policy != "exact") {
        throw std::runtime_error("--impl ref restates ExactTime only");
      }
      events = sync_oracle_showcase::run_ref(order, q, mutant);
    } else {
      throw std::runtime_error("unknown impl " + impl);
    }
    if (!short_form) {
      std::string cfg = sync_oracle_showcase::config_json(policy, q);
      cfg += ",\"corpus\":\"" + c.digest + "\",\"impl\":\"" + impl + "\"";
      if (!mutant.empty()) {
        cfg += ",\"mutant\":\"" + mutant + "\"";
      }
      std::printf("%s}\n", cfg.c_str());
    }
    for (size_t k = 0; k < events.size(); ++k) {
      const std::string line = short_form ?
        sync_oracle_showcase::event_short(events[k]) :
        sync_oracle_showcase::event_json(events[k], k);
      std::printf("%s\n", line.c_str());
    }
  } catch (const std::exception & e) {
    std::fprintf(stderr, "sync_oracle: %s\n", e.what());
    return 2;
  }
  return 0;
}
