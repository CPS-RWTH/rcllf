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

/// bna_replay: feed a JSONL corpus of BNA input events (params, admit,
/// missing, wake, close) through the C++ engine and print the input and
/// output records as JSONL, the same lines the runtime traces after the
/// `RCLLF_BNA ` prefix. Used by tools/lf_eval/tests/bna/test_replay.py to
/// check the engine against the Python oracle.
///
///   bna_replay --corpus FILE [--mutant NAME]

#include <cstdint>
#include <cstdlib>
#include <cstdio>
#include <fstream>
#include <iostream>
#include <map>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "rcllf_message_filters/bna_engine.hpp"

namespace bna = rcllf_message_filters::bna;

namespace
{

/// A value of a flat JSON object: string, integer, integer array or null.
struct Value
{
  enum Kind {STR, INT, ARR, NUL} kind = NUL;
  std::string str;
  std::string text;  // the integer as written
  int64_t num = 0;
  std::vector<int64_t> arr;
};

using Object = std::map<std::string, Value>;

class Parser
{
public:
  explicit Parser(const std::string & s)
  : s_(s) {}

  Object object()
  {
    Object o;
    expect('{');
    ws();
    if (peek() == '}') {
      ++i_;
      return o;
    }
    for (;; ) {
      ws();
      std::string k = string();
      ws();
      expect(':');
      ws();
      o[k] = value();
      ws();
      if (peek() == ',') {
        ++i_;
        continue;
      }
      expect('}');
      return o;
    }
  }

private:
  char peek() const {return i_ < s_.size() ? s_[i_] : '\0';}
  void ws()
  {
    while (i_ < s_.size() && (s_[i_] == ' ' || s_[i_] == '\t' || s_[i_] == '\r')) {
      ++i_;
    }
  }
  void expect(char c)
  {
    if (peek() != c) {
      throw std::runtime_error(std::string("expected '") + c + "' at column " +
              std::to_string(i_));
    }
    ++i_;
  }
  std::string string()
  {
    expect('"');
    std::string o;
    while (peek() != '"') {
      if (peek() == '\0') {
        throw std::runtime_error("unterminated string");
      }
      if (peek() == '\\') {
        ++i_;
        const char e = peek();
        if (e == 'u') {
          o += static_cast<char>(std::stoi(s_.substr(i_ + 1, 4), nullptr, 16));
          i_ += 5;
          continue;
        }
        o += e == 'n' ? '\n' : e == 't' ? '\t' : e;
        ++i_;
        continue;
      }
      o += s_[i_++];
    }
    ++i_;
    return o;
  }
  int64_t integer()
  {
    const size_t start = i_;
    if (peek() == '-') {
      ++i_;
    }
    while (peek() >= '0' && peek() <= '9') {
      ++i_;
    }
    if (start == i_) {
      throw std::runtime_error("expected a value at column " + std::to_string(i_));
    }
    // Saturates instead of throwing: an epoch above INT64_MAX is read from `text`.
    return std::strtoll(s_.substr(start, i_ - start).c_str(), nullptr, 10);
  }
  Value value()
  {
    Value v;
    if (peek() == '"') {
      v.kind = Value::STR;
      v.str = string();
    } else if (s_.compare(i_, 4, "null") == 0) {
      i_ += 4;
    } else if (peek() == '[') {
      ++i_;
      v.kind = Value::ARR;
      ws();
      while (peek() != ']') {
        ws();
        v.arr.push_back(integer());
        ws();
        if (peek() == ',') {
          ++i_;
        }
      }
      ++i_;
    } else {
      v.kind = Value::INT;
      const size_t start = i_;
      v.num = integer();
      v.text = s_.substr(start, i_ - start);
    }
    return v;
  }

  const std::string & s_;
  size_t i_ = 0;
};

const Value & field(const Object & o, const std::string & k)
{
  auto it = o.find(k);
  if (it == o.end()) {
    throw std::runtime_error("missing field '" + k + "'");
  }
  return it->second;
}

int64_t num(const Object & o, const std::string & k) {return field(o, k).num;}
const std::string & str(const Object & o, const std::string & k) {return field(o, k).str;}

bna::Tag tag(const Object & o)
{
  const auto & a = field(o, "tag").arr;
  if (a.size() != 2) {
    throw std::runtime_error("tag must be [time, microstep]");
  }
  return bna::Tag{a[0], static_cast<uint32_t>(a[1])};
}

bna::Key key(const Object & o)
{
  bna::Key k;
  if (!bna::Key::parse(str(o, "key"), &k)) {
    throw std::runtime_error("malformed key " + str(o, "key"));
  }
  return k;
}

bna::Params params(const Object & o)
{
  bna::Params p;
  p.epoch = std::stoull(field(o, "epoch").text);
  p.delta = num(o, "delta");
  const auto & l = field(o, "lateness").arr;
  const auto & c = field(o, "capacity").arr;
  if (l.size() != 2 || c.size() != 2) {
    throw std::runtime_error("lateness and capacity take two values");
  }
  p.lateness[0] = l[0];
  p.lateness[1] = l[1];
  p.capacity[0] = c[0];
  p.capacity[1] = c[1];
  p.retention = num(o, "retention");
  p.timeout = num(o, "timeout");
  return p;
}

int usage()
{
  std::cerr << "usage: bna_replay --corpus FILE [--mutant NAME]\n";
  return 2;
}

}  // namespace

int main(int argc, char ** argv)
{
  std::string corpus;
  std::string mutant = "none";
  for (int i = 1; i < argc; ++i) {
    const std::string a = argv[i];
    if (a == "--corpus" && i + 1 < argc) {
      corpus = argv[++i];
    } else if (a == "--mutant" && i + 1 < argc) {
      mutant = argv[++i];
    } else {
      return usage();
    }
  }
  if (corpus.empty()) {
    return usage();
  }
  std::ifstream in(corpus);
  if (!in) {
    std::cerr << "bna_replay: cannot open " << corpus << "\n";
    return 2;
  }
  const auto m = bna::mutant_from_name(mutant);
  if (!m) {
    std::cerr << "bna_replay: unknown mutant " << mutant << "\n";
    return 2;
  }

  std::unique_ptr<bna::Engine<int>> engine;
  int sync = 0;
  std::string line;
  int n = 0;
  try {
    while (std::getline(in, line)) {
      ++n;
      if (line.find('{') == std::string::npos) {
        continue;
      }
      Parser parser(line);
      const Object o = parser.object();
      const std::string ev = str(o, "ev");
      if (ev == "params") {
        if (engine) {
          throw std::runtime_error("second params record");
        }
        auto it = o.find("sync");
        sync = it == o.end() ? 0 : static_cast<int>(it->second.num);
        engine = std::make_unique<bna::Engine<int>>(params(o), sync);
        engine->set_mutant(*m);
        engine->on_record([](const bna::Record & r) {std::cout << bna::to_json(r) << "\n";});
        std::cout << bna::params_json(engine->params(), sync) << "\n";
        continue;
      }
      if (!engine) {
        throw std::runtime_error("the first event must be params");
      }
      if (ev == "admit") {
        const int input = static_cast<int>(num(o, "input"));
        const bna::Key k = key(o);
        std::cout << bna::admit_json(sync, input, num(o, "stamp"), k, tag(o)) << "\n";
        engine->admit(input, num(o, "stamp"), k, tag(o));
      } else if (ev == "missing") {
        const int input = static_cast<int>(num(o, "input"));
        if (o.count("stamp") != 0) {
          std::cout << bna::missing_json(sync, input, key(o), tag(o), num(o, "stamp")) << "\n";
          engine->missing(input, key(o), tag(o), num(o, "stamp"));
        } else {
          std::cout << bna::missing_json(sync, input, key(o), tag(o)) << "\n";
          engine->missing(input, key(o), tag(o));
        }
      } else if (ev == "wake") {
        std::cout << bna::wake_json(sync, tag(o)) << "\n";
        engine->wake(tag(o));
      } else if (ev == "close") {
        std::cout << bna::close_json(sync, str(o, "reason"), tag(o)) << "\n";
        engine->close(str(o, "reason"), tag(o));
      } else {
        throw std::runtime_error("unknown input event " + ev);
      }
    }
  } catch (const std::exception & e) {
    std::cout.flush();
    std::cerr << "bna_replay: line " << n << ": " << e.what() << "\n";
    return 1;
  }
  return 0;
}
