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

#ifndef RCLLF_PLANT__HASH_HPP_
#define RCLLF_PLANT__HASH_HPP_

#include <cstdint>
#include <cstring>
#include <vector>

namespace rcllf_plant
{

// Canonical hash: 64-bit FNV-1a over the IEEE-754 bit
// pattern of every double, each serialized little-endian, in the caller's
// (Descriptor) order, nothing rounded. Independent of the host byte order, so
// a hash from an x86_64 run and an aarch64 run of equal values is equal.
class CanonicalHash
{
public:
  static constexpr uint64_t kOffset = 0xcbf29ce484222325ULL;
  static constexpr uint64_t kPrime = 0x100000001b3ULL;

  void add(double value)
  {
    uint64_t bits = 0;
    std::memcpy(&bits, &value, sizeof(bits));
    for (int i = 0; i < 8; ++i) {
      hash_ ^= (bits >> (8 * i)) & 0xffU;
      hash_ *= kPrime;
    }
  }

  void add(const std::vector<double> & values)
  {
    for (const double v : values) {
      add(v);
    }
  }

  uint64_t value() const {return hash_;}

private:
  uint64_t hash_{kOffset};
};

inline uint64_t canonical_hash(const std::vector<double> & values)
{
  CanonicalHash h;
  h.add(values);
  return h.value();
}

}  // namespace rcllf_plant

#endif  // RCLLF_PLANT__HASH_HPP_
