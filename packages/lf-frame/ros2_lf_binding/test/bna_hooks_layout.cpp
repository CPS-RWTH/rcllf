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


// The rcllf_message_filters side of the BNA hook structs, compiled alone so
// test_bna_runtime can compare its layout with the binding's.

#include <cstddef>

#include "../../rcllf_message_filters/include/rcllf_message_filters/bna_runtime_hooks.h"

extern "C" size_t rcllf_bna_policy_layout(int i)
{
  const size_t v[] = {
    sizeof(rcllf_bna_hooks), offsetof(rcllf_bna_hooks, missing),
    offsetof(rcllf_bna_hooks, wake), offsetof(rcllf_bna_hooks, close),
    sizeof(rcllf_bna_dispatch), offsetof(rcllf_bna_dispatch, micro),
    offsetof(rcllf_bna_dispatch, epoch), offsetof(rcllf_bna_dispatch, ordinal),
    offsetof(rcllf_bna_dispatch, gid), RCLLF_BNA_OK - RCLLF_BNA_NOT_CONSTRUCTING * 10 -
    RCLLF_BNA_MULTIPLE * 100 - RCLLF_BNA_NOT_DRAIN * 1000 - RCLLF_BNA_WRONG_INPUT * 10000};
  return v[i];
}
