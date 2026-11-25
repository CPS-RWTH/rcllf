<!--
Copyright 2026 rcllf Contributors

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
-->

# rcllf_plant

The plant transactor of rcllf and its simulator backends (MuJoCo, the F1TENTH
Gym worker, and an optional Gazebo backend).

## MuJoCo download at configure time

Configuring this package downloads a pinned MuJoCo release (version in
`CMakeLists.txt`) from the MuJoCo GitHub releases page. The archive is
verified against a SHA-256 hash that is pinned in `CMakeLists.txt`, and the
configure step fails on a mismatch. The package then installs `libmujoco.so`
next to its own libraries.

To build without network access, unpack the same MuJoCo release yourself and
pass `-DMUJOCO_ROOT=<dir>`. The download is skipped then.

MuJoCo is licensed under the Apache License 2.0. See the repository `NOTICE`
file for the third-party notices.
