# Copyright 2026 rcllf Contributors
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
"""Verb: ``ros2 lf scan [--json] [--html FILE]``, packages, launch files, nodes and topics of a workspace.

Thin shell. The work lives in ``colcon_lf.scan`` (the verb extends the existing
tools rather than duplicating them).
"""

from ros2cli.verb import VerbExtension

_MISSING = (
    "Error: 'ros2 lf scan' needs the colcon_lf package "
    "(pip install -e tools/colcon_lf).")


class ScanVerb(VerbExtension):
    """List packages, launch files, nodes and topics, and whether each launch file converts."""

    def add_arguments(self, parser, cli_name):  # noqa: D102
        try:
            from colcon_lf.scan import add_scan_arguments
        except ImportError:
            self._available = False
            return
        self._available = True
        add_scan_arguments(parser)

    def main(self, *, args):  # noqa: D102
        if not getattr(self, "_available", False):
            print(_MISSING)
            return 1
        from colcon_lf.scan import scan_command
        return scan_command(args)
