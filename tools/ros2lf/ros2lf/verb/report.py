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
"""Verb: ``ros2 lf report <results...> [--html]``, Markdown or static HTML from an eval dir.

Thin shell. The work lives in ``lf_eval.results_report`` (the verb extends the existing
tools rather than duplicating them).
"""

from ros2cli.verb import VerbExtension

_MISSING = (
    "Error: 'ros2 lf report' needs the lf_eval package "
    "(pip install -e tools/lf_eval).")


class ReportVerb(VerbExtension):
    """Experimental: render an eval dir as Markdown or static HTML.

    The eval dir is verified against its SHA256SUMS first.
    """

    def add_arguments(self, parser, cli_name):  # noqa: D102
        try:
            from lf_eval.results_report import add_report_arguments
        except ImportError:
            self._available = False
            return
        self._available = True
        add_report_arguments(parser)

    def main(self, *, args):  # noqa: D102
        if not getattr(self, "_available", False):
            print(_MISSING)
            return 1
        from lf_eval.results_report import report_command
        return report_command(args)
