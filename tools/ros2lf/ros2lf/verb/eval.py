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
"""Verb: ``ros2 lf eval <results...>``, metrics over results folders.

Thin shell. Step 0 verification, per-group metrics, ``--baseline`` comparison
and the eval dir live in ``lf_eval.results_eval`` (the verb extends lf_eval rather than
duplicating it).
"""

from ros2cli.verb import VerbExtension

_MISSING = (
    "Error: 'ros2 lf eval' needs the lf_eval package "
    "(pip install -e tools/lf_eval).")


class EvalVerb(VerbExtension):
    """Experimental: metrics over results folders, compared against a baseline variant.

    Verifies results folders against SHA256SUMS, computes every metric per
    (variant, sweep point) group and compares against a baseline variant.
    """

    def add_arguments(self, parser, cli_name):  # noqa: D102
        try:
            from lf_eval.results_eval import add_eval_arguments
        except ImportError:
            self._available = False
            return
        self._available = True
        add_eval_arguments(parser)

    def main(self, *, args):  # noqa: D102
        if not getattr(self, "_available", False):
            print(_MISSING)
            return 1
        import logging

        from lf_eval.results_eval import eval_command
        logging.basicConfig(level=logging.WARNING, format="%(levelname)s %(name)s: %(message)s")
        return eval_command(args)
