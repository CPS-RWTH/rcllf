# Copyright 2026 rcllf Contributors
# Licensed under the Apache License, Version 2.0
"""The 'ros2 lf' command."""

import importlib.util

from ros2cli.command import add_subparsers_on_demand, CommandExtension

# Verbs that run only with an optional package, to the package they import. Without
# that package such a verb is neither listed by `ros2 lf --help` nor accepted.
OPTIONAL_VERBS = {
    "eval": "lf_eval",
    "report": "lf_eval",
}


def _installed(module):
    try:
        return importlib.util.find_spec(module) is not None
    except ValueError:  # a module in sys.modules without __spec__; a broken install raises ImportError
        return False


def unavailable_verbs():
    """The verbs of ``OPTIONAL_VERBS`` whose package is not installed, sorted."""
    return sorted(verb for verb, module in OPTIONAL_VERBS.items() if not _installed(module))


class LfCommand(CommandExtension):
    """Lingua Franca tools for ROS2."""

    def add_arguments(self, parser, cli_name, *, argv=None):  # noqa: D102
        self._subparser = parser
        hidden = unavailable_verbs()
        subparsers = add_subparsers_on_demand(
            parser, cli_name, "_verb", "ros2lf.verb", hide_extensions=hidden or None,
            required=False, argv=argv)
        # hide_extensions only drops a verb from the listing; unregister it as well,
        # so `ros2 lf run` without the benchmark package is an invalid choice, not a broken verb.
        for verb in hidden:
            subparsers.choices.pop(verb, None)

    def main(self, *, parser, args):  # noqa: D102
        if not hasattr(args, "_verb"):
            self._subparser.print_help()
            return 0

        extension = getattr(args, "_verb")
        return extension.main(args=args)
