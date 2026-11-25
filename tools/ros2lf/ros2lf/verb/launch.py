# Copyright 2026 rcllf Contributors
# Licensed under the Apache License, Version 2.0

"""Verb: ``ros2 lf launch`` — run a system under LF control."""

import re
import shlex
import subprocess
import sys

from ros2cli.verb import VerbExtension

from ros2lf.api.discovery import (
    find_system,
    find_workspace,
    generated_launch_filename,
    generated_package_name,
    stale_wrapper_artifacts,
)


class LaunchVerb(VerbExtension):
    """Launch an LF-controlled ROS2 system."""

    def add_arguments(self, parser, cli_name):  # noqa: D102
        parser.add_argument(
            "package",
            help="Package name.",
        )
        parser.add_argument(
            "stem",
            help="Launch stem name (e.g., 'bringup').",
        )
        parser.add_argument(
            "--workspace",
            default=None,
            help="Colcon workspace root (default: auto-detect).",
        )
        parser.add_argument(
            "launch_args",
            nargs="*",
            metavar="NAME:=VALUE",
            help=(
                "Launch arguments, as ros2 launch takes them. Only arguments the "
                "generated launch file declares are forwarded; the others were "
                "fixed when colcon lf analyzed the launch file."
            ),
        )

    def main(self, *, args):  # noqa: D102
        workspace = find_workspace(getattr(args, "workspace", None))
        if workspace is None:
            print("Error: Could not find workspace.", file=sys.stderr)
            return 1

        system = find_system(workspace, args.package, args.stem)
        if system is None:
            print(
                f"Error: System '{args.package}/{args.stem}' not found.",
                file=sys.stderr,
            )
            print("Run 'ros2 lf list' to see available systems.")
            return 1

        if not system.has_launch:
            print(
                f"Error: No launch file generated for "
                f"'{args.package}/{args.stem}'.",
                file=sys.stderr,
            )
            print("Run 'colcon lf-generate' first.")
            return 1

        stale = stale_wrapper_artifacts(workspace, args.package, args.stem)
        if stale:
            # Name the leftovers of the removed wrapper path; they
            # are ignored, the launch below uses the lf_generated_* package.
            print(
                "Note: ignoring stale wrapper-layout artifacts from the removed "
                "wrapper path, safe to delete:",
                file=sys.stderr,
            )
            for path in stale:
                print(f"  {path}", file=sys.stderr)

        return self._launch_lfc(workspace, system, args)

    def _launch_lfc(self, workspace, system, args):
        """Launch the colcon-installed lf_generated_<name> package."""
        pkg_name = system.generated_package or generated_package_name(system.package, system.stem)
        launch_filename = system.launch_filename or generated_launch_filename(system.stem)

        if not system.wrapper_built:
            print(
                f"Error: '{pkg_name}' not built for "
                f"'{args.package}/{args.stem}'.",
                file=sys.stderr,
            )
            print(f"Run 'colcon lf-build --package {args.package}' first.")
            return 1

        launch_args = list(getattr(args, "launch_args", None) or [])
        if launch_args:
            error = self._refuse_undeclared(workspace, pkg_name, launch_filename, args, launch_args)
            if error:
                print(error, file=sys.stderr)
                return 1

        source_cmds = ["source /opt/ros/jazzy/setup.bash"]
        main_setup = workspace / "install" / "setup.bash"
        if main_setup.is_file():
            source_cmds.append(f"source {main_setup}")

        source_str = " && ".join(source_cmds)
        cmd = f"{source_str} && ros2 launch {pkg_name} {launch_filename}"
        cmd += "".join(f" {shlex.quote(a)}" for a in launch_args)

        print(f"Launching LF-controlled system: {args.package}/{args.stem}")
        print(f"  Package: {pkg_name}")
        print(f"  Launch file: {launch_filename}")

        # Execute, inheriting stdin/stdout/stderr for interactive use
        return subprocess.call(["bash", "-c", cmd])

    @staticmethod
    def _refuse_undeclared(workspace, pkg_name, launch_filename, args, launch_args):
        """An error text naming the first launch argument the generated file lacks."""
        declared = declared_launch_arguments(workspace, pkg_name, launch_filename)
        for token in launch_args:
            name, sep, _ = token.partition(":=")
            if not sep or not name:
                return f"Error: launch argument {token!r} is not of the form NAME:=VALUE."
            if name not in declared:
                accepts = ", ".join(sorted(declared)) if declared else "(none)"
                return (
                    f"Error: launch argument '{name}' is fixed when colcon lf analyzes the "
                    f"launch file; this system accepts: {accepts}. "
                    f"Re-run: colcon lf --package {args.package} --launch-arg {name}:=VALUE"
                )
        return None


_DECLARE_RE = re.compile(r"""DeclareLaunchArgument\(\s*['"]([^'"]+)['"]""")


def declared_launch_arguments(workspace, pkg_name, launch_filename):
    """Names the generated launch file declares with DeclareLaunchArgument."""
    for path in (
        workspace / "install" / pkg_name / "share" / pkg_name / "launch" / launch_filename,
        workspace / "lf-generated" / pkg_name / "launch" / launch_filename,
    ):
        if path.is_file():
            return set(_DECLARE_RE.findall(path.read_text(encoding="utf-8")))
    return set()
