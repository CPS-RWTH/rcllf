# Copyright 2026 rcllf Contributors
# Licensed under the Apache License, Version 2.0
"""``python3 -m colcon_lf.resolved_model {resolve,check}`` (T8).

- ``resolve``: build and write ``resolved_model.json`` from a ``system.json``.
- ``check``: RCLLF-R003 staleness check — refuse if the model file is
  missing, or if any recorded ``inputs[*].sha256`` no longer matches the
  file on disk.
"""

from __future__ import annotations

import argparse
import hashlib
import sys
from pathlib import Path


def _sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _cmd_resolve(args: argparse.Namespace) -> int:
    from colcon_lf.exec_config.model import ExecConfig
    from colcon_lf.resolved_model.model import to_json
    from colcon_lf.resolved_model.resolve import resolve

    system_json = Path(args.system_json)
    if not system_json.is_file():
        print(f"Error: system.json not found: {system_json}", file=sys.stderr)
        return 1

    workspace = Path(args.workspace) if args.workspace else system_json.parent

    if args.exec_config:
        exec_config_path = Path(args.exec_config)
        if not exec_config_path.is_file():
            print(f"Error: --exec-config file not found: {exec_config_path}", file=sys.stderr)
            return 1
        from colcon_lf.output.serializer import deserialize_system_graph
        graph = deserialize_system_graph(system_json.read_text(encoding="utf-8"))
        exec_config = ExecConfig.load(exec_config_path, graph)
    else:
        exec_config = ExecConfig.empty()

    params_files = [Path(p) for p in (args.params_file or [])]
    for pf in params_files:
        if not pf.is_file():
            print(f"Error: --params-file not found: {pf}", file=sys.stderr)
            return 1

    try:
        model = resolve(
            system_json, params_files=params_files, exec_config=exec_config,
            workspace=workspace,
        )
    except ValueError as exc:
        print(f"Error: {exc}", file=sys.stderr)
        return 1

    output = Path(args.output) if args.output else system_json.parent / "resolved_model.json"
    output.write_text(to_json(model), encoding="utf-8")
    print(f"Wrote {output} (claim_status={model.claim_status}, hash={model.model_hash})")
    return 0


def _cmd_check(args: argparse.Namespace) -> int:
    from colcon_lf.resolved_model.model import from_json

    model_path = Path(args.resolved_model)
    if not model_path.is_file():
        print(
            f"Error: RCLLF-R003 resolved model stale: resolved_model.json "
            f"missing at {model_path}; re-run colcon lf-generate",
            file=sys.stderr,
        )
        return 1

    model = from_json(model_path.read_text(encoding="utf-8"))
    workspace = Path(args.workspace) if args.workspace else model_path.parent

    for entry in model.inputs:
        path = Path(entry["path"])
        if not path.is_absolute():
            path = workspace / path
        if not path.is_file():
            print(
                f"Error: RCLLF-R003 resolved model stale: {entry['role']} {entry['path']} "
                f"no longer exists (recorded sha256:{entry['sha256']}); "
                "re-run colcon lf-generate",
                file=sys.stderr,
            )
            return 1
        from colcon_lf.resolved_model.resolve import input_sha256
        current = input_sha256(path, role=entry["role"])
        if current != entry["sha256"]:
            print(
                f"Error: RCLLF-R003 resolved model stale: {entry['role']} {entry['path']} "
                f"changed since resolution (recorded sha256:{entry['sha256']}, "
                f"now sha256:{current}); re-run colcon lf-generate",
                file=sys.stderr,
            )
            return 1

    print(f"OK: {model_path} is current (claim_status={model.claim_status})")
    return 0


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(prog="python3 -m colcon_lf.resolved_model")
    sub = parser.add_subparsers(dest="command", required=True)

    p_resolve = sub.add_parser("resolve", help="Build resolved_model.json from a system.json")
    p_resolve.add_argument("system_json")
    p_resolve.add_argument("--params-file", action="append", default=[])
    p_resolve.add_argument("--exec-config")
    p_resolve.add_argument("-o", "--output")
    p_resolve.add_argument("--workspace")
    p_resolve.set_defaults(func=_cmd_resolve)

    p_check = sub.add_parser("check", help="RCLLF-R003 staleness check")
    p_check.add_argument("resolved_model")
    p_check.add_argument("--workspace")
    p_check.set_defaults(func=_cmd_check)

    args = parser.parse_args(argv)
    return args.func(args)


if __name__ == "__main__":
    raise SystemExit(main())
