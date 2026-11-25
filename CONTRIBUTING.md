# Contributing to rcllf

Thank you for your interest in rcllf. This page says where issues and pull requests go,
how to make and test a change, and what review looks like. House style comes last: CI
and the reviewer point out style problems, so they need not block a first contribution.

## How this repository works

This repository is release-only. Each release is one commit plus a tag, exported from the
maintainers' development repository, where day-to-day development happens. The
`CHANGELOG.md` lists what each release contains.

- **Issues** go to this repository's issue tracker: bug reports, feature requests and
  questions. For a security problem, do not open an issue, see `SECURITY.md`.
- **Issue numbers** in this repository are its own. They are unrelated to the numbers in
  the development repository, so a `#N` in an issue or pull request here always means an
  item of this repository.

### Contribution route

Issues and pull requests are welcome on this public repository. A maintainer lands an
accepted pull request in the development repository, keeping the author credit with a
`Co-authored-by` trailer. The next export supersedes the public pull request, and the
public pull request is closed with a link to the release that carries it. Nothing is
merged directly on the public repository, because the next export would overwrite it.

## Making a change

**Small fixes** (a typo, a broken link, a wrong command in the docs, an obvious bug with
an obvious fix) go straight to a pull request, without an issue. **Larger changes** (a
new feature, a change to what the generator emits, a change to a documented guarantee)
start with an issue, so that the approach is agreed before you spend time on it.

1. **Set up.** Fork, clone with the submodule, and open the dev container. It has ROS 2
   Jazzy, the Lingua Franca compiler, `libclang` and the tools preinstalled. By default its
   first start builds only the runtime and the example packages. The test and lint commands
   below need every package built, so start it with `ENTRYPOINT_PACKAGES=all`:

    ```bash
    git clone https://github.com/<you>/rcllf.git && cd rcllf
    git submodule update --init third_party/reactor-c
    docker compose build dev
    docker compose run --rm -e ENTRYPOINT_PACKAGES=all dev
    ```

2. **Change the code** on a branch from `main`. Where the code lives:
   [Code Structure](docs/reference/code-structure.md), and for the extraction and
   generation CLI, [`tools/colcon_lf/README.md`](tools/colcon_lf/README.md).
3. **Test what you changed** (next section). If your change alters what a generated
   program does, run an example and read its output, not only the tests.
4. **Open a pull request** that says what changed, why, and how you tested it (the
   commands and what they printed). Name the issue it addresses, if any (`Closes #N`).
5. **Review.** CI runs on the pull request, and a maintainer reviews it and may ask for
   changes. An accepted pull request then follows the
   [contribution route](#contribution-route) above.

## Testing

Run the suites that cover the code you touched, inside the dev container
(`docker compose run --rm dev`), after the full build of step 1. In a container that built
only the examples, `colcon test` and `scripts/lint.sh` fail with "Has this package been
built before?": run `cd packages && colcon build --symlink-install` once. The entrypoint
installs the tools editable, so no `pip install` step is needed there.

| Suite | Command, from the repository root | Covers |
|---|---|---|
| Extraction & generation | `cd tools/colcon_lf && pytest tests/ -q` | `colcon lf-*` verbs, analysis, generated LF |
| Trace evaluation | `cd tools/lf_eval && pytest tests/ -q` | `lf-eval` metrics and reports |
| `ros2 lf` CLI | `cd tools/ros2lf && pytest test/ -q` | `ros2 lf` verbs |
| ROS 2 packages + linters | `cd packages && colcon test && colcon test-result --verbose` | runtime binding, examples, ament lint |
| Docs | `mkdocs build --strict` | the documentation site |

The `colcon_lf` suite is large: run the files that match your change first
(`pytest tests/generation/ -q`, `pytest tests/ -q -k launch`), and the whole suite before
you ask for review. Tests that need something missing (ROS headers, a built example,
Graphviz) skip and say why, and `pytest -q -rs` lists the reasons.

To check a generator or runtime change end to end:

```bash
cd packages && source install/setup.bash
colcon lf --package pubsub_example && source install/setup.bash
ros2 lf launch pubsub_example bringup
```

The [Quickstart](docs/getting-started/index.md#what-success-looks-like) shows the output
to expect.

### Running on the host

`lf_eval` has no ROS 2 dependency and runs on the host:
`cd tools/lf_eval && pip install -e ".[test,stats]" && pytest tests/ -q`.
`colcon_lf` mostly runs on the host too (`pip install -e ".[test]"`), except
`tests/test_clang_parser.py`, which parses the example C++ against the ROS 2 headers and
fails without them. Use the host for a fast inner loop and the container before review.

### What CI runs

`.github/workflows/ci.yml` runs on every push and pull request: `colcon test` with the
ament linters, the `colcon_lf` suite in parallel shards, the `lf_eval` and `ros2lf`
suites, runtime smoke runs that build and launch the examples, and the docs build. Each
step runs a script under `scripts/ci/`, and the job's log shows the command: run it in the
dev container to reproduce a failure.

The jobs run inside a CI image keyed by the image inputs (`docker/Dockerfile`,
`docker/ros-apt/`, `requirements-docs.txt`, `third_party/`). When no image for those
inputs exists yet, the `image` job builds it and pushes it to the project's container
registry. A pull request from a fork cannot push there, so a fork pull request that
changes an image input fails at the `image` job. Say so in the pull request; a
maintainer then runs it from a branch of this repository.

## Extending the extractor

`tools/colcon_lf/README.md` maps the modules. The usual extension points:

- **A new C++ API pattern** (a call the extractor should recognize): detection in
  `node_analysis/clang_parser.py`, fields in `node_analysis/models.py`, an example
  package under `packages/src/` that uses it, and assertions in
  `tests/test_clang_parser.py`.
- **A new launch-file pattern**: handling in `launch_analysis/parser.py`
  (`_walk_entities()`), fields in `launch_analysis/models.py` if needed, a launch file
  that uses it, and assertions in `tests/test_launch_parser.py`.
- **A new visualization**: a module under `visualization/` returning a
  `graphviz.Digraph`, wired into `verb/analyze.py` (`_generate_visualizations()`), with
  DOT-source assertions in `tests/test_visualizations.py`.

`parse_node()` and `parse_launch()` cache results per resolved file path. After editing
a parsed source during a session, call `clear_parse_node_cache()` or
`clear_parse_launch_cache()`.

## House style

CI checks most of this, and the reviewer points out the rest.

### Code

The `ament_lint_common` linters run as part of `colcon test`: `ament_cpplint` (Google
C++ style), `ament_uncrustify` (formatting), `ament_cppcheck`, `ament_copyright`,
`ament_lint_cmake`, `ament_xmllint`, `ament_flake8` and `ament_pep257`. To fix and check
formatting:

```bash
scripts/format.sh              # auto-fix (ament_uncrustify)
scripts/lint.sh                # check
scripts/lint.sh timer_example  # check one package
```

`docker compose run --rm lint` runs the same check. A new package picks up the linters
through `ament_lint_auto` and `ament_lint_common` test dependencies in `package.xml` and
the usual `if(BUILD_TESTING) ... ament_lint_auto_find_test_dependencies()` block in
`CMakeLists.txt`.

A few files grew large during the prototype phase: `ros2_lf_binding/src/corr_runtime.cpp`,
`colcon_lf/generation/lf_generator.py` and the biggest binding tests. Splitting them is
planned. Until then, keep new code out of them where a new file fits, and keep changes to
them small so they stay reviewable.

### Copyright headers

Every C/C++, Python, CMake, YAML and shell file starts (after a `#!` line) with:

```python
# Copyright 2026 rcllf Contributors
# Licensed under the Apache License, Version 2.0
```

C and C++ files use the same text with `//` comments.

### Commits

Conventional commits, `type(scope): summary`, for example
`fix(colcon-lf): keep remapped topic names in system.json`. Types: `feat`, `fix`,
`docs`, `refactor`, `test`, `perf`, `ci`, `chore`. A maintainer may reword commits when
landing the change.

### Documentation

Changes to user-facing behavior update the pages under `docs/` that describe it, and the
site must build with `mkdocs build --strict`.

`docs/features/support-matrix.md` is generated from
`tools/colcon_lf/colcon_lf/features/` (`registry.py`, `evidence.py`, `limits.py`). Do not
edit it by hand: regenerate it with `python3 -m colcon_lf.features.docs_table` from
`tools/colcon_lf/` and commit the result. CI checks it for drift.

The docs follow a terse house style:

- Short declarative sentences, no marketing adjectives, no "Overview" section that
  restates the title. Second person imperative for instructions, present tense for
  system behavior.
- No em or en dashes in prose (CI runs `python3 scripts/check_punctuation.py`), and
  preferably no semicolons. A colon introduces a list or a code block.
- Each page opens with its H1 and a one or two sentence scope statement.
- Each concept has one canonical page. Elsewhere, link to it instead of re-explaining.
- Verify every documented command and flag against the source, not against another
  docs page.
- Admonitions are for hazards and version caveats only.

## Questions?

Open a blank issue in this repository and start its title with "Question:". For
security issues, see `SECURITY.md`.
