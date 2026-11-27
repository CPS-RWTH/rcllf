# CLI Reference

## `colcon lf` (All-in-One)

Runs the full pipeline for the workspace or a specific package: analyze, configure,
generate, then build.

```bash
colcon lf                            # All packages
colcon lf --package pubsub_example   # Single package
```

`colcon lf` runs `lf-analyze`, `lf-configure` (scaffold only), `lf-generate` and
`lf-build`. Its analyze step writes only `system.json` (no visualizations, as
`--format json`). The three main steps by hand:

```bash
colcon lf-analyze --package pubsub_example
colcon lf-generate --package pubsub_example
colcon lf-build --package pubsub_example
```

| Option | Default | Description |
|---|---|---|
| `--package` | all packages | Process only this package |
| `--workspace` | auto-detect | Colcon workspace root |
| `--launch-arg NAME:=VALUE` | none | Launch argument value, repeatable; passed to `lf-analyze` |
| `--after-delay` | `0` | Default `after` delay of the connections, passed to `lf-generate` |
| `--fast` | off | Fast mode, passed to `lf-generate` (see [Fast mode and logical timeout](#fast-mode-and-logical-timeout)) |
| `--timeout "<LF time>"` | none | Logical timeout, passed to `lf-generate` |
| `--federate` | off | `per-process`, `per-node` or a `federation.yaml` path, passed to `lf-generate` |
| `--coordination` | `centralized` | `centralized` or `decentralized`, passed to `lf-generate` |

`colcon lf` does not stop at one refused or failed system: it builds every system that
generated and ends with one row per launch file, then exits non-zero if any system was
refused or failed. A launch file that is only a helper (it starts one node for manual
testing, for example) can carry the line `# rcllf: not-a-system` (in XML,
`<!-- rcllf: not-a-system -->`); the analysis then skips it and the summary lists it as
`skipped (not-a-system)`. `--launch-file` still selects a marked file.

`colcon lf` has no `--launch-file` option: it processes every launch file of the
package. To convert one launch file of a package that has several, run the steps
yourself and hand `lf-generate` that one analysis:

```bash
colcon lf-analyze --package my_bringup --launch-file minimal_pubsub.launch.py
colcon lf-generate --system-json lf-analysis/my_bringup/minimal_pubsub/system.json
colcon lf-build --package my_bringup
```

There is one generation path: every run emits the lfc-compilable `.lf` and a colcon
package named `lf_generated_<package>_<system>`, see
[Generated package names](#generated-package-names).

## `colcon lf-analyze`

Extracts ROS 2 system models from launch files. Runs inside a built and sourced
ROS 2 workspace. Produces `system.json` files.

```bash
colcon lf-analyze                                                    # all packages
colcon lf-analyze --package pubsub_example                           # one package
colcon lf-analyze --package pubsub_example --launch-file bringup.launch.py  # one launch file
```

Discovers all launch files (`*.launch.py`, `*_launch.py`, `*.launch.xml` and
`*.launch.yaml`) in every package's
`launch/` directory and its subdirectories and analyzes each one, unless narrowed by
`--package` or `--launch-file`. A package is found by the `<name>` in its
`package.xml`, as colcon names it, wherever its directory sits under `src/`.
`--launch-file` takes a path relative to `launch/`, which must name that file exactly,
or a bare file name that occurs once under `launch/`, counting `launch/` itself and its
subdirectories. A bare name that occurs twice is refused with both paths. A file directly
in `launch/` is then selected as `./<name>`. The output directory is named after the file
without its directory and suffix (`launch/topics/talker_listener_launch.py` writes
`lf-analysis/<package>/talker_listener/`). When two selected launch files would write the
same directory (`launch/left/demo.launch.py` and `launch/right/demo.launch.py`), the
command names both and exits 1 before it writes anything: rename one, or select one with
`--launch-file`.

### Options

| Option | Default | Description |
|--------|---------|-------------|
| `--package` | all packages | ROS 2 package to analyze |
| `--launch-file` | all launch files | Specific launch file name |
| `--workspace` | auto-detect | Colcon workspace root path |
| `--output-dir` | `lf-analysis/{package}/` | Override output location |
| `--format` | `all` | Visualization format: `svg`, `pdf`, `json` (no visualizations besides the launch graph) or `all`. `all` writes SVG only; `pdf` writes PDF. See [Output structure](#output-structure) |
| `--launch-arg NAME:=VALUE` | none | Launch argument value, as `ros2 launch` takes it, repeatable. The analysis is fixed to these values. An argument the launch file does not declare is refused |
| `--viz-only` | off | Skip extraction and regenerate visualizations from existing `system.json` files in the output directory |
| `--launch-only` | off | Run only the launch-file parsing stage, writing `launch_graph.json` per launch file without C++ analysis |

```bash
colcon lf-analyze --package pubsub_example --format json  # no visualizations (the launch graph SVG is still written)
colcon lf-analyze --package pubsub_example --format svg   # SVG, the same as the default
colcon lf-analyze --package pubsub_example --format pdf   # PDF
```

### Output structure

Default output goes to `lf-analysis/{package}/{launch_stem}/`. With the default format,
`lf-analyze` writes:

```text
packages/lf-analysis/pubsub_example/bringup/
  system.json                   # the whole analysis (SystemGraph, see Data Model)
  nodes/
    publisher_node.json         # one file per node
    subscriber_node.json
  launch/
    launch_graph.json           # launch file hierarchy
    launch_graph.svg
  viz/
    comm_graph.svg              # inter-node communication
    launch_graph.svg            # launch file hierarchy
    system_graph.svg            # unified system view
    topology.svg
    nodes/
      publisher_node_internals.svg
      subscriber_node_internals.svg
```

`--format pdf` writes PDF instead of SVG under `viz/`. `--format json` skips `viz/`, but
`launch/launch_graph.svg` is written either way. The launch graph is the one picture that
is not under `--format`.

`colcon lf-configure` writes `config/`, and `colcon lf-generate` writes the other files next to `system.json`:

```text
  config/
    execution.yaml              # your execution policy (sparse, scaffolded once)
    execution.reference.yaml    # every key with its default (rewritten on each run)
  exec_config.json              # the resolved policy the generator read
  resolved_model.json           # see Resolved Model
  generate_status.json          # generated, refused or failed, and why
  lf/                           # the generated .lf program
  lfc_packages.txt              # name of the generated package
  rcllf_trace_manifest.json     # static manifest for trace analysis
```

With `--output-dir`, output uses `{output_dir}/{package}/{launch_stem}/` instead.

The analyzer writes a `COLCON_IGNORE` file into the output root so that colcon does
not crawl its output. It therefore refuses an `--output-dir` that is the workspace,
contains the workspace, or holds a `package.xml` that cannot be parsed, has no
`<name>`, or names a package colcon-lf did not generate. The package name prefixes
`lf_generated_` and `lf_wrapper_` are reserved for colcon-lf. A package with such a
name counts as tool output, so do not give your own packages these prefixes.

### Docker usage

```bash
docker compose --profile lf run --rm analyzer   # analyzes all packages

# Or enter the dev container and run manually
docker compose run --rm dev
cd packages
colcon lf-analyze --package pubsub_example --launch-file bringup.launch.py
```

### Error handling

- **Missing workspace**: provide `--workspace` or run from the workspace directory.
- **Package not found**: no `package.xml` under `src/` names that package. Check the name and the `--workspace`.
- **No launch files found**: check that the package has a `launch/` directory with `*.launch.py` files.
- **Visualization failure**: ensure the `graphviz` system package is installed (provides the `dot` binary).
- **libclang errors**: ensure the workspace is built with `colcon build --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON`.

## `colcon lf-configure`

Scaffolds, validates, and inspects the execution-policy config next to each
`system.json`.

```bash
colcon lf-configure                              # All analyses in the workspace
colcon lf-configure --package pubsub_example      # One package (all its launch stems)
colcon lf-configure --system-json path/to/system.json   # One exact analysis
colcon lf-configure --show                       # Print the resolved effective table
colcon lf-configure --interactive                # Guided prompt loop
```

| Option | Default | Description |
|---|---|---|
| `--system-json` | every `system.json` under `lf-analysis/` | One exact analysis |
| `--package` | all packages | Every analysis under `lf-analysis/<package>/` |
| `--workspace` | auto-detect | Colcon workspace root |
| `--show` | off | Print the resolved effective policy table (after merging, before environment overrides) and write nothing |
| `--interactive` | off | Prompt for policy values (defaults, nodes, subscriptions) and write the sparse `execution.yaml` |

`--package` and `--system-json` are mutually exclusive. See
[Execution Configuration](../features/execution-config.md) for the config file format
and every policy key.

## `colcon lf-generate`

Generates a real Lingua Franca (`.lf`) program and its C++ scaffolding from `system.json`.

```bash
colcon lf-generate                                    # All packages
colcon lf-generate --package pubsub_example            # One package (all its launch stems)
colcon lf-generate --system-json path/to/system.json   # One exact analysis
colcon lf-generate --after-delay "5 msec"              # Custom connection delay
colcon lf-generate --federate per-node                 # Split into federates
colcon lf-generate --fast --timeout "20 sec"            # Fast mode, ends at logical 20 s
```

`--package` and `--system-json` are mutually exclusive. `--package <name>` resolves to
every analysis under `lf-analysis/<name>/` (one per launch file `lf-analyze` processed)
and prints which launch stems it matched before generating. Use `--system-json` when
you want exactly one of them.

### Options

| Option | Default | Description |
|---|---|---|
| `--system-json` | every `system.json` under `lf-analysis/` | One exact analysis |
| `--package` | all packages | Every analysis under `lf-analysis/<package>/` |
| `--output-dir` | `lf/` next to `system.json` | Output directory for the generated `.lf` files |
| `--workspace` | auto-detect | Colcon workspace root |
| `--after-delay` | `0` | Default `after` delay of the inter-node connections, for example `"5 msec"`. Overrides `defaults.after_delay` of `execution.yaml`; a per-link `after_delay_ns` in `system.json` still wins |
| `--launch` / `--no-launch` | on | Generate (or skip) the launch files and wrappers |
| `--federate` | off | `per-process`, `per-node` or a `federation.yaml` path. See [Federation](../features/federation.md) |
| `--coordination` | `centralized` | `centralized` or `decentralized`, only with `--federate`. Without it, a `federation.yaml`'s own `coordination:` key applies |
| `--emit-federation-template` | off | Write a starter `federation.yaml` next to each `system.json` and generate no code |
| `--force` | off | Let `--emit-federation-template` overwrite an existing file |
| `--exec-config` | the analysis dir's `config/execution.yaml`, if present | An [execution config](../features/execution-config.md). A malformed file is an error |
| `--fast`, `--timeout` | off | See [Fast mode and logical timeout](#fast-mode-and-logical-timeout) |
| `--params-file` | none | A ROS 2 parameter file applied at generate time, repeatable (later files win). Feeds the RCLLF-R001/R002/R003 gate |

`colcon lf-generate --help` prints the same list.

### Output

For each system, generates a self-contained colcon package at
`lf-generated/lf_generated_<package>_<system>/`, see
[Generated package names](#generated-package-names):

| Output | Description |
|--------|-------------|
| `lf/<Main>.lf` | Lingua Franca source, one reactor per ROS 2 node class; `<Main>` is the launch file stem in CamelCase |
| `lf/include/` | Header with the reaction ID macros |
| `src/ros2_lf_startup.cpp` | Pre-`main()` init (constructor attribute) |
| `src/ros2_lf_reaction_ids_init.cpp` | Populates the reaction ID table at startup |
| `src/node_factories.cpp`, `src/nodes/` | Factories that create your node classes inside the program |
| `launch/lf_<system>.launch.py` | Launch file that starts the program |
| `system.json`, `exec_config.json`, `correspondence.json` | Copies of the analysis and the settings the program reads at start-up |
| `CMakeLists.txt`, `package.xml` | Invokes `lfc`, links `ros2_lf_binding` |

### Generated package names

The generated package is named `lf_generated_<package>_<system>`. `<system>` is the launch
file stem with the underscores removed, in lower case: `bringup.launch.py` gives
`bringup`, `long_names.launch.py` gives `longnames`. When that name equals the name of one
of the system's node classes, `system` is appended: `publisher_node.launch.py` gives
`publishernodesystem`.

`ros2 lf list` prints the launch stem, not the package name. The exact package names are the
directories under `lf-generated/` and the first word of each `lfc_packages.txt`. For
`pubsub_example` they are:

```text
lf_generated_pubsub_example_bringup
lf_generated_pubsub_example_longnames
lf_generated_pubsub_example_publishernodesystem
lf_generated_pubsub_example_subscribernodesystem
```

A federated build adds one package per federate with the suffix `_<federate>`.

See [Architecture](../design/architecture.md) for the full generation and build pipeline, and
[Federation](../features/federation.md) for `--federate`.

`--target-lfc` is still accepted for one release and does nothing except print a
deprecation note: the lfc output is the only output.

### Fast mode and logical timeout

| Option | Emits | Effect |
|---|---|---|
| `--fast` | `-f true` in the generated launch files | Logical time advances as fast as the reactions allow; the runtime never waits for physical time. Not the `fast` target property: lfc refuses it in programs with physical actions, which every rcllf program has. |
| `--timeout "<LF time>"` | `timeout: <LF time>` in the target block | The program ends at that logical time (`20 sec`, `1500 msec`; any LF time literal is accepted and canonicalised). |

Both also work on `colcon lf`. A built program takes the same two settings at launch
time without regeneration: the generated launch files (the one program and every
federate) append the words of `RCLLF_LF_ARGS` to the program's arguments, after the
generated ones.

```bash
RCLLF_LF_ARGS="-f true -o 20 sec" ros2 launch lf_generated_pubsub_example_bringup lf_bringup.launch.py
```

`-f true` is reactor-c's fast mode and `-o 20 sec` its timeout (value and unit as
two words); `-f false` turns a `--fast` program back to real time. See
[Fast Mode](../features/fast-mode.md) for what fast mode does and does not change.

## `colcon lf-build`

Builds the generated packages: `colcon build --packages-select` over every
`lf_generated_<package>_<system>` package (and its per-federate packages) generated from
the analyzed systems, into the workspace's own install space. The package's
`CMakeLists.txt` invokes `lfc` as a build step and links the result against
`ros2_lf_binding`.

```bash
colcon lf-build                              # Every generated package
colcon lf-build --package pubsub_example     # Only those generated from one package
```

| Option | Default | Description |
|---|---|---|
| `--package` | all packages | Only the packages generated from this source package |
| `--workspace` | auto-detect | Colcon workspace root |

To build one generated package by hand, give `colcon build` its exact name from
[Generated package names](#generated-package-names):

```bash
colcon build --packages-select lf_generated_pubsub_example_bringup
```

## `ros2 lf list`

Discovers and lists all LF-analyzed systems with their status.

```bash
ros2 lf list                             # All systems
ros2 lf list --package basic_components  # Filter by package
```

| Option | Default | Description |
|---|---|---|
| `--workspace` | auto-detect | Colcon workspace root |
| `--package` | all packages | Only systems of this package |

The columns are the package, the launch stem, the number of nodes, the node type
(`nodes`, `components`, `mixed` or `unknown`) and `Launch Ready`:

| `Launch Ready` | Meaning |
|---|---|
| `ready`, `ready (federated)` | generated and built; launch it with `ros2 lf launch` |
| `refused (RCLLF-...)` | generation stopped on that code, see [Diagnostic codes](diagnostic-codes.md) |
| `failed (see: colcon lf-generate)` | generation failed; run it again to read the error |
| `- (run: colcon lf-generate)` | analyzed but never generated |
| `- (run: colcon lf-build --package <pkg>)` | generated but not built yet |

## `ros2 lf launch`

Launches a generated system: finds the installed generated package for
`<package> <stem>` (see [Generated package names](#generated-package-names)), sources the
workspace and runs its launch file.

```bash
ros2 lf launch pubsub_example bringup
ros2 lf launch basic_components combo_pipeline
```

| Argument | Description |
|---|---|
| `package`, `stem` | Package name and launch stem, as `ros2 lf list` prints them |
| `NAME:=VALUE` | Launch arguments, as `ros2 launch` takes them. Only arguments the generated launch file declares are forwarded; the others were fixed when `lf-analyze` read the launch file |
| `--workspace` | Colcon workspace root (default: auto-detect) |


## `ros2 lf eval`

Experimental. Evaluates one or more results folders (the `rcllf.result/1` layout: `result.json`,
`run_set.json`, `runs/`, `build/` and a `SHA256SUMS` manifest) with
`lf_eval`'s metric registry and writes `<first folder>/eval/<UTC>/`. Such folders
are the input of this command. It works in four steps.

1. **Verify (step 0).** Before anything else is read, every file of each
   folder is checked against the folder's `SHA256SUMS`. A listed file that is
   missing, a file whose bytes changed, a file `SHA256SUMS` does not list, or a
   symlink outside `runs/*/logs/` rejects the folder: the command names every
   offending path (`changed: runs/<run>/outputs.jsonl`,
   `unlisted: runs/<run>/extra.txt`), evaluates nothing and exits 4. The
   `eval/` subfolder is outside the checked scope only when it is a real
   directory: an `eval` symlink or file rejects the folder, because the eval
   dir is written there. `--allow-missing GLOB` waives missing files that
   match the glob, for folders whose traces were trimmed after recording;
   every waived path is listed in `metrics.json`. Changed or unlisted files
   are never waived, and neither is a missing `result.json`.

   Every later read is bound to the verified bytes: `result.json`,
   `run.json`, `system.json` and the `*.jsonl` inputs are read once, their
   sha256 compared with `SHA256SUMS`, and parsed from those bytes, so a file
   replaced after step 0 is a rejection (exit 4, `changed after step 0`),
   never a metric attributed to the verified hash. A run's trace is read by
   path (babeltrace), so every verified file of that run dir is hashed again
   right before and right after the trace load. The remaining window: a file
   swapped and swapped back between those two checks is not detected.
2. **Load.** `result.json` gives the runs, grouped by (variant, sweep point). A `run_id`
   or `build_ref` that points outside `runs/` or `build/` is a rejection (exit 4).
   Runs that `run_set.json` excluded are listed in each group and left out of
   the metrics unless `--include-excluded` is given.
3. **Compute per group.** Each metric runs once per group with that group's
   own `build/<variant>-<point>/system.json`. A metric declares what it reads
   (`trace`, `outputs`, `arrivals`, `cpu`, `bag`, `graph_model`, `lf_model`).
   When a run lacks one, the metric is `unavailable` with the reason, and a
   metric that needs the LF model is `not_applicable` on a stock group. A
   metric gets options only when it declares them: `--option
   output_count.horizon_ns=1000000000` works, and an option for a metric that
   declares none is an error (exit 2). `output_count`'s horizon defaults to
   the run's `run.json` horizon. Group keys read `[folder:]variant|point`;
   two groups whose keys (or plot dir names) would coincide, such as
   variant `a|b` at `p0` and variant `a` at `b|p0`, are an error (exit 2).
   A cross-group metric is called once over all groups with no graph model,
   so one that declares `graph_model` or `lf_model` is `not_applicable`.
4. **Compare.** `--baseline VARIANT` compares every other variant with the
   baseline at the same sweep point (from the candidate's own folder; from
   another folder only when exactly one has it), through each metric that implements
   `compare()`, with `delta = candidate - baseline`. A comparison between an
   rcllf and a stock group is refused unless both declare a horizon.

The eval dir holds `metrics.json` (`rcllf.metrics/1`: the hash of each raw
`SHA256SUMS` that step 0 verified, per group and metric a `status` of `ok`,
`unavailable`, `not_applicable` or `error`, the metric's data and summary, and
the comparison rows), `report.md`, `report.html` (as `ros2 lf report --html`
renders it), `plots/<group>/` and its own `SHA256SUMS`.
That manifest also lists the raw `SHA256SUMS` by its relative path, so
`sha256sum -c SHA256SUMS` run inside the eval dir checks both the eval and
the folder manifest it was computed from.

```bash
ros2 lf eval <folder> --baseline lfc
ros2 lf eval <folder> --metrics output_count,e2e_latency
ros2 lf eval <folder> --allow-missing 'runs/*/tracing/*'   # traces trimmed after the run
```

| Flag | Meaning |
|---|---|
| `results` | one or more results folders |
| `--baseline VARIANT` | compare the other variants with this one, per sweep point |
| `--metrics A,B` | metrics to compute (default: all; `lf-eval list-metrics`) |
| `--option METRIC.KEY=VALUE` | option for a metric that declares it, repeatable |
| `--include-excluded` | also feed runs that `run_set.json` excluded to the metrics |
| `--allow-missing GLOB` | step 0: waive missing files matching GLOB, repeatable |
| `--chain A->B->C` | explicit chain, repeatable (default: derived from the group's `system.json`) |
| `--backend {bt2,cli}` | force a trace reader backend |
| `-o, --out DIR` | eval dir (default: `<first folder>/eval/<UTC>/`); inside a results folder only under its `eval/` |
| `--no-plots` | skip plots |

Single run dirs are not results folders; `lf-eval analyze` and `lf-eval
report` still take those. `lf-eval eval` is the same command without ros2cli.
There is no MLflow export.

## `ros2 lf report`

Experimental. Renders an eval dir written by `ros2 lf eval` as Markdown (default) or, with
`--html`, as one static HTML page. `RESULTS` is either an eval dir or a results
folder, whose newest `eval/<UTC>/` holding a `metrics.json` is used. Several
arguments give one page with a section each.

Before rendering, the eval dir is checked against its own `SHA256SUMS`, the
same way `ros2 lf eval` checks a results folder: a missing, changed, unlisted
or symlinked file, a symlinked eval dir, a changed raw `SHA256SUMS` (the
`../../SHA256SUMS` line), or an input named in `metrics.json` whose raw
`SHA256SUMS` no line pins, names the path, renders nothing and exits 4. The page
is built from the bytes that were hashed; no file is read a second time.

The HTML page is self-contained: CSS inline, no script, no external fetch.
Metric plots (PNG or SVG files inside the eval dir) are embedded as data URIs,
and each comparison table gets an inline SVG bar chart of `delta = candidate -
baseline`. Every string from the data is HTML-escaped. Every number is shown
in full precision (`repr` of the float, so a value reads back to the same
float) and carries the RFC 6901 JSON pointer of its place in `metrics.json` in
a `data-path` attribute, for example
`data-path="/compare/0/rows/0/delta"`. A test parses the page and compares
each tagged value with `metrics.json`. A metric's `data` block shows at most
400 values; past that it says the rest is in `metrics.json`.

```bash
ros2 lf report <eval-dir>                         # Markdown to stdout
ros2 lf report --html <eval-dir> -o report.html
```

| Flag | Meaning |
|---|---|
| `results` | one or more eval dirs or results folders |
| `--html` | static self-contained HTML instead of Markdown |
| `-o, --out FILE` | write to FILE (default stdout); a path inside the eval dir or inside a results folder it pins is refused (exit 2), since a `SHA256SUMS` seals both |

`ros2 lf eval` already writes both reports into the eval dir it seals;
`ros2 lf report` re-renders them, or writes them elsewhere, from a verified
eval dir.


## `ros2 lf scan`

Answers "can `colcon lf` convert this workspace?" without writing anything.
For each launch file of each package under `src/` it builds the same system
graph `colcon lf-analyze` builds (in memory) and lists:

- the nodes the launch file starts (package, executable, name, namespace),
  following includes and groups;
- the analysed nodes with the topics each publishes and subscribes and its
  timer count;
- the topics with message type, publishers and subscribers, plus services and
  actions;
- the analyser's diagnostics with their severity, launch-file errors, and
  nodes that could not be analysed;
- a verdict `convertible`: `no` when analysis failed (skipped nodes,
  unresolved includes, libclang errors, an error-severity diagnostic), `warn`
  when it succeeded with warnings, launch errors or unanalysed nodes, `yes`
  otherwise, with the reasons.

The default output is a text summary. `--json` prints the `rcllf.scan/1`
document, `-o FILE` also writes it, and `--html FILE` writes a static
self-contained page (inline CSS, no script, every string escaped). The C++
parse needs the workspace's message headers, so run it in the dev container
after `colcon build`.

```bash
ros2 lf scan                                       # ./packages, all packages
ros2 lf scan --package pubsub_example --json
ros2 lf scan --package pubsub_example --launch-file bringup.launch.py --html scan.html
```

| Flag | Meaning |
|---|---|
| `--workspace DIR` | colcon workspace root holding `src/` (default: `./packages`, `.` or `..`) |
| `--package NAME` | scan only this package (unknown name: exit 2) |
| `--launch-file NAME` | only this launch file of `--package`, matched as for `colcon lf-analyze` (exit 2 without `--package`) |
| `--json` | print the JSON instead of the text summary |
| `-o, --out FILE` | also write the JSON to FILE |
| `--html FILE` | also write the HTML page |

## Docker Compose services

The Compose services that run these commands (`lf-build`, `analyzer` and the others) are
listed in [Docker](docker.md#docker-compose-services). See [Architecture](../design/architecture.md)
for what the pipeline does step by step.
