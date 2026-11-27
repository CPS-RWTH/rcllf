# Environment Variables

This page lists every environment variable that the rcllf runtime, the `colcon lf`
tools, the generated launch files and the Docker Compose services read. Each row says
who reads it, its default, what it changes, and which page explains it. Link to a row
with its name in lower case, for example `environment-variables.md#ros2_lf_take_mode`.

## Runtime

The runtime binding reads these once when a generated program starts. Set them in the
shell that runs `ros2 lf launch` or `ros2 launch`.

| Name | Read by | Default | Effect | Explained in |
|---|---|---|---|---|
| <a id="ros2_lf_take_mode"></a>`ROS2_LF_TAKE_MODE` | runtime | unset | `skip` or `block`. Overrides the resolved `take_mode` of every subscription that reads from inside the system for this run, and logs one `WARN` that counts the entries it changed. | [execution.yaml Reference](../features/execution-config.md#env-var-overrides) |
| <a id="ros2_lf_maxwait_ms"></a>`ROS2_LF_MAXWAIT_MS` | runtime | unset | Overrides the resolved `maxwait_ms` of every subscription for this run. It does not change the timeout of an admitted edge, which is fixed at generation. | [execution.yaml Reference](../features/execution-config.md#env-var-overrides) |
| <a id="ros2_lf_take_timeout_ms"></a>`ROS2_LF_TAKE_TIMEOUT_MS` | runtime | unset | Old name of `ROS2_LF_MAXWAIT_MS`. Both set to different values: the override is refused with a `WARN`. | [execution.yaml Reference](../features/execution-config.md#env-var-overrides) |
| <a id="ros2_lf_exec_config"></a>`ROS2_LF_EXEC_CONFIG` | runtime | the generated `exec_config.json` | Path of the resolved policy table. The node parameter `exec_config` wins over it. The generated launch files set it. | [execution.yaml Reference](../features/execution-config.md#binding-keys-and-exec_configjson) |
| <a id="ros2_lf_discovery_timeout_ms"></a>`ROS2_LF_DISCOVERY_TIMEOUT_MS` | runtime | `5000` | How long startup waits for every subscription to match a publisher, and how long a federate waits for its peers. `0` or less skips the wait. | [Federation](../features/federation.md#troubleshooting) |
| <a id="ros2_lf_match_warmup_ms"></a>`ROS2_LF_MATCH_WARMUP_MS` | runtime, federates | `ROS2_LF_DISCOVERY_TIMEOUT_MS` | Bound of the endpoint match warm-up before the first tag. A federate still unmatched at the bound exits 3. `0` checks once and does not wait. | [Federation](../features/federation.md#endpoint-match-warm-up) |
| <a id="ros2_lf_xfed_clock_bound_ns"></a>`ROS2_LF_XFED_CLOCK_BOUND_NS` | runtime, federates | unset | The largest offset between the clocks of two hosts, in decimal nanoseconds, as measured by chrony or PTP. Needed when an admitted edge crosses hosts, otherwise the receiver refuses the other host's records. Enters every cutoff as eps. | [Federation](../features/federation.md#several-hosts) |
| <a id="ros2_lf_xfed_clock_synchronized"></a>`ROS2_LF_XFED_CLOCK_SYNCHRONIZED` | runtime, federates | unset | `1` declares that the hosts share one clock, the same as a bound of 0. | [Federation](../features/federation.md#several-hosts) |
| <a id="ros2_lf_clock_publish"></a>`ROS2_LF_CLOCK_PUBLISH` | runtime | `on_change` | Publishes the logical time on `/clock` and `/lf_clock` for tools to watch: `off`, `on_change` or `every_reaction`. An unknown value logs a warning and uses `on_change`. Nodes never read their time from it. | [ADR 0001](../design/adr/0001-lf-owns-time.md) |
| <a id="rcllf_trace_callbacks"></a>`RCLLF_TRACE_CALLBACKS` | runtime | unset | `1` prints one JSON line per callback, dispatch and missing message. | [Trace schema](../reference/trace-schema.md) |
| <a id="rcllf_stop_timeout_ms"></a>`RCLLF_STOP_TIMEOUT_MS` | runtime | `3000` | After a stop signal, how long the process may go without progress before it exits with code 124. `0` turns the bound off. | [Federation](../features/federation.md#stopping-a-federation) |
| <a id="rcllf_stop_max_ms"></a>`RCLLF_STOP_MAX_MS` | runtime | four times `RCLLF_STOP_TIMEOUT_MS` | Cap on the whole stop, counted from the signal. Past it the process exits with code 124. | [Federation](../features/federation.md#stopping-a-federation) |
| <a id="rcllf_startup_timeout_ms"></a>`RCLLF_STARTUP_TIMEOUT_MS` | runtime | `30000` | Bound on startup up to the discovery wait. Past it the process exits with code 124. `0` turns it off. | [Federation](../features/federation.md#exit-codes) |
| <a id="rcllf_exit_with_parent"></a>`RCLLF_EXIT_WITH_PARENT` | runtime, RTI guard | set by the generated launch files | Process id of the launcher. The process ends when the launcher dies. | [Federation](../features/federation.md#stopping-a-federation) |
| <a id="rcllf_admitted_reader_reliable"></a>`RCLLF_ADMITTED_READER_RELIABLE` | runtime | unset | By default the reader of an admitted edge is best effort. `1` keeps the node's reliable reader reliable, for runs that compare against stock ROS 2 reliable delivery. | [Publication Correspondence](../features/correspondence.md) |
| <a id="rcllf_plant_record"></a>`RCLLF_PLANT_RECORD` | runtime | unset | Directory. Records what entered the system, for a later replay. | [Plants](../features/plants.md) |
| <a id="rcllf_plant_replay"></a>`RCLLF_PLANT_REPLAY` | runtime | unset | Directory of a recording. Replays it at its recorded tags. | [Plants](../features/plants.md) |
| <a id="rcllf_readback_dir"></a>`RCLLF_READBACK_DIR` | runtime | unset | Directory. The binding writes `rcllf_readback.<pid>.json`, what the running program actually used, for the read-back check. | [Resolved model](../reference/resolved-model.md) |
| <a id="rcllf_run_token"></a>`RCLLF_RUN_TOKEN` | runtime | empty | Free text copied into every trace record of the process, to tell runs apart. | [Trace schema](../reference/trace-schema.md) |
| <a id="system_json"></a>`SYSTEM_JSON` | runtime | set by the generated launch files | Path of the `system.json` the program was generated from. | [Data model](../reference/data-model.md) |

### Rejected at startup

A program with an admitted edge refuses to start when one of these is set, because it
changes how Fast DDS delivers a sample:

| Name | Refused when |
|---|---|
| `RMW_FASTRTPS_PUBLICATION_MODE` | set to anything but `SYNCHRONOUS` |
| `FASTRTPS_DEFAULT_PROFILES_FILE`, `FASTDDS_DEFAULT_PROFILES_FILE`, `RMW_FASTRTPS_USE_QOS_FROM_XML` | set at all |

A `DEFAULT_FASTRTPS_PROFILES.xml` or `DEFAULT_FASTDDS_PROFILES.xml` in the working
directory is refused the same way. The refusal names the variable or the file.

## Generated launch files

| Name | Read by | Default | Effect | Explained in |
|---|---|---|---|---|
| <a id="rcllf_lf_args"></a>`RCLLF_LF_ARGS` | generated launch files | empty | Extra reactor-c arguments, split like a shell command line and added after the generated ones: `-f true` (fast), `-f false`, `-o 2 sec` (timeout), `-w 2` (workers). The last value of an option wins. | [Fast Mode](../features/fast-mode.md) |
| <a id="rcllf_timer_offsets"></a>`RCLLF_TIMER_OFFSETS` | generated launch files | unset | Start offsets of the timers built with `offset_runtime: true`: `<node>/<timer callback>=<integer> <unit>`, separated by `,` or `;`. An unknown name is refused at launch. | [execution.yaml Reference](../features/execution-config.md#offsets-set-at-program-start) |
| <a id="rti_host"></a>`RTI_HOST` | federate and orchestrator launch files | federate: `localhost`. Orchestrator: empty | Host of the RTI. Empty in the orchestrator launch file means it starts its own RTI. | [Federation](../features/federation.md#several-hosts) |
| <a id="rti_port"></a>`RTI_PORT` | launch files, `rti` service | `15045` | TCP port of the RTI. | [Federation](../features/federation.md#docker-compose-services) |
| <a id="federation_id"></a>`FEDERATION_ID` | federate launch files | empty (the lfc default) | Federation ID. Every federate and the RTI must use the same one. | [Federation](../features/federation.md#docker-compose-services) |
| <a id="rcllf_rti_term_grace_s"></a>`RCLLF_RTI_TERM_GRACE_S` | RTI guard | `3` | Seconds the guard gives the RTI to grant the stop after a SIGTERM before it passes the SIGTERM on. | [Federation](../features/federation.md#stopping-a-federation) |

## Generation time

`colcon lf-generate` and `colcon lf-analyze` read these. Changing one needs a
regeneration.

| Name | Read by | Default | Effect | Explained in |
|---|---|---|---|---|
| <a id="rcllf_count_publishes"></a>`RCLLF_COUNT_PUBLISHES` | `colcon lf-generate` | on | `0` generates a program without per-reaction publish counts. A take whose count is unknown then never waits, and cross-federate links fall back to the count rule. | [execution.yaml Reference](../features/execution-config.md#take_mode-and-maxwait_ms) |

## Docker Compose and container scripts

| Name | Read by | Default | Effect | Explained in |
|---|---|---|---|---|
| <a id="ros_domain_id"></a>`ROS_DOMAIN_ID` | every service | `0` | DDS domain. Give parallel runs on one host different values. | [Docker](../reference/docker.md) |
| <a id="entrypoint_packages"></a>`ENTRYPOINT_PACKAGES` | `docker/entrypoint.sh` | `examples` | What the first `docker compose run --rm dev` builds: `examples` (the runtime and the learning-path examples), `all`, or a list of package names. | [Docker](../reference/docker.md) |
| <a id="entrypoint_skip_build"></a>`ENTRYPOINT_SKIP_BUILD` | `docker/entrypoint.sh` | unset | `1` skips the build at container start. | [Docker](../reference/docker.md) |
| <a id="entrypoint_force_build"></a>`ENTRYPOINT_FORCE_BUILD` | `docker/entrypoint.sh` | unset | `1` builds even when the sources did not change. | [Docker](../reference/docker.md) |
| <a id="entrypoint_force_pip"></a>`ENTRYPOINT_FORCE_PIP` | `docker/entrypoint.sh` | unset | `1` reinstalls the Python tools (`colcon-lf`, `ros2 lf`, `lf-eval`) even when they already import. | [Docker](../reference/docker.md) |
| <a id="entrypoint_parallel_workers"></a>`ENTRYPOINT_PARALLEL_WORKERS` | `docker/entrypoint.sh`, `docker/lf_build.sh` | `2` | `colcon build --parallel-workers`. | [Docker](../reference/docker.md) |
| <a id="makeflags"></a>`MAKEFLAGS` | `docker/entrypoint.sh` | `-j3` | Make jobs per package. | [Docker](../reference/docker.md) |
| <a id="federate"></a>`FEDERATE` | `docker/lf_build.sh` (`lf-build` service) | unset | Builds the packages as a federation: `per-node`, `per-process` or a federation YAML. | [Federation](../features/federation.md) |
| <a id="coordination"></a>`COORDINATION` | `docker/lf_build.sh` | unset (centralized) | `--coordination` of a federated build. | [Federation](../features/federation.md#coordination-mode) |
| <a id="lf_launch"></a>`LF_LAUNCH` | `docker/lf_build.sh` | the package's default launch file | Builds another launch file of the package. | [Docker](../reference/docker.md) |
| <a id="lf_exec_config"></a>`LF_EXEC_CONFIG` | `docker/lf_build.sh` | unset | Passed as `--exec-config` to `colcon lf-generate`. | [execution.yaml Reference](../features/execution-config.md#colcon-lf-configure-and-exec-config) |
| <a id="rti_num_federates"></a>`RTI_NUM_FEDERATES` | `rti` service | `2` | How many federates the standalone RTI waits for. | [Federation](../features/federation.md#docker-compose-services) |
| <a id="fed_timeout"></a>`FED_TIMEOUT` | `lf-fed-publisher`, `lf-fed-subscriber` | `60` | Seconds a federate service runs before it gets SIGINT. The service whose timeout fired exits 124. | [Federation](../features/federation.md#docker-compose-services) |
| <a id="soundness_package"></a>`SOUNDNESS_PACKAGE`, `SOUNDNESS_LAUNCH`, `SOUNDNESS_DURATION`, `SOUNDNESS_STRICT` | `soundness-check` service | `pubsub_example`, `bringup`, `20`, `0` | Package, launch file and seconds of the soundness check. `SOUNDNESS_STRICT=1` fails the service on a finding. | [Soundness](../features/soundness.md) |
| <a id="experiment_trace_dir"></a>`EXPERIMENT_TRACE_DIR` | `docker/trace.sh` | `output/experiments/trace_<date>` | Where the LTTng trace is written. | [Tracing](../features/tracing.md) |
| <a id="lttng_session_name"></a>`LTTNG_SESSION_NAME` | `docker/trace.sh` | `ros2_trace` | LTTng session name, to tell parallel sessions apart. | [Tracing](../features/tracing.md) |

The Compose services pass `ROS2_LF_TAKE_MODE`, `ROS2_LF_MAXWAIT_MS` and
`ROS2_LF_TAKE_TIMEOUT_MS` from the host into the container. A host
`ROS2_LF_MAXWAIT_MS` also sets the alias, so the two never disagree.

## Test and CI only

The test suites and the benchmark tools read further variables: every `RCLLF_TEST_*`,
`ROS2_LF_TEST_START_DELAY`, `ROS2_LF_TEST_RESIGN_DELAY`, `TAKE_FUZZ_LOG`,
`TAKE_TAG_FUZZ_LOG` and `RCLLF_MATCH_PROBE_EVENTS`. They are not for applications, and
some of them make a run log warnings or behave on purpose like a faulty one. The
generator also reads `RCLLF_XFED_FLUSH`, `RCLLF_XFED_SPLIT_DRAIN`,
`RCLLF_LOCAL_SPLIT_DRAIN` and `RCLLF_CLANG_FULL_PARSE`, which select older code paths
for comparison runs.
