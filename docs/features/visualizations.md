# Visualizations

`colcon lf-analyze` draws the system it extracted as Graphviz graphs. All of them are
rendered from the extracted `system.json`, so what you see is what the generator
will convert. Use them to check that rcllf found every node, topic and callback
you expect before you generate code.

## Generating visualizations

```bash
docker compose run --rm dev colcon lf-analyze --package pubsub_example
```

One run analyses every launch file of the package (four for `pubsub_example`) and
writes each result to `packages/lf-analysis/<package>/<launch-stem>/`. The graphs
are in the `viz/` folder there, for example
`packages/lf-analysis/pubsub_example/bringup/viz/system_graph.svg`. Open the SVG
files in a browser.

The format is chosen with `--format`:

| Option | Writes |
|---|---|
| `--format all` (the default) | the JSON model and SVG graphs |
| `--format svg` | the JSON model and SVG graphs |
| `--format pdf` | the JSON model and PDF graphs (`viz/topology.svg` and `launch/launch_graph.svg` stay SVG) |
| `--format json` | the JSON model only, no `viz/` folder |

For `pubsub_example/bringup`, the default writes:

```text
viz/launch_graph.svg
viz/comm_graph.svg
viz/system_graph.svg
viz/topology.svg
viz/nodes/publisher_node_internals.svg
viz/nodes/subscriber_node_internals.svg
```

With `--format pdf` the same files appear as PDF, except `viz/topology.svg`. To
redraw the graphs from an existing `system.json` without extracting again, add
`--viz-only`.

The `dot` binary of the `graphviz` package draws them. It is installed in the
container. Outside the container, install it with `sudo apt install graphviz`
(Ubuntu/Debian) or `brew install graphviz` (macOS).

## Launch graph

**File:** `viz/launch_graph.svg` (also written to `launch/launch_graph.svg`)

Shows the launch file include hierarchy as a graph:

- **Blue boxes:** launch files (package and file name)
- **Yellow ellipses:** ROS 2 nodes (package and node name)
- **Dashed boxes:** group actions with a namespace
- **Edges:** include relationships

It answers: "What does this launch file start, and how are things organized?"

## Node internals

**Files:** `viz/nodes/<node_name>_internals.svg`

Shows one node's internal structure:

- **Diamonds:** triggers (timers, subscriptions, service servers, action servers)
  with callback group annotations. Service servers are pink.
- **Green boxes:** callbacks with parameter signatures
- **Salmon boxes:** publishers
- **Pink boxes:** service clients
- **Purple boxes:** action clients
- **Gray text:** member variables with read and write edges
- **Yellow note:** declared parameters with types and defaults
- **Edges:** trigger to callback, callback to publisher, callback to variable

It answers: "What does this node do internally? What triggers what?"

## Communication graph

**File:** `viz/comm_graph.svg`

Shows inter-node communication:

- **Blue boxes:** nodes
- **Solid black edges:** topic connections, labeled with topic name and message type
- **Dashed blue edges:** service connections (client to server)
- **Bold purple edges:** action connections (client to server)

It answers: "How do nodes talk to each other?"

## System graph

**File:** `viz/system_graph.svg`

A unified view of all layers:

- **Blue cluster borders:** launch file scopes
- **Dashed gray clusters:** namespace groups
- **Yellow record boxes:** nodes with a summary (timer count, pub/sub topics, services)
- **Inter-node edges:** the same as the communication graph

It answers: "What does the full system look like?"

## Topology

**File:** `viz/topology.svg`

A fifth picture, drawn in the style of an LF program: one box per node with its
ports and reactions, and wires for the topics between them. It is always written
as SVG. Its layout suits linear pipelines. On systems with many fan-in and fan-out
edges the wires can overlap, so use the system graph there.

## Limits

Some extracted fields (remappings, launch arguments, composable-container
membership) are not drawn yet. They are still in `system.json`.
