# Visualizations

The extraction tool generates four types of Graphviz DOT graphs, rendered to PDF and/or SVG. All visualizations read from the JSON output (not from dataclasses directly), ensuring the JSON is the single source of truth.

## Launch Graph

**File:** `viz/launch_graph.{pdf,svg}`

Shows the launch file include hierarchy as a DAG:

- **Blue boxes:** Launch files (package + filename)
- **Yellow ellipses:** ROS 2 nodes (package + node name)
- **Dashed boxes:** GroupActions with namespace
- **Edges:** Include relationships

This graph answers: "What does this launch file start, and how are things organized?"

## Node Internals

**Files:** `viz/nodes/{node_name}_internals.{pdf,svg}`

Shows one node's internal structure:

- **Diamonds:** Triggers (timers, subscriptions, service servers, action servers) with callback group annotations
- **Green boxes:** Callbacks with parameter signatures
- **Salmon boxes:** Publishers
- **Pink boxes:** Service clients
- **Purple boxes:** Action clients
- **Gray text:** Member variables with read/write edges
- **Yellow note:** Declared parameters with types and defaults
- **Edges:** Trigger->callback (triggers), callback->publisher (publish), callback->variable (reads/writes)

This graph answers: "What does this node do internally? What triggers what?"

## Communication Graph

**File:** `viz/comm_graph.{pdf,svg}`

Shows inter-node communication:

- **Blue boxes:** Nodes
- **Solid black edges:** Topic connections (labeled with topic name + message type)
- **Dashed blue edges:** Service connections (client -> server)
- **Bold purple edges:** Action connections (client -> server)

This graph answers: "How do nodes talk to each other?"

## System Graph

**File:** `viz/system_graph.{pdf,svg}`

Unified hierarchical view combining all layers:

- **Blue cluster borders:** Launch file scopes
- **Dashed gray clusters:** Namespace groups
- **Yellow record boxes:** Nodes with summary (timer count, pub/sub topics, services)
- **Inter-node edges:** Same as communication graph (topics, services, actions)

This graph answers: "What does the full system look like?"

## Generating Visualizations

```bash
# All formats (PDF + SVG)
colcon lf-analyze --package pubsub_example --format all

# SVG only
colcon lf-analyze --package pubsub_example --format svg

# JSON only (skip visualizations)
colcon lf-analyze --package pubsub_example --format json
```

## Requirements

Visualization rendering requires the `graphviz` system package (provides the `dot` binary). It is pre-installed in the Docker dev image. For local use:

```bash
# Ubuntu/Debian
sudo apt install graphviz

# macOS
brew install graphviz
```
