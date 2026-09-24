# The Legible Node

Materials from the ROSCon 2026 talk **"The Legible Node: Patterns and tools for clearer, more declarative ROS applications"**.

This repository collects the code that appears on the slides alongside a full, buildable workspace you can run yourself to explore the concepts discussed in the talk.

> [!IMPORTANT]
> **Talk slides:** [View or download *The Legible Node* (PDF)](./the_legible_node_slides.pdf).

## Repository layout

- [snippets/](./snippets): Standalone code snippets shown directly on the slides, extracted for reference and readability.
  They illustrate individual patterns and are not intended to run on their own.
For the complete, runnable examples use the workspace in [`ws/`](./ws/).
- [ws/](./ws): A working ROS 2 workspace that demonstrates the patterns and tools presented in the talk.

## Example Workspace

[ws/](./ws/) contains a working ROS 2 workspace that demonstrates the concepts presented in the talk and slides.

The easiest way to get started is with [pixi](https://pixi.sh) to provide a reproducible, cross-platform ROS 2 environment (via [RoboStack](https://robostack.github.io)), so you don't need a system ROS installation.

### Packages

The workspace presents the same occupancy-grid-map outlier filter in three stages. If you are exploring the repository for the first time, read them in this order:

| Directory | ROS package | What to look at |
| --- | --- | --- |
| [`autoware_occupancy_grid_map_outlier_filter`](./ws/src/autoware_occupancy_grid_map_outlier_filter/) | `autoware_occupancy_grid_map_outlier_filter` | The baseline Autoware implementation. The ROS interface, orchestration, and filtering algorithm all live in the node package. Start with [`src/node.cpp`](./ws/src/autoware_occupancy_grid_map_outlier_filter/src/node.cpp). |
| [`thinned`](./ws/src/thinned/) | `thin_ogmof` | A manual [thin-node refactor](./ws/src/thinned/README.md). The filtering algorithm becomes a reusable library with direct unit tests, while the node is reduced to ROS setup and dispatch. |
| [`nodled`](./ws/src/nodled/) | `nodl_ogmof` | A [NoDL-based refactor](./ws/src/nodled/README.md) built on the thin-node library. A declarative interface drives generated ROS wiring, parameter validation, documentation, and a conformance test. |

The workspace also contains `pixi.toml` and `pixi.lock`, which define the pinned development environment and workspace tasks.

### Prerequisites

Just [pixi](https://pixi.sh/latest/#installation) installed on your machine.

```shell
curl -fsSL https://pixi.sh/install.sh | bash
```

### Setup

Install the environment including all needed build tools and dependencies:

```shell
cd ws/
pixi install
```

Import the source dependencies that aren't available on the pixi/RoboStack channels:

```shell
pixi run import-deps
```

### Workflow

Drop into an interactive shell

```shell
cd ws/
pixi shell
```

Now you have the full ROS workspace environment, including `colcon` for standard workflows.

```shell
colcon build

colcon test --event-handlers console_direct+

# etc...
```
