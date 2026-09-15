# The Legible Node

Materials from the ROSCon 2026 talk **"The Legible Node: Patterns and tools for clearer, more declarative ROS applications"**.

This repository collects the code that appears on the slides alongside a full, buildable workspace you can run yourself to explore the concepts discussed in the talk.

## Repository layout

- [snippets/](./snippets): Standalone code snippets shown directly on the slides, extracted for reference and readability.
  They illustrate individual patterns and are not intended to run on their own.
For the complete, runnable examples use the workspace in [`../ws`](../ws).
- [ws/](./ws): A working ROS 2 workspace that demonstrates the patterns and tools presented in the talk.

## Example Workspace

[ws/](./ws/) contains a working ROS 2 workspace that demonstrates the concepts presented in the talk and slides.

The easiest way to get started is with [pixi](https://pixi.sh) to provide a reproducible, cross-platform ROS 2 environment (via [RoboStack](https://robostack.github.io)), so you don't need a system ROS installation.

### Contents

- `src/` — ROS 2 packages demonstrating the patterns from the talk.
- `pixi.toml` / `pixi.lock` — The pinned environment and workspace tasks.

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
