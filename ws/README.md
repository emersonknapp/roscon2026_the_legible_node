# Workspace

A working ROS 2 workspace that demonstrates the concepts presented in the talk **"The Legible Node: Patterns and tools for clearer, more declarative ROS applications"**.

The workspace uses [pixi](https://pixi.sh) to provide a reproducible, cross-platform ROS 2 environment (via [RoboStack](https://robostack.github.io)), so you don't need a system ROS installation to build and run it.

## Contents

- `src/` — ROS 2 packages demonstrating the patterns from the talk.
- `pixi.toml` / `pixi.lock` — The pinned environment and workspace tasks.

## Prerequisites

- [pixi](https://pixi.sh/latest/#installation) installed on your machine.

## Setup

1. Install pixi (if you haven't already):

   ```bash
   curl -fsSL https://pixi.sh/install.sh | bash
   ```

2. From this `ws/` directory, install the environment. This resolves and
   downloads everything pinned in `pixi.lock`:

   ```bash
   pixi install
   ```

## Building

Build the workspace:

```bash
pixi shell
colcon build --symlink-install
```

## Working interactively

To drop into a shell with the environment (and built overlay) activated:

```bash
pixi shell
```

From there you can use `ros2`, `colcon`, and other tools directly.
