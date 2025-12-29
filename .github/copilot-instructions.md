# Copilot / AI agent instructions (autonomous_turtle)

This repository is a small ROS 2 Python package named `autonomous_turtle` (ament_python). The goal of these notes is to help an AI coding agent be immediately productive by describing the architecture, conventions, build/test/debug commands, and important files.

- **Big picture:** a single ROS2 package that provides two console nodes (Python entry points): `move_turtle` and `avoid_walls`. Nodes communicate via standard ROS topics used by `turtlesim`:
  - publishes to `/turtle1/cmd_vel` (message type: `geometry_msgs/Twist`)
  - subscribes to `/turtle1/pose` (message type: `turtlesim/Pose`)

- **Where to look (key files):**
  - package manifest: [src/autonomous_turtle/package.xml](src/autonomous_turtle/package.xml)
  - packaging & entry points: [src/autonomous_turtle/setup.py](src/autonomous_turtle/setup.py)
  - node implementations: [src/autonomous_turtle/autonomous_turtle/move_turtle.py](src/autonomous_turtle/autonomous_turtle/move_turtle.py) and [src/autonomous_turtle/autonomous_turtle/avoid_walls.py](src/autonomous_turtle/autonomous_turtle/avoid_walls.py)
  - tests: [src/autonomous_turtle/test](src/autonomous_turtle/test) (flake8/pep257 lint tests; pytest extras listed in `setup.py`)

- **Architecture & patterns to follow:**
  - Single-package design: keep nodes small and focused (each file defines one Node class and `main()` entry point).
  - Console scripts are declared in `setup.py` under `entry_points`. Modify `entry_points` to expose new nodes.
  - Use `rclpy` idioms: `create_publisher`, `create_subscription`, `create_timer`, and `self.get_logger().info(...)` for logs.
  - Topic names in this repo use absolute topic strings (e.g. `/turtle1/cmd_vel`). When adding features, be explicit about topic names or document them in comments.

- **Build / test / run (developer workflow):**
  - Build the workspace from repository root:

    colcon build --packages-select autonomous_turtle

  - Source the install overlay before running or testing:

    source install/setup.bash

  - Run nodes (after sourcing):

    ros2 run autonomous_turtle move_turtle
    ros2 run autonomous_turtle avoid_walls

  - Tests (recommended):

    colcon test --packages-select autonomous_turtle
    colcon test-result --verbose

  - Linter tests are included (`ament_flake8`, `ament_pep257`) and may fail if standard headers or style are missing. Use the `test` extras in `setup.py` for local pytest runs.

- **Runtime / integration notes:**
  - The nodes expect a `turtlesim` environment. Start `ros2 run turtlesim turtlesim_node` before running these nodes for useful behavior.
  - Use `ros2 topic echo /turtle1/pose` and `ros2 topic pub` to debug and simulate inputs.
  - Because entry points are console scripts, the package can be used via `ros2 run` or invoked directly when packaged.

- **Conventions & gotchas specific to this repo:**
  - `ament_python` packaging (see `package.xml` export): prefer `colcon build` and source the install overlay, not `pip install` into the system Python for development.
  - Tests in `test/` are style/linter focused; functional tests (integration with `turtlesim`) are not present.
  - `__init__.py` is currently empty — adding module-level helpers is allowed, but avoid changing node entry point names unless you update `setup.py`.

- **When making changes an AI should do first:**
  1. Update or add node code in `src/autonomous_turtle/autonomous_turtle/` following existing Node class patterns.
 2. If exposing a new node, add a `console_scripts` entry in `setup.py`.
 3. Run `colcon build` and `colcon test` locally (or instruct the user to do so) and source `install/setup.bash` before running nodes.

If any section needs more detail (example unit/integration test, CI config, or how you want topic naming handled), tell me which part to expand or clarify.
