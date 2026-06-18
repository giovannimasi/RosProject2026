# ROS & System Design

This document explains the **engineering** side: the ROS concepts used and *where*,
the technical choices and *why*, the workspace/repo structure, the build system, and
the Docker image. For the planning **algorithm** see [ARCHITECTURE.md](ARCHITECTURE.md).

---

## 1. Technology stack & rationale

| Choice | Why |
|--------|-----|
| **ROS 1 Noetic** | Both ROS 1 and ROS 2 Gazebo run only under **XWayland** (neither is native Wayland). On the dev setup ROS 2 + Gazebo did **not** work under XWayland, while ROS 1 + Gazebo Classic did — so ROS 1 was chosen for GUI compatibility. Noetic is the last ROS 1 LTS (Ubuntu 20.04). |
| **C++** (not Python) | The planner is heavy: every cycle runs many OMPL RRT solves + a joint game over the action cross-product + per-point collision/cost over long trajectories. C++ gives the throughput; a Python loop would not keep an interactive rate. (Python deps such as `nashpy` are provisioned in the image but unused — that path was abandoned.) |
| **OOP, single node class** | The whole planner is one class (`GameTheoryPlannerNode`): ROS handles, parameters, OMPL setup, perception, the game, and control are encapsulated as members/methods. State shared between the planning thread and the control thread (poses, the chosen path) lives in members guarded by mutexes — OOP keeps that ownership explicit. |
| **OMPL** (control-based RRT) | Sampling-based kinodynamic planner used as the per-robot **action generator** — avoids hand-rolling an RRT and gives a proper unicycle propagator + state-validity checker. |
| **Gazebo Classic** + **TurtleBot3 Waffle Pi** | Standard ROS 1 simulation; the Waffle Pi ships a differential-drive + 2D lidar model, exactly what the planner consumes. |
| **Docker** | Reproducible environment with pinned ROS/Gazebo/OMPL and the actor-collision plugin prebuilt (see §5). |
| **catkin_tools** (`catkin build`) | Per-package isolated builds, nicer than `catkin_make` for a multi-package workspace. |

---

## 2. Repository / workspace structure

```
ros/
├── Dockerfile                     # image: ROS Noetic + Gazebo + TB3 + OMPL + actor plugin
├── docker-compose.yml             # base service (container ros1_dev, host network, X11)
├── docker-compose.wsl.yml         # Windows/WSL2 overlay (mounts WSLg)
├── ros1.sh                        # xhost (only for linux) + exec into the container
├── docs/                          # this documentation + paper
└── ros1_ws/                       # catkin workspace (mounted into the container at /root/ros1_ws)
    └── src/
        ├── gt_motion_planner/     # THE planner (C++ node, configs, launch, RViz)
        ├── game_theory_sim/       # simulation bring-up: worlds, launch, URDF namespacing
        └── gazebo_random_actor/   # Gazebo model plugin that walks the pedestrian actors (deprecated after first simulation)
```

**Why three packages** — separation of concerns, the conventional ROS layout:

- **`gt_motion_planner`** — the algorithm. Node `gt_planner_node` (`src/gt_planner_node.cpp`),
  per-scenario `config/*.yaml`, `launch/*.launch` (planner + RViz), `config/*.rviz`.
  This is the only package that contains the planning logic.
- **`game_theory_sim`** — *bring-up only*: Gazebo `worlds/` (`actor_collisions.world`,
  `two_robots.world`, `road.world`), `launch/` that start Gazebo + spawn robots, and
  `scripts/gen_ns_urdf.sh` (URDF namespacing, §3). It also carries an early planner
  skeleton (`src/`) kept for history but **superseded** by `gt_motion_planner`.
- **`gazebo_random_actor`** — a Gazebo **model plugin** (`random_smooth_actor_plugin.cc`)
  that drives the pedestrian actors. It is a simulation component, not part of the
  robot's software, so it lives in its own package.

Keeping the planner, the simulated world, and the actor-driver in distinct packages
means the planner can be built/run against any world, and the sim can be changed
without touching the algorithm.

---

## 3. ROS graph — concepts and where they are used

### Node

One C++ node, **`gt_planner_node`**, controls the whole fleet (centralized game).
Gazebo, `robot_state_publisher` (one per robot), a static TF publisher, and RViz are
the other nodes in the graph.

### Topics, messages, publishers/subscribers

| Direction | Topic | Message type | Role |
|-----------|-------|--------------|------|
| **Sub** | `/gazebo/model_states` | `gazebo_msgs/ModelStates` | ground-truth world poses of robots (and, in older scenarios, actors) |
| **Sub** | `/<ns>/scan` (e.g. `/tb1/scan`) | `sensor_msgs/LaserScan` | per-robot 2D lidar → pedestrian perception |
| **Pub** | `/<ns>/cmd_vel` (e.g. `/tb1/cmd_vel`) | `geometry_msgs/Twist` | velocity command to each robot's diff-drive |
| **Pub** | `gt_planner/markers` | `visualization_msgs/MarkerArray` | RViz: all evaluated trajectories, Nash/Pareto, chosen path, lidar points |

This is **asynchronous publish/subscribe** — the right pattern for streaming sensor
data (`/scan`, `/model_states`) and continuous commands (`/cmd_vel`). Subscriber
callbacks store the latest message; the planning and control loops consume the most
recent snapshot.

### Services / Actions — deliberately *not* used

No ROS **services** (request/response) or **actions** (goal/feedback/result) are used.
The task is a continuous control loop, not a discrete request: goals are internal
(per-robot waypoint lists in the config), advanced automatically when reached. There
is no caller waiting for a one-shot reply, so the pub/sub + timer model fits and
services/actions would add latency and complexity for no benefit. (An action server
*could* expose "go to goal X" to an external client — noted as a possible extension.)

### Timers & the spinner model

- A **`ros::Timer`** drives planning. It is **one-shot and self-rearming**: each cycle
  re-arms the next only after finishing, so cycles never overlap and the rate
  self-adapts to CPU load.
- A second **`ros::Timer`** runs the high-rate controller (pure-pursuit → `/cmd_vel`).
- The node spins with a **`ros::AsyncSpinner`** (multiple threads) so the slow planning
  callback, the fast control callback, and the sensor callbacks run concurrently.
  Shared state is protected by two `std::mutex` (one for the latest `model_states`
  snapshot, one for the chosen plan handed to the controller). This is the standard
  ROS answer to "one callback is slow but others must stay responsive."

### Namespaces

Multi-robot uses **namespaces** to give each robot its own topic tree:
`/tb1/cmd_vel`, `/tb1/scan`, `/tb2/cmd_vel`, … Each robot is spawned inside a
`<group ns="tbN">` with its own `robot_state_publisher` and a `tf_prefix`. The stock
TurtleBot3 Gazebo plugins, however, do **not** honor the namespace for their topics, so
`scripts/gen_ns_urdf.sh` injects a `<robotNamespace>` into the diff-drive and laser
plugin blocks of the xacro at spawn time — that is what actually routes the plugin
topics to `/tbN/...`. The planner subscribes/publishes per-robot using these namespaced
topic names from its config.

### TF

TF is intentionally **minimal**. The planner reads world-frame poses directly from
`/gazebo/model_states` and transforms lidar points to the world frame using those
poses (treating `base_scan ≈ base_link`), instead of relying on the TF tree — this
avoids TF timing/lookup issues and keeps perception simple. A single static
`world → map` transform is published only so RViz has a fixed frame for the markers
(which are published in the `world` frame). `robot_state_publisher` still broadcasts
each robot's internal joints with a `tf_prefix`.

### Parameters

All tunables are **ROS parameters** loaded from per-scenario YAML via
`<rosparam file=...>` in the launch files and read with `pnh.param<T>(...)` at startup
(see the tunables tables in ARCHITECTURE.md). Robots and their goals are themselves
parameters (a list of dicts), so the same node code serves the single-, multi-, and
road scenarios with only a different config.

---

## 4. Build system

- **catkin_tools** (`catkin build`) over the `ros1_ws` workspace.
- `gt_motion_planner/CMakeLists.txt`: `add_compile_options(-std=c++14)`;
  `find_package(catkin REQUIRED COMPONENTS roscpp gazebo_msgs geometry_msgs sensor_msgs
  visualization_msgs tf)`; `find_package(ompl REQUIRED)`; `find_package(Boost ...)`;
  one executable `gt_planner_node` linked against catkin + OMPL + Boost.
- `package.xml` declares the message/lib dependencies (`gazebo_msgs`, `geometry_msgs`,
  `sensor_msgs`, `visualization_msgs`, `roscpp`, `tf`, `ompl`) so `rosdep` and the build
  resolve them.

---

## 5. Docker image (`Dockerfile`)

Built `FROM osrf/ros:noetic-desktop-full`, adding exactly the dependencies the project
needs so the environment is reproducible:

- **Sim & robot**: `ros-noetic-turtlebot3(+msgs, +simulations)`, `ros-noetic-gazebo-ros-pkgs`,
  `ros-noetic-gazebo-ros-control`.
- **Planner**: `ros-noetic-ompl`, `ros-noetic-visualization-msgs`, `ros-noetic-tf/tf2`.
- **Tooling**: `python3-catkin-tools` (the `catkin build` tool), git/tmux/editors.
- **Actor collisions**: the upstream `actor_collisions` Gazebo plugin is cloned and
  **compiled at image build**, and `GAZEBO_PLUGIN_PATH` is set to its `build/` dir so
  worlds can reference `libActorCollisionsPlugin.so` by name. Gazebo `<actor>` skins
  have no collision by default — this plugin gives them collision geometry so the
  lidar can see the pedestrians.
- `rosdep` is initialized/updated. The workspace is **mounted** (not copied) from the
  host into `/root/ros1_ws`, so code edits on the host are live in the container.

Compose: `docker-compose.yml` is the base service (`ros1_dev`, host networking, X11
socket); `docker-compose.wsl.yml` overlays the WSLg mount for Windows. See the root `README.md` for
the exact up/build/run commands.
