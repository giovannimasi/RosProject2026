# ROS Project 2026 — Project 7

Game-theoretic motion planning (Turnwald & Wollherr, 2019) on ROS 1 Noetic +
Gazebo, in Docker. Two TurtleBot3 Waffle Pi robots negotiate a shared space with
each other and with lidar-detected pedestrians, using Nash/Pareto equilibria.

**Design & history:** see [`docs/`](docs/) —
[Architecture](docs/ARCHITECTURE.md) (how the code works) and
[Evolution](docs/EVOLUTION.md) (how we got here). Reference paper: [`docs/paper.md`](docs/paper.md).

---

## 1. Setup the container

Docker is preconfigured. Pick your host:

### Windows (WSL2)

The WSL overlay mounts WSLg so Gazebo/RViz GUIs work:

```bash
docker compose -f docker-compose.yml -f docker-compose.wsl.yml up -d --build
```

### Linux

```bash
docker compose up -d --build
```

> `docker compose` auto-loads `docker-compose.override.yml` if present. That file
> is git-ignored (per-developer local tweaks, e.g. software GL or extra mounts) —
> it is **not** required and is absent on a fresh checkout. On Linux, X11 GUI works
> out of the box via the `ros1.sh` helper below.

### Enter the container

```bash
./ros1.sh
```

This grants X11 permission to the Docker user (`xhost +local:docker`) and drops you
into a bash shell inside the `ros1_dev` container. Run it again in another terminal
whenever you need an extra shell (each scenario below needs two).

---

## 2. Build the workspace

Inside the container:

```bash
cd ~/ros1_ws
catkin build
source devel/setup.bash
export TURTLEBOT3_MODEL=waffle_pi   # if not already set in the container .bashrc
```

Re-run `catkin build` after editing the planner; re-`source devel/setup.bash` in
every shell.

---

## 3. Run a simulation

Each scenario needs **two shells** (open a second with `./ros1.sh` from the host):
one launches Gazebo + the world, the other launches the planner + RViz. In **every**
container shell first run:

```bash
source ~/ros1_ws/devel/setup.bash
export TURTLEBOT3_MODEL=waffle_pi
```

### a) Single agent among pedestrians

```bash
# shell 1 — simulation
roslaunch game_theory_sim game_theory.launch
# shell 2 — planner + RViz
roslaunch gt_motion_planner gt_planner.launch
```

### b) Multi-robot (centralized joint game)

```bash
# shell 1
roslaunch game_theory_sim game_theory_multi.launch
# shell 2
roslaunch gt_motion_planner gt_planner_multi.launch
```

### c) Road (final scenario: two robots, opposite directions, walking pedestrians)

```bash
# shell 1
roslaunch game_theory_sim game_theory_road.launch
# shell 2
roslaunch gt_motion_planner gt_planner_road.launch
```

The planner launch also opens RViz showing every evaluated trajectory (chosen one
in thick green) and the lidar/pedestrian data. Per-cycle costs and detections are
logged to the planner terminal.

> **Performance tip:** Gazebo + RViz both use the GPU. On an integrated GPU, running
> Gazebo headless (`gui:=false` on the sim launch) frees resources for RViz. Avoid
> forcing software GL (`LIBGL_ALWAYS_SOFTWARE=1`) unless your driver needs it — it
> renders on the CPU and starves the planner.

---

## Contributors

Giovanni Masi · Pierfilippo Orsini · Federico Melizza · Riccardo Landi · Alessandro Salomone
