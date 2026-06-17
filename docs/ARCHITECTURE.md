# Architecture & Design (current code)

This document describes the planner as it stands now. It is the living reference
for how the algorithm is built; update it as the code changes.

---

## 1. Overview

A single ROS node (`gt_planner_node`) drives one or more TurtleBot3 robots toward
their goals while avoiding each other and pedestrians, by playing a
**non-cooperative static game** every planning cycle. Each robot proposes a set
of candidate trajectories (its *actions*); the joint outcome is scored with the
paper's cost (path length + infinite cost on collision); Nash equilibria are
found, Pareto-filtered, and one is selected. The chosen trajectory is then
executed by a separate high-rate controller.

Key properties:

- **C++**, OMPL **control-based RRT** as the action generator.
- **Centralized joint game** (paper §4.3.3): all controlled robots are players in
  one game with one shared equilibrium selection.
- **Pedestrians are perceived, not known**: detected and tracked from raw lidar
  (no `model_states` identities), classified as moving, and predicted. They enter
  the game as single-action predicted agents (paper §4.3.4 in spirit).
- **Decoupled control**: planning (slow, ~1–2 Hz) and command publishing (smooth,
  ~15 Hz) run on separate threads, so the robot never freezes on a slow cycle.

---

## 2. Process & threading

```
AsyncSpinner(3)
 ├─ ModelStatesCallback     writes latest_msg_ (ground-truth poses) under state_mutex_
 ├─ ScanCb(i)               per-robot /scan into robots_[i].scan
 ├─ TimerCallback (oneshot) → PlanCycle(): one full planning cycle, then re-arms
 │                            itself (control_rate = gap after completion)
 └─ ControlPublish (~15 Hz) pure-pursuit of the last chosen path → /cmd_vel
```

- The **planning timer is one-shot and self-rearming**: the next cycle starts only
  after the previous finishes, so cycles never overlap or get truncated; the rate
  auto-adapts to the machine. `control_rate` is the post-cycle gap.
- `PlanCycle` snapshots `latest_msg_` into `work_msg_` once (under `state_mutex_`)
  and works on that snapshot, so it never holds a lock during the heavy work.
- The chosen path + a `follow_stop` flag are published to the controller under
  `plan_mutex_`. **Only `ControlPublish` writes `/cmd_vel`** (no two-writer races).

---

## 3. Planning cycle (`PlanCycle`)

1. **Perception** (`BuildPerception`, §4): lidar → static occupancy grid +
   dynamic pedestrians (`dyn_obs_`).
2. **Per-robot action sets** (`GenerateRobotActions`, §5): for each robot, load its
   per-robot context (goal, escape state, memorized action, hysteresis reference)
   into shared members ("context swap"), then build its candidate trajectories.
3. **Pedestrian agents**: each detected pedestrian becomes a non-robot agent with a
   **single** predicted-trajectory action.
4. **Joint game** (§6): enumerate the action cross-product, build the cost table,
   find Nash equilibria, Pareto-filter, select one row (`SelectJoint`).
5. **Dispatch**: for each robot, if the selected action's cost is `INF` (every path
   collides) or the static safety brake fires → request stop; else store the chosen
   path for the controller. Memorize the chosen `(v, w)` and trajectory.
6. **Markers + debug log**.

---

## 4. Perception — lidar pedestrian detection (`BuildPerception`)

The robot does **not** know who the pedestrians are. Pipeline each cycle:

1. **Merge** every robot's `/scan` into world-frame points using that robot's
   ground-truth pose (`base_scan ≈ base_link`, sub-grid offset ignored).
2. **Fleet filter**: drop points within `lidar_robot_filter_radius` of *other*
   controlled robots (known fleet bodies). **Never** filter around a robot's own
   centre — a lidar can't see itself, so points near its own pose are real
   obstacles (filtering them once caused close obstacles to vanish → collisions).
3. **Cluster**: grid connected-components at `cluster_cell` (large enough to merge a
   pedestrian's two legs into one stable blob).
4. **Track**: greedy nearest-centroid association within `track_gate`; velocity is
   estimated over a **time window** (`vel_window`, ~0.5 s) from raw centroids — not
   frame-to-frame, which leg swing made far too noisy — with **outlier rejection**
   (a measured speed above ~1.5·`ped_max_speed` is an association glitch and is
   dropped). Tracks survive `track_max_misses` missed frames.
5. **Classify**: a track confirmed (`track_min_hits`) and moving
   (`≥ dyn_speed_thresh`) becomes **dynamic** and stays dynamic for its whole life
   (`ever_dynamic` — a pedestrian must never flip back to a static "wall", which
   used to trigger spurious braking).
   - **Static** clusters → inflated into the shared occupancy grid (`InsertInflated`,
     `lidar_inflation`).
   - **Dynamic** clusters → `dyn_obs_` (position + filtered velocity).

Pedestrian prediction (`PredictDynObs`): constant-velocity straight line, sampled at
`integrator_dt` over `ped_predict_horizon` (≈7 s, longer than the robot horizon so
a head-on is detected early). One action only ⇒ it does not enlarge the joint space
but still makes any robot action that hits it (time-indexed) cost `INF`.

---

## 5. Robot action set (`GenerateRobotActions`)

Per robot, per cycle:

- **OMPL control-RRT, full-to-goal**: `robot_num_actions` independent solves. The
  control sampler (`Eq8ControlSampler`, paper Eq. 8) fixes linear speed `robot_v`
  and samples one angular rate `w ∈ [w_min, w_max]` (plus a curvature-scaled
  variant) per action; the unicycle `StatePropagator` integrates at `integrator_dt`.
  Each solution is the full path toward the goal region.
- **Memorized action** (paper §4.3.3 "remembering the winning combination"):
  re-offers the *previous* chosen trajectory (not a `(v,w)` replay, which spiralled).
  Skipped if the previous command was a stop, to avoid freeze-replay.
- **Stand-still** (`τ⁰`): zero motion, cost just above any moving action.
- **Fallback arcs**: deterministic Eq.8 arcs, added **only if** RRT returned nothing
  (guarantees motion without competing with full-to-goal paths in normal conditions).

Each action's trajectory is also flagged `obstacle_blocked` (hits a static lidar/
object cell) or `group_blocked` (enters a group corridor; unused in the road
scenario). In **escape mode** every action's cost is overridden to `−clearance` of
its endpoint (see §8).

---

## 6. The game (cost, Nash, Pareto, selection)

**Cost** (`BuildCostTable`), per allocation row, per agent `i`:

```
collided[i]  = any pair (i,j) whose time-indexed trajectories come within d_min
               d_min = robot_collision_dist  if both i,j are robots
                       dyn_collision_dist     otherwise (robot–pedestrian)
cost[i] = INF                      if collided[i] or (robot & obstacle/group blocked)
        = TrajectoryCost(action)   otherwise
```

`TrajectoriesCollide` compares positions **at the same time index** — the robot only
treats it as a conflict if both bodies are at the crossing point *at the same moment*,
not anywhere along the pedestrian's line.

`TrajectoryCost = traveled + remaining_to_goal (+ penalties)`:
- `+ goal_block_penalty` if the straight line from the path end to the goal is
  blocked (keeps a short path aimed into an obstacle from looking cheap).
- `+ road_penalty · Σ excursion` for points outside `[road_min_x, road_max_x]` when
  `road_keep` is on — a *soft* lane keeper (can briefly leave the lane to dodge,
  then return).

**Equilibria**: `FindNash` marks rows where no single agent can unilaterally lower
its cost; `ParetoFilter` keeps the non-dominated ones. `SelectJoint` picks the
Pareto row minimizing the **sum of controlled-robot costs + a hysteresis penalty**
(§8). Falls back to Nash if Pareto is empty.

---

## 7. Execution — decoupled controller (`ControlPublish`, `PurePursuit`)

Runs at `control_pub_rate`, independent of planning:

- Takes the robot's current pose and the last planned path; **pure-pursuit** picks a
  point ~`lookahead` ahead and steers to it at `robot_v`.
- Published angular velocity is **smoothed** (`w_ema`): RRT paths are jagged, so raw
  pure-pursuit jittered the steering ±0.2 rad/s per tick and the robot drifted off
  its planned path into collisions; the low-pass gives fluid steering that actually
  follows the plan.
- Turn slowdown is **gentle** (floored at 0.55·speed) — an earlier hard slowdown made
  the robot crawl through dodges and goal U-turns.
- **Hard stop within `goal_tolerance`** (no slow creep toward the goal while waiting
  for the next plan) and on `follow_stop`.

---

## 8. Supporting mechanisms

- **Safety brake** (`SafetyBrake`) — **static only**: stops if a future trajectory
  point comes within physical distance (`agent_radius + brake_margin`) of a static
  lidar point/object that is **ahead** (within `brake_cone` of heading). It does *not*
  brake when passing beside an obstacle. Pedestrian avoidance is *not* the brake's
  job — that is game-driven (a colliding selected action has cost `INF` ⇒ stop).
- **Escape mode**: if the robot's own pose is inside a blocked cell
  (`StateBlockedRaw`), the objective flips to maximizing clearance (action cost =
  `−clearance`); a conditional **ego-bubble** frees the RRT start so it can escape,
  but only while trapped (so it can't carve a hole and drive *into* an obstacle).
- **Hysteresis** (`hysteresis_weight`, `EarlyDir`): penalizes changing the initial
  heading vs. the previous cycle's chosen path → commits to one side / homotopy,
  preventing left-right zig-zag indecision.
- **Diagnostics**: per cycle the node logs each robot's selected action, cost
  (`INF`/value), closest approach to the nearest pedestrian `ca=dist@time`, closest
  approach to the other robot `rr=dist@time`, and the controller logs the actual
  published `pub=v/w` with `STOP/ATGOAL` flags.

---

## 9. Tunables (road config)

| Group | Params |
|------|--------|
| RRT | `robot_v`, `w_min/w_max`, `curvature_factor`, `prop_*`, `robot_num_actions`, `rrt_solve_time`, `planning_horizon`, `integrator_dt` |
| Workspace | `ws_min/max_x/y`, `road_keep`, `road_min/max_x`, `road_penalty` |
| Collision | `agent_radius`, `dyn_collision_dist` (robot–ped), `robot_collision_dist` (robot–robot), `goal_block_penalty` |
| Lidar grid | `use_lidar`, `lidar_grid_res`, `lidar_inflation`, `static_margin`, `lidar_robot_filter_radius`, `ego_bubble` |
| Tracker | `cluster_cell`, `track_gate`, `track_min_hits`, `track_max_misses`, `dyn_speed_thresh`, `vel_window`, `dyn_latch_frames`, `ped_max_speed`, `ped_predict_horizon` |
| Control | `control_rate` (planning gap), `control_pub_rate`, `lookahead`, `w_ema` |
| Safety brake | `safety_brake`, `brake_lookahead`, `brake_margin`, `brake_cone` |
| Selection | `hysteresis_weight` |

---

## 10. From the paper vs. built from scratch

**From Turnwald & Wollherr (2019):**
- The game-theoretic formulation: actions as finite trajectories, cost
  `J = Ĵ(length) + J̃(∞ on collision)`, **Nash** equilibria, **Pareto** optimality.
- Centralized all-controllable joint game (§4.3.3) and the among-humans setting
  (§4.3.4) with trajectory prediction and equilibrium memory.
- The **stand-still** action and the idea of an **emergency stop** safety layer.
- The discrete control set (Eq. 8) and unicycle model (Eq. 7).

**Built from scratch (engineering on top of the paper):**
- **Lidar pedestrian perception** — cluster → track → classify from leg motion, with
  no a-priori identities; window velocity + outlier rejection + sticky `ever_dynamic`.
- **OMPL control-RRT** as the concrete action generator, in a **full-to-goal** variant
  with an admissible remaining-distance cost term.
- **Decoupled architecture**: async planning + high-rate pure-pursuit controller,
  self-rearming planning timer, steering smoothing.
- **Static safety brake** (directional, physical-distance) separate from the
  game-driven dynamic stop.
- **Escape mode**, conditional ego-bubble, and **hysteresis** homotopy commitment.
- **Soft road-corridor** lane keeping.
- **Multi-robot Gazebo plumbing**: namespaced URDF injection, merged fleet lidar.
- All robustness tuning: self-filter fix, robot-vs-pedestrian collision buffers,
  goal-creep elimination.
