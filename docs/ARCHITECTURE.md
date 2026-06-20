# Architecture & Design (current code)

This document describes the planner as it stands now. It is the living reference
for how the algorithm is built; update it as the code changes.

---

## 1. Overview

A single ROS node (`gt_planner_node`) drives one or more TurtleBot3 robots toward
their goals while avoiding each other and pedestrians, by playing a
**non-cooperative static game** every planning cycle. Each robot proposes a set
of candidate trajectories (its *actions*); the joint outcome is scored with the
same cost as presented in the paper; Nash equilibria are found, Pareto-filtered, 
and one is selected. The chosen trajectory is then executed by a separate high-rate
controller.

Key properties:

- **C++**, OMPL **control-based RRT** as the action generator.
- **Centralized joint game** (paper §4.3.3): all controlled robots are players in
  one game with one shared equilibrium selection.
- **Pedestrians are perceived, not known**: detected and tracked from raw lidar,
  classified as moving, and predicted. They enter the game as single-action 
  predicted agents.
- **Decoupled control**: planning (slow, ~1–2 Hz) and command publishing (smooth,
  ~15 Hz) run on separate threads, so the robot never freezes on a slow cycle.

---

## 2. Process & threading

Planning is expensive and its duration varies, while the velocity commands must come
out smoothly; the two therefore run on **separate threads** under a multi-threaded
spinner, together with the sensor callbacks (poses and lidar), which simply store the
latest data as it arrives.

Planning runs as a **self-rescheduling cycle**: each cycle re-arms the next one only
after it has finished, so cycles never overlap or get cut short and the effective rate
adapts to whatever the machine can sustain (typically ~1–2 Hz). The controller instead
runs on a **fixed high-rate timer** (~15 Hz), independent of how long a planning cycle
takes, so the robot keeps moving along its last plan even while the next one is being
computed.

To keep the threads from interfering, a planning cycle copies the latest poses into a
private snapshot at its start and then computes on that copy, holding no lock during the
heavy work; when it finishes it hands the chosen path (and a stop flag) to the controller
through a brief lock. The controller is the **only** producer of velocity commands, so
the two threads never write the command at the same time.

---

## 3. Planning cycle (`PlanCycle`)

1. **Perception** (§4): the merged lidar is turned into a static occupancy grid plus 
   a list of moving pedestrians with their estimated velocities.
2. **Per-robot action sets** (§5): for each robot the planner loads that robot's context
   (goal, escape state, hysteresis reference) and uses it to build its candidate
   trajectories.
3. **Pedestrian agents**: each detected pedestrian is added as a non-controlled agent
   whose only action is its single predicted trajectory.
4. **Joint game** (§6): the planner enumerates the combinations of all agents' actions,
   scores them in the cost table, keeps the Nash equilibria, Pareto-filters them, and
   selects one combination.
5. **Dispatch**: for each robot, if the selected action's cost is `INF` (every path
   collides) or the static safety brake fires, the robot is told to stop; otherwise its
   chosen path is stored for the controller and kept as the hysteresis reference for the
   next cycle.

---

## 4. Perception — lidar pedestrian detection (`BuildPerception`)

The robot does **not** know who the pedestrians are; it discovers them from the lidar
each cycle, through the following steps.

1. **Merge.** The lidar returns of all robots are combined into a single set of points
   in the world frame, using each robot's known pose to place them.
2. **Remove the fleet.** Points that fall on the *other* controlled robots are dropped,
   since those positions are known. A robot's own surroundings are never dropped: a
   lidar cannot see itself, so a return next to a robot is a real nearby obstacle.
   (Dropping them once made close obstacles disappear from the map and caused
   collisions.)
3. **Cluster.** The remaining points are grouped into blobs, with a grouping size large
   enough that a walking person's two legs form one stable blob instead of flickering as
   two.
4. **Track.** Each blob is matched to the nearest blob from previous frames, and its
   velocity is estimated over a short time window (about half a second) rather than
   between consecutive frames — over a single frame the leg motion swamps the real
   displacement and the velocity becomes noise. A physically impossible speed is treated
   as a matching glitch and ignored, and a track survives a few missed frames before
   being dropped.
5. **Classify.** A blob that has been seen for a few frames and is actually moving is
   labelled a pedestrian, and it keeps that label for the rest of its life — a confirmed
   pedestrian is never mistaken again for a static wall (which used to cause spurious
   braking). Blobs that stay still are added to the obstacle map; moving ones are kept
   as pedestrians together with their estimated velocity.

Finally, each pedestrian's future is predicted as a straight line at constant velocity,
over a horizon longer than the robot's own (about seven seconds) so that a head-on
meeting is noticed while it is still far away. A pedestrian carries a single predicted
trajectory, which is enough to forbid any robot path that would meet it at the same
moment without enlarging the search.

---

## 5. Robot action set (`GenerateRobotActions`)

Each cycle the planner builds a small set of candidate trajectories for every robot,
drawn from up to three sources.

- **Full paths to the goal.** A handful of complete trajectories from the robot's
  current pose to the goal, produced by the sampling-based kinodynamic planner. Each
  keeps the cruise speed fixed and explores a different turn rate, so the set spans
  several ways of reaching the goal (more to the left, straighter, more to the right).
- **Stand-still.** The option of staying put, priced just above any moving option, so
  the robot prefers to move but can choose to wait.
- **Fallback arcs.** A few simple fixed arcs, added only when the kinodynamic planner
  found no path at all, so the robot always has something to do in a hard spot without
  these short arcs competing with the full paths in normal conditions.

There is deliberately **no "memorized" / previous-trajectory action**. Re-offering the
last cycle's path was identical to the hysteresis reference, so it always scored zero
hysteresis penalty and beat every fresh path, locking the selection onto a stale
trajectory frozen at an old start point even as the robot moved. Plan continuity now
comes only from hysteresis (§8), which compares the *fresh* candidates against the
previous choice, and from the controller, which keeps following the last path between
replans.

Every candidate is also marked if it passes through a known static obstacle. When the
robot is trapped, the scoring switches so it prefers whichever candidate moves it
farthest away from everything (escape mode, §8).

---

## 6. The game (cost, Nash, Pareto, selection)

**Cost** (`BuildCostTable`). For each combination of actions the planner checks every
pair of agents for collision: two trajectories collide if, at any shared time step,
their centres come closer than a minimum distance (larger if between two robots). 
An agent whose action collides is given cost `INF`; otherwise its cost is 
its `TrajectoryCost`.

The check is **time-indexed**: positions are compared step by step in time, so a robot
is in conflict with a pedestrian only if they would reach the crossing point *at the
same moment*; crossing the pedestrian's line after it has passed is allowed. The tight
hard distances mean the robot stops only on a real conflict.

On top of the hard cost, a **soft comfort penalty** is added to a robot's cost 
for each pedestrian whose time-indexed closest approach is below
a predetermined distance: this makes the robot *prefer* to pass wide, but it keeps
walking past a pedestrian at close range instead of stopping when a wider path isn't
available. Other soft penalities are applied in case of blocked goal or leaving the road.

**Equilibria**: `FindNash` marks rows where no single agent can unilaterally lower
its cost; `ParetoFilter` keeps the non-dominated ones. `SelectJoint` picks the
Pareto row minimizing the **sum of controlled-robot costs + a hysteresis penalty**
(§8). Falls back to Nash if Pareto is empty.

---

## 7. Execution — decoupled controller (`ControlPublish`, `PurePursuit`)

The controller runs on its own high-rate timer, independently of planning. For each
robot it looks at where the robot is now and at the most recent planned path, and
follows that path with a *pure-pursuit* rule: it aims at a point a short distance ahead
on the path and steers toward it while driving at cruise speed.

Two corrections make this reliable. First, the planned paths are jagged (they come from
a sampling-based planner), so the steering command is **smoothed** before being sent;
without this the robot jerked left and right on every tick and drifted off the very path
it was meant to follow, into collisions. Second, the speed is only **mildly reduced in
turns** (kept above about half cruise speed); an earlier, sharper slowdown made the robot
crawl through every dodge and every U-turn at a goal.

The controller publishes a **full stop** in two cases: when the robot is within the goal
tolerance (so it does not creep slowly toward the goal while the next plan is computed),
and whenever the planner has flagged the robot to stop.

---

## 8. Supporting mechanisms

- **Safety brake** — **static only**: stops if a future trajectory point comes within 
  physical distance of a lidar detected point that is **ahead**.
- **Escape mode**: triggers when the robot's own pose is inside a blocked cell
  **or** when it has been gridlocked  (e.g. two robots waiting each other).  
- **Hysteresis**: penalizes changing the initial heading vs. the previous cycle's chosen path,
  preventing left-right zig-zag indecision.

---

## 9. Tunables (road config)

| Group | Params |
|------|--------|
| RRT | `robot_v`, `w_min/w_max`, `curvature_factor`, `prop_*`, `robot_num_actions`, `rrt_solve_time`, `planning_horizon`, `integrator_dt` |
| Workspace | `ws_min/max_x/y`, `road_keep`, `road_min/max_x`, `road_penalty` |
| Collision | `agent_radius`, `dyn_collision_dist` (hard robot–ped contact), `ped_comfort_dist`/`ped_comfort_weight` (soft wide-pass preference), `robot_collision_dist` (robot–robot), `goal_block_penalty` |
| Lidar grid | `use_lidar`, `lidar_grid_res`, `lidar_inflation`, `static_margin`, `lidar_robot_filter_radius`, `ego_bubble` |
| Tracker | `cluster_cell`, `track_gate`, `track_min_hits`, `track_max_misses`, `dyn_speed_thresh`, `vel_window`, `dyn_latch_frames`, `ped_max_speed`, `ped_predict_horizon` |
| Control | `control_rate` (planning gap), `control_pub_rate`, `lookahead`, `w_ema` |
| Safety brake | `safety_brake`, `brake_lookahead`, `brake_margin`, `brake_cone` |
| Selection | `hysteresis_weight` |
| Gridlock | `dyn_stuck_limit` (cycles of `INF` before escape-to-separate) |

---

## 10. From the paper vs. built from scratch

**Taken from Turnwald & Wollherr (2019):**
- The game-theoretic decision model: each agent chooses among a finite set of candidate
  trajectories, and an outcome is scored with the paper's cost — the path length plus an
  infinite penalty whenever the trajectory collides.
- The use of **Nash equilibria** (no agent can do better by changing its action alone)
  and **Pareto optimality** (no agent can do better without making another worse) to
  pick a jointly sensible outcome.
- Both settings the paper describes, combined here: the fully-controllable joint game
  among the robots (§4.3.3), and the among-humans setting (§4.3.4) where the other agents
  are predicted. (The paper's equilibrium-memory idea was tried but removed — see §5 —
  because it locked the selection onto a stale trajectory.)
- The stand-still action and the idea of an emergency-stop safety layer.
- The discrete control set (Eq. 8) and the unicycle motion model (Eq. 7) the trajectories
  are built on.

**Built from scratch on top of the paper:**
- **Pedestrian perception from lidar.** The paper assumes the other agents are known;
  here they are discovered from the raw lidar — grouped, tracked, and recognized as
  moving from their displacement over time — with no prior knowledge of who or where
  they are.
- **Concrete trajectory generation.** The abstract "set of actions" is realized with a
  sampling-based kinodynamic planner (control-based RRT) that plans full paths to the
  goal, scored by length plus the straight-line distance still remaining.
- **Decoupled real-time architecture.** Planning and command output run on separate
  threads, with a self-pacing planning loop and a smooth high-rate path follower, so a
  slow planning cycle never stalls the robot.
- **Static safety brake.** A directional, distance-based emergency stop for static
  obstacles, kept separate from the game (which already handles the dynamic ones).
- **Deadlock and indecision handling.** An escape behaviour that makes a trapped or
  gridlocked robot move to free itself, and a hysteresis term that commits the robot to
  one side instead of oscillating left and right.
- **Lane keeping.** A soft penalty that keeps the robots on the road yet still lets them
  leave it briefly to avoid someone.
- **Multi-robot simulation plumbing.** Per-robot namespacing of the simulated robots and
  the merging of their lidar into a single shared view of the world.
- **Robustness tuning** accumulated from testing: the lidar self-filter fix, separate
  collision margins for robot-vs-robot and robot-vs-pedestrian, and removal of the
  goal-crawl behaviour.
