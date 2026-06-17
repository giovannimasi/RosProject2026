# Project Evolution

How the planner grew from a single-agent prototype to the multi-robot road
scenario, with the problems hit and how each was diagnosed and fixed. Read
[ARCHITECTURE.md](ARCHITECTURE.md) first for the current design.

---

## Phase 0 — Paper to code

The reference paper was converted to markdown (`paper.md`, formulas preserved) and
the decision was locked: implement in **C++** extending `gt_planner_node.cpp`, with
**OMPL control-based RRT** as the action generator, **RViz** visualization of all
evaluated trajectories (chosen one highlighted), and per-path cost logging.

---

## Phase 1 — Single agent among actors (`gt_planner`)

**Starting point / scope.** Paper §4.3.4: one TurtleBot among autonomous,
*predicted* actors. Full pipeline built: RRT action sets → cost (`length` +
`∞` on collision) → Nash → Pareto → equilibrium selection → memorize `ε*`. Actors
predicted by constant velocity; group actors handled via a "group corridor" the
robot must never cross. RViz markers + terminal cost logging.

**Problems → diagnosis → fix**

| Problem | Diagnosis | Fix |
|--------|-----------|-----|
| All RRT actions had identical cost (~0.44) | Cost was the truncated horizon length only | Cost = traveled + **remaining-to-goal** estimate |
| "Invalid bounds", robot walked off and stopped | Robot left the ±3.5 workspace | Widen workspace to ±5, clamp start/goal into bounds |
| `libompl.so` missing after container restart | Live `apt` install lost | Added `ros-noetic-ompl` to the Dockerfile |
| Robot ignored obstacles added at runtime | Only file obstacles known | Autodetect from `model_states` (later removed for lidar-only) |
| Robot drove straight into an obstacle | Short *memorized* action + optimistic straight-line remaining looked cheap | `goal_block_penalty` when end→goal line is blocked; memorized re-offers the full stored trajectory |
| RRT "invalid start" when robot sat in an inflated zone | Start state blocked | **Ego-bubble**: free a disc around the current pose |
| Ego-bubble made the robot creep *into* obstacles | Bubble always on = hole in the obstacle | Make ego-bubble **conditional** — only while already trapped (escape) |
| Escape pointed toward the obstacle / stalled | Escape had no good objective | Escape cost = **−clearance** of the endpoint |
| Left/right zig-zag indecision | First→last direction identical on both sides | **Hysteresis** on the *initial* heading (`EarlyDir`) |
| "Why isn't the whole path to the goal shown?" | Horizon truncation | **Full-to-goal** RRT (cost keeps remaining term) |
| Box obstacles snagged corners (treated as cylinders) | Shape mismatch; object-based only | Treat **lidar points** as obstacles directly, *in addition to* objects |

Outcome: a faithful single-robot planner that avoids static (object + lidar) and
dynamic agents, plans full-to-goal, recovers via escape, and doesn't zig-zag.

---

## Phase 2 — Multi-robot, centralized joint game (`gt_planner_multi`)

**Starting point.** Generalized the node from one robot to a list of
`ControlledRobot`s, each with its own goals and per-robot planning context (escape,
hysteresis, memorized). One **centralized joint game** (§4.3.3): all robots are
players, joint Nash/Pareto, `SelectJoint` picks `ε*` minimizing the sum of
controlled costs + per-robot hysteresis. Two TurtleBots with crossing goals. New
files: `two_robots.world`, namespaced spawns, and `gen_ns_urdf.sh` which injects
`<robotNamespace>` into the stock TurtleBot3 diff-drive + laser plugins so topics
become `/tbN/cmd_vel` and `/tbN/scan` (stock URDF doesn't namespace them).
Per-robot `/scan` is merged into **one shared occupancy grid** via each robot's
world pose.

**Problems → diagnosis → fix**

| Problem | Diagnosis | Fix |
|--------|-----------|-----|
| Bots followed short horizon arcs instead of full paths | Fallback arcs competed with (and beat) full-to-goal RRT | Arcs added **only when RRT is empty**; memorized re-offers the stored trajectory |
| Bot froze with `cmd=0` forever | Memorized replayed a *stopped* previous action | **Skip memorized** if the previous command was a stop |
| Robot drove into a close obstacle that the lidar clearly saw | Self-filter removed scan points within 0.6 m of **any** robot — including the robot's **own** body — so close obstacles vanished from the grid | Never filter around a robot's **own** centre; split into `lidar_robot_filter_radius` (other robots) vs actor filter |
| n=2 stalls / escape deadlock | Clearance ignored the other agents | Include other robots/actors in clearance; fallback arcs to guarantee motion |
| Both robots published on `/cmd_vel`, `/scan` | Stock plugins not namespaced | `gen_ns_urdf.sh` |

Also in this phase: the **emergency stop** safety mechanism was added, and obstacle
knowledge was made **lidar-only** (no preconfigured obstacles — even world objects
are detected only via lidar). An early brake bug (stopping when merely passing
*beside* an obstacle) was fixed by making the static brake **directional**
(only brake for obstacles ahead, within a cone) and **physical-distance** based
(not the inflated planning grid).

---

## Phase 3 — Road scenario (`gt_planner_road`)

**Starting point.** A narrow "road" along Y: two TurtleBots travelling in opposite
directions, several pedestrians (Gazebo `<actor>` with `ActorCollisionsPlugin` so
the lidar sees them) walking the same two directions slowly, and — initially — a
couple of thin lampposts. **Crucial new requirement:** the robot must *not* know the
actors by name; it must detect a moving obstacle from lidar, estimate its heading
and speed, and predict it. This drove the whole **perception pipeline**
(cluster → track → classify → predict) replacing the name-based `model_states`
actor lookup.

**Simulation/world problems**

| Problem | Diagnosis | Fix |
|--------|-----------|-----|
| Actors invisible to lidar | Spawned at `z=1.0` → body above the lidar plane (~0.18 m); also wrong plugin filename | Spawn actors at **z=0** (feet on ground, shins cross the lidar); correct `libActorCollisionsPlugin.so` name + real collider scaling names |
| Actors lying flat | `roll=1.5708` in scripted poses tipped them over | `roll=0` (scripted-trajectory actors stand at z=0) |
| One actor stuck in a T-pose at world centre | `delay_start` holds an un-started actor at the origin | All `delay_start=0`; stagger by start position instead |
| Lampposts caused cluster-merge stalls | A pedestrian passing a lamppost fused their clusters | **Removed the lampposts** (not needed) |

**Algorithm problems**

| Problem | Diagnosis | Fix |
|--------|-----------|-----|
| Bot "frozen" on far goals; stale highlighted path | One synchronous timer ran ~24 full-to-goal RRT solves/cycle (~0.6 s on far goals, where `solve()` burns its full budget) → no fresh `cmd`; diff-drive held the last `(v,w)` | **One-shot self-rearming** planning timer (never overlaps, auto-paces) + **decoupled high-rate controller** following the last path on its own thread (`AsyncSpinner`) |
| (considered) straight-line "carrot" subgoal to speed up far goals | Would aim into obstacles | Rejected; kept RRT aimed at the true goal |
| Bot stopped whenever an actor was merely *near* (even passing beside) | Brake treated the pedestrian as a static disc at its current position; and the game collision threshold `2·agent_radius+margin = 0.95 m` was un-clearable on a 3.6 m road | Pedestrian avoidance made **game-driven** (stop only if the *selected* action's cost is `INF` = every path collides); static brake demoted to static-only; dedicated, smaller `dyn_collision_dist` |
| Bot crept slowly past the goal | Controller pursued the "slowing near goal" path until the next plan | **Hard stop within `goal_tolerance`** |
| Pedestrian velocity wildly noisy; prediction wobbled; head-on detected too late | Frame-to-frame velocity dominated by leg swing; tracks churned; classification flickered static↔dynamic | Velocity over a **time window** + **outlier rejection**; larger `cluster_cell` (merge legs); **sticky `ever_dynamic`**; **longer prediction horizon** so head-on is seen early |
| Deterministic "dodge" arcs caused left-right wobble and didn't avoid | Equal-cost arcs flipped each cycle; doubled the joint space | **Removed** the dodge arcs |
| Bot hesitated / crawled when dodging or doing a goal U-turn; long "uncontrolled" stretch after a goal | Pure-pursuit slowed to 0.3·speed on any sharp turn (≈0.07 m/s) | **Gentle** turn slowdown (floored at 0.55·speed) |
| A pedestrian not detected (no prediction) → got hit | `track_gate` too tight → association broke before the velocity window matured → never `ever_dynamic` | Loosen `track_gate` (still below pedestrian spacing) |
| Collisions despite a "finite" (safe) planned cost — with both robots and pedestrians | Jagged RRT paths made pure-pursuit jitter the steering ±0.2 rad/s/tick → the robot drifted off its plan, turning planned `rr=0.64` into actual `rr=0.26` | **Steering smoothing** (`w_ema`) + **larger robot–robot buffer** (`robot_collision_dist`) to survive residual drift |

**Diagnostics added along the way** (so failures are readable from the log): per-ped
`@(x,y) v=(vx,vy) |v|`; per-robot `cost`, `ca=dist@time` (closest approach to the
nearest pedestrian) and `rr=dist@time` (to the other robot); and the controller's
actual `pub=v/w` with `STOP/ATGOAL` flags — which is how the robot-robot collision
and the "uncontrolled after goal" issues were pinned down.

---

## Known limitations / open items

- **Robot–pedestrian gridlock**: a robot correctly stops to let a pedestrian pass,
  but if the actor then walks into the stopped robot they can wedge. A dynamic
  escape/separation nudge is not yet implemented.
- Execution still tracks pure-pursuit *geometry*, not the plan's *time*
  parameterization; under heavy load the two can drift. A time-parameterized
  follower is the next robustness step if needed.
- Actor direction changes at the road ends are a quirk of the actor controller, not
  the planner.
