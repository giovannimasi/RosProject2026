# Game-Theoretic Motion Planner — Documentation

Implementation of **Turnwald & Wollherr (2019), "Human-Like Motion Planning Based
on Game-Theoretic Decision Making"** on ROS 1 Noetic + Gazebo (Docker), for
TurtleBot3 Waffle Pi robots sharing space with simulated pedestrians.

The planner lives in the `gt_motion_planner` package (`src/gt_planner_node.cpp`,
C++). The reference paper converted to markdown is in [`paper.md`](paper.md).

## Documents

| File | Content |
|------|---------|
| [ARCHITECTURE.md](ARCHITECTURE.md) | How the **current** code works: features, structure, the algorithm pipeline, every tunable, and what comes from the paper vs. what was built from scratch. Kept in sync with the code. |
| [EVOLUTION.md](EVOLUTION.md) | The project history: single-agent → multi-agent → road scenario, with the problems hit, the diagnoses, and the fixes/features added at each step. |

## Scenarios at a glance

| Scenario | Sim launch | Planner launch + config | World |
|----------|-----------|--------------------------|-------|
| Single agent among actors | `game_theory_sim game_theory.launch` | `gt_motion_planner gt_planner.launch` | `actor_collisions.world` |
| Multi-robot (joint game) | `game_theory_sim game_theory_multi.launch` | `gt_motion_planner gt_planner_multi.launch` | `two_robots.world` |
| Road (final) | `game_theory_sim game_theory_road.launch` | `gt_motion_planner gt_planner_road.launch` | `road.world` |
