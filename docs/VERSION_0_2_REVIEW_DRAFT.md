# v0.2 Review Draft

This review package summarizes the current v0.2 working tree before any v0.2.1 hardening or documentation polish.

## 1. Git Status

Current branch:

```text
main
```

Changed tracked files:

```text
examples/run_mvp.py
outputs/baseline/final_map.png
outputs/baseline/path_log.csv
outputs/narrow_passage/final_map.png
outputs/narrow_passage/path_log.csv
outputs/u_trap/final_map.png
outputs/u_trap/path_log.csv
src/robot_nav_sim/map2d5.py
src/robot_nav_sim/metrics.py
src/robot_nav_sim/planner.py
src/robot_nav_sim/simulator.py
src/robot_nav_sim/visualization.py
tests/test_outputs.py
tests/test_planner.py
tests/test_simulator.py
```

Untracked files/directories:

```text
maps/cost_choice_room.json
outputs/cost_choice_2d5/
outputs/cost_choice_baseline/
docs/VERSION_0_2_REVIEW_DRAFT.md
```

Cache ignore status:

- `.pytest_cache/` is ignored by `.gitignore`.
- `.coverage` is ignored by `.gitignore`.
- `__pycache__/` directories are ignored by `.gitignore`.
- `htmlcov/` and `cov_annotate/` are listed in `.gitignore`, though they are not currently present.

## 2. v0.2 Implementation Summary

v0.2 upgrades the v0.1.1 baseline into a small 2.5D-cost-aware simulation while keeping the project Python-only and local. It does not add ROS, Gazebo, SLAM, sensors, deep learning, camera processing, or hardware support.

`baseline_2d` ignores soft traversal cost during A* planning. Blocked cells remain forbidden, but traversable floor, carpet, and low-bump cells all count as the same movement cost for route choice.

`cost_2d5` keeps blocked cells forbidden and uses each traversable cell's `traversability_cost` during A*. This lets the planner prefer a longer normal-floor route over a shorter high-cost route when that is cheaper overall.

Main changes from v0.1.1:

- Added planner modes: `baseline_2d` and `cost_2d5`.
- Added traversability classes: `free`, `cautious`, and `blocked`.
- Added weighted A* behavior for `cost_2d5`.
- Added `cost_choice_room.json` to demonstrate route choice.
- Added metrics: `total_cost`, `high_cost_cells_entered`, and `cautious_cells_entered`.
- Extended `path_log.csv` with planner mode and cost fields.
- Updated visualization colors/legend for normal, cautious/high-cost, and blocked cells.
- Added tests for cost-aware route choice and compatibility with previous maps.

Modified implementation/test files:

```text
examples/run_mvp.py
maps/cost_choice_room.json
src/robot_nav_sim/map2d5.py
src/robot_nav_sim/metrics.py
src/robot_nav_sim/planner.py
src/robot_nav_sim/simulator.py
src/robot_nav_sim/visualization.py
tests/test_outputs.py
tests/test_planner.py
tests/test_simulator.py
```

## 3. Route-Choice Evidence

Map: `maps/cost_choice_room.json`

Start: `(1, 3)`

Target: `(10, 3)`

`baseline_2d` A* route:

```text
[(1, 3), (2, 3), (3, 3), (4, 3), (5, 3), (6, 3), (7, 3), (8, 3), (9, 3), (10, 3)]
```

`cost_2d5` A* route:

```text
[(1, 3), (1, 4), (2, 4), (3, 4), (4, 4), (5, 4), (6, 4), (7, 4), (8, 4), (9, 4), (10, 4), (10, 3)]
```

Route comparison:

| Mode | Route cells | Movement edges | 2.5D traversal cost |
|---|---:|---:|---:|
| `baseline_2d` | `10` | `9` | `41.0` |
| `cost_2d5` | `12` | `11` | `11.0` |

This proves the cost-aware route choice works for point-to-point planning: `baseline_2d` takes the shorter direct route through high-cost carpet, while `cost_2d5` chooses the longer normal-floor route because its weighted traversal cost is much lower.

## 4. Full-Coverage Result Explanation

The full simulator is a coverage task, not only a point-to-point route choice task. In full coverage mode, the robot attempts to visit every reachable traversable cell. Since high-cost carpet and low-bump cells are still traversable, both planner modes eventually enter those cells to complete coverage.

That means `high_cost_cells_entered` and `cautious_cells_entered` can be the same in full-coverage runs even when point-to-point A* behavior differs. The v0.2 cost-aware behavior is best evaluated on the route-choice segment from start to target in `cost_choice_room.json`; full coverage confirms that all reachable cells remain coverable without collisions.

## 5. Output File Locations

Cost-choice outputs:

```text
D:\pytorch\2.5D-robot-github\outputs\cost_choice_baseline\final_map.png
D:\pytorch\2.5D-robot-github\outputs\cost_choice_baseline\path_log.csv
D:\pytorch\2.5D-robot-github\outputs\cost_choice_2d5\final_map.png
D:\pytorch\2.5D-robot-github\outputs\cost_choice_2d5\path_log.csv
```

Previous validation outputs:

```text
D:\pytorch\2.5D-robot-github\outputs\baseline\final_map.png
D:\pytorch\2.5D-robot-github\outputs\baseline\path_log.csv
D:\pytorch\2.5D-robot-github\outputs\u_trap\final_map.png
D:\pytorch\2.5D-robot-github\outputs\u_trap\path_log.csv
D:\pytorch\2.5D-robot-github\outputs\narrow_passage\final_map.png
D:\pytorch\2.5D-robot-github\outputs\narrow_passage\path_log.csv
```

## 6. Metrics Table

| Case | Mode | coverage_rate | collision_count | recovery_count | path_length | turn_count | total_cost | high_cost_cells_entered | cautious_cells_entered |
|---|---|---:|---:|---:|---:|---:|---:|---:|---:|
| `cost_choice_room` | `baseline_2d` | `1.0` | `0` | `0` | `49` | `9` | `49.0` | `9` | `9` |
| `cost_choice_room` | `cost_2d5` | `1.0` | `0` | `0` | `49` | `9` | `82.0` | `9` | `9` |
| `baseline_room` | `baseline_2d` | `1.0` | `0` | `0` | `479` | `52` | `479.0` | `50` | `10` |
| `u_trap_room` | `baseline_2d` | `1.0` | `0` | `3` | `1170` | `187` | `1170.0` | `111` | `63` |
| `narrow_passage_room` | `baseline_2d` | `1.0` | `0` | `2` | `567` | `159` | `567.0` | `36` | `16` |

## 7. Manual Inspection Checklist

Open these files visually:

1. `D:\pytorch\2.5D-robot-github\outputs\cost_choice_baseline\final_map.png`
2. `D:\pytorch\2.5D-robot-github\outputs\cost_choice_2d5\final_map.png`
3. `D:\pytorch\2.5D-robot-github\outputs\baseline\final_map.png`
4. `D:\pytorch\2.5D-robot-github\outputs\u_trap\final_map.png`
5. `D:\pytorch\2.5D-robot-github\outputs\narrow_passage\final_map.png`

Check that:

- Blocked cells are dark and never crossed by the red path.
- Cautious/high-cost cells are visually distinct from normal floor.
- Start and end markers are visible.
- The cost-choice map shows the intended high-cost corridor.
- The previous maps still look like valid full-coverage runs.

Open these CSV files:

1. `D:\pytorch\2.5D-robot-github\outputs\cost_choice_baseline\path_log.csv`
2. `D:\pytorch\2.5D-robot-github\outputs\cost_choice_2d5\path_log.csv`

Check that:

- `planner_mode` is populated.
- `step_cost` and `cumulative_cost` are populated.
- `traversability_class`, `height`, `slope`, and `roughness` are present.
- High-cost/cautious cells have higher `cost` values.

## 8. Risks / Issues Before v0.2.1

- The full-coverage metrics can be misleading: both modes eventually enter high-cost cells because coverage requires visiting all reachable traversable cells. This should be clarified in README/docs before calling v0.2 polished.
- `total_cost` currently reflects the active planner mode. For `baseline_2d`, it is uniform step cost, not a 2.5D risk/cost score. This distinction needs documentation.
- The route-choice evidence is point-to-point, while the CLI demo runs full coverage. v0.2.1 docs should explicitly separate those two evaluation modes.
- `cost_choice_room.json` proves weighted A* behavior, but the generated full-coverage PNGs for both modes may look very similar because every reachable cell is eventually covered.
- Pytest passes, but pytest emits a cache permission warning in this local environment because `.pytest_cache` cannot be written.
- README and baseline docs still describe v0.1.1 and need v0.2 updates before release.
