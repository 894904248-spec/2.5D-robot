# v0.2 Cost-Aware Simulation

## v0.2 Goal

v0.2 extends the v0.1.1 2D-core / 2.5D-ready baseline with a cost-aware planning mode. The goal is to demonstrate that the same grid-map API can support weighted route choice using simulated 2.5D traversability costs.

This is still a local Python simulation. It does not add ROS, Gazebo, SLAM, real LiDAR, camera processing, deep learning, sensors, or hardware support.

## Difference From v0.1.1

v0.1.1 provided:

- 2D grid coverage.
- 4-neighbor A*.
- Static obstacle avoidance.
- Safe-path backtracking recovery.
- CSV logs, metrics, and PNG visualization.
- A 2.5D-ready map schema, but not 2.5D-cost-aware route choice.

v0.2 adds:

- `baseline_2d` planner mode: blocked cells are forbidden, but traversable cells have uniform A* route cost.
- `cost_2d5` planner mode: blocked cells are forbidden, and traversable cells use `traversability_cost` during A*.
- Traversability classes: `free`, `cautious`, and `blocked`.
- Metrics: `total_cost`, `high_cost_cells_entered`, and `cautious_cells_entered`.
- Extended `path_log.csv` fields for cost and planner mode.
- A new `cost_choice_room.json` map to demonstrate route-choice differences.

## Changed Files

Implementation and tests:

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

Documentation:

```text
README.md
docs/VERSION_0_2_COST_AWARE.md
docs/VERSION_0_2_REVIEW_DRAFT.md
```

Generated validation outputs:

```text
outputs/cost_choice_baseline/
outputs/cost_choice_2d5/
outputs/baseline/
outputs/u_trap/
outputs/narrow_passage/
```

## New Map: `cost_choice_room.json`

`maps/cost_choice_room.json` is designed to make route choice visible. It contains:

- a short direct corridor through high-cost carpet cells, and
- a longer normal-floor route around that corridor.

The high-cost corridor remains traversable. It is not an obstacle. This is important because v0.2 is testing cost-aware route choice, not binary obstacle avoidance.

## Validation Result

Latest validation:

```text
13 passed, 1 warning
```

The warning is a local pytest cache permission warning:

```text
could not create cache path ... .pytest_cache ...
```

It is not a test failure.

## Route-Choice Comparison

Point-to-point target:

```text
start = (1, 3)
target = (10, 3)
```

`baseline_2d` route:

```text
[(1, 3), (2, 3), (3, 3), (4, 3), (5, 3), (6, 3), (7, 3), (8, 3), (9, 3), (10, 3)]
```

`cost_2d5` route:

```text
[(1, 3), (1, 4), (2, 4), (3, 4), (4, 4), (5, 4), (6, 4), (7, 4), (8, 4), (9, 4), (10, 4), (10, 3)]
```

| Mode | Route cells | Movement edges | 2.5D traversal cost |
|---|---:|---:|---:|
| `baseline_2d` | `10` | `9` | `41.0` |
| `cost_2d5` | `12` | `11` | `11.0` |

This demonstrates weighted route choice: `baseline_2d` chooses the shorter high-cost corridor, while `cost_2d5` chooses a longer route with lower total 2.5D traversal cost.

The optional image below is a point-to-point route-choice diagnostic, not a full-room coverage result:

```text
outputs/cost_choice_route_comparison.png
```

Full-room coverage results for the cost-choice map are saved separately:

```text
outputs/cost_choice_baseline/final_map.png
outputs/cost_choice_2d5/final_map.png
```

## Full Coverage Clarification

The full simulator is a coverage task. It tries to visit every reachable traversable cell.

Because high-cost cells are still traversable cleaning targets, not forbidden obstacles, both modes may eventually visit them during full coverage. Therefore:

- v0.2 demonstrates weighted point-to-point route choice.
- v0.2 does not yet claim to reduce total full-coverage cost.
- `high_cost_cells_entered` counts all entries into high-cost cells, including required visits for coverage.
- `cautious_cells_entered` also counts all required coverage entries into cautious cells.

This distinction matters when interpreting `total_cost`. In `baseline_2d`, `total_cost` is uniform step cost. In `cost_2d5`, `total_cost` is weighted traversal cost. These values are useful for mode-specific accounting, but they should not yet be treated as proof of optimized full-coverage ordering.

Full coverage-order optimization is a future v0.3 topic.

## Risks / Limitations

- Full-coverage output images for `baseline_2d` and `cost_2d5` can look similar because both modes eventually cover all reachable cells.
- The strongest v0.2 evidence is the point-to-point route comparison on `cost_choice_room.json`.
- Coverage target ordering is not yet optimized by 2.5D cost.
- The cost model is simulated and semantic/rule-based.
- Movement remains 4-neighbor grid movement.
- Recovery remains a planner-level backtracking strategy, not a physical vehicle maneuver.

## Next Step

The next step should be v0.3 cost-aware coverage-order optimization. That work should compare:

- v0.1.1 2D baseline coverage,
- v0.2 weighted point-to-point A* route choice, and
- a future planner that reduces full-coverage cost while still maintaining high coverage and zero collisions.
