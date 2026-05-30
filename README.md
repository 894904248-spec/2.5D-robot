# Robot Navigation MVP

This repository is a Python-only research MVP for a cleaning-robot-like rover. It is intentionally scoped as a **2D-core / 2.5D-ready** simulator: the robot moves on a 2D grid, while every cell already carries fields that future 2.5D traversability logic can use.

The current work has two research steps:

- **v0.1.1 baseline**: 2D coverage, 4-neighbor A*, obstacle avoidance, safe-path backtracking recovery, metrics, CSV logs, and PNG visualization.
- **v0.2 cost-aware extension**: adds planner modes so a weighted 2.5D-cost A* route can be compared against the original 2D baseline.

This is still not a full robotics stack. It does not include ROS, Gazebo, MAVLink, ArduPilot, SLAM, real sensors, camera processing, deep learning, or hardware support.

## Planner Modes

`baseline_2d` keeps the v0.1.1 behavior for route choice. Blocked cells are forbidden, but all traversable cells use uniform movement cost during A*. This means a short route through carpet or low-bump cells may be selected.

`cost_2d5` keeps blocked cells forbidden and uses each cell's `traversability_cost` during A*. This lets the planner prefer a longer normal-floor route over a shorter high-cost route when the weighted cost is lower.

Both modes still use the same grid, map schema, coverage loop, recovery mechanism, output files, and tests.

## What `cost_choice_room` Demonstrates

`maps/cost_choice_room.json` contains:

- a shorter direct route through high-cost cells, and
- a longer normal-floor route around those cells.

For a point-to-point route from `(1, 3)` to `(10, 3)`:

| Mode | Route cells | Movement edges | 2.5D traversal cost |
|---|---:|---:|---:|
| `baseline_2d` | `10` | `9` | `41.0` |
| `cost_2d5` | `12` | `11` | `11.0` |

This proves the v0.2 weighted route-choice behavior: `baseline_2d` chooses the shorter high-cost corridor, while `cost_2d5` chooses the longer but safer/lower-cost route.

The optional figure `outputs/cost_choice_route_comparison.png` is a point-to-point route-choice diagnostic for this comparison. It is not a full-room coverage result.

Full-room coverage results for the same map are saved separately:

```text
outputs/cost_choice_baseline/final_map.png
outputs/cost_choice_2d5/final_map.png
```

## Point-To-Point Evidence vs Full Coverage

The route-choice evidence above is point-to-point A* evidence. The full simulator is a coverage task.

In full coverage, the robot tries to visit every reachable traversable cell. Carpet and low-bump cells are still reachable cleaning targets, not forbidden obstacles, so both planner modes may eventually enter all reachable high-cost cells to complete coverage. Therefore, v0.2 should not be described as reducing full-coverage `total_cost` yet.

Current v0.2 demonstrates cost-aware A* route choice, not full coverage-order optimization. Full coverage-order optimization is a future v0.3 topic.

## Algorithm Overview

1. Load a JSON grid map and expand rectangular semantic features into cells.
2. Compute traversability from occupancy, semantic class, height, slope, and roughness.
3. Flood-fill from the start cell to identify reachable traversable cells.
4. Generate a deterministic serpentine target order across reachable cells.
5. Move cell-by-cell using 4-neighbor A* in the selected planner mode.
6. Record every safe cell in `safe_path`.
7. When no unvisited neighbor is available, backtrack along `safe_path` to the nearest recorded branch candidate, then replan.

## Map JSON Format

Maps use a small rectangular-feature format:

```json
{
  "name": "u_trap_room",
  "width": 36,
  "height": 24,
  "start": [2, 2],
  "features": [
    {"type": "wall", "rect": [0, 0, 36, 1]},
    {"type": "carpet", "rect": [4, 4, 6, 5]},
    {"type": "low_bump", "rect": [5, 20, 20, 1]}
  ]
}
```

Each rectangle is `[x, y, width, height]`. Feature `type` maps to the cell semantic class. Optional 2.5D-ready fields may also be supplied on a feature: `height`, `slope`, `roughness`, `traversability_cost`, and `occupied`.

Blocked semantic classes are `wall`, `hard_obstacle`, `pit`, `too_high`, and `too_steep`. `carpet`, `low_bump`, and `curtain_soft` remain traversable but have higher costs than normal `floor`.

Available validation maps:

- `maps/baseline_room.json`: simple baseline room with static obstacles and soft-cost regions.
- `maps/u_trap_room.json`: U-shaped trap / recovery scenario.
- `maps/narrow_passage_room.json`: narrow-passage coverage scenario.
- `maps/cost_choice_room.json`: point-to-point route-choice demonstration for `baseline_2d` vs `cost_2d5`.
- `maps/complex_indoor_room.json`: denser indoor validation scene with furniture blocks, narrow passage, U-shaped/dead-end region, carpet, and cautious areas.

## Metrics Definition

- `coverage_rate`: visited reachable traversable cells divided by total reachable traversable cells. Blocked cells are not included.
- `visited_cells`: number of reachable traversable cells visited at least once.
- `total_reachable_cells`: flood-fill count from the start over traversable cells.
- `steps`: number of executed cell-to-cell moves.
- `collision_count`: attempted moves into blocked or non-traversable cells. This should stay `0`.
- `recovery_count`: number of safe-path backtracking recovery events.
- `unreachable_targets`: targets skipped because A* could not find a route.
- `path_length`: number of executed movement edges. In this simulator it normally matches `steps`.
- `turn_count`: number of direction changes between consecutive movement edges.
- `total_cost`: cumulative executed movement cost in the active planner mode. For `baseline_2d`, this is uniform step cost. For `cost_2d5`, this is weighted traversal cost.
- `high_cost_cells_entered`: number of movement steps into traversable cells whose 2.5D cost is above normal floor. In full coverage, this includes required visits to high-cost cells.
- `cautious_cells_entered`: number of movement steps into cells classified as `cautious`. This also counts required coverage entries.

## Recovery Strategy

The simulator records a safe path history and branch candidates around each visited cell. If the robot reaches a dead end or has no immediate unvisited neighbor, it scans the safe path backward, finds the nearest previous cell with an unvisited traversable branch, moves back along known safe cells, and replans to that branch. This is a simple baseline version of safe-path left/right branch marking, not a dynamic physical recovery controller.

## Output Files

Each run writes an output directory containing:

- `final_map.png`: main coverage summary with map semantics, visited cells, path, dock/start, final_pose, planner mode, and key metrics.
- `trajectory_order_map.png`: execution-order view using a step-color gradient and sparse arrows.
- `recovery_debug_map.png`: recovery-focused view that highlights backtracking and branch-recovery segments.
- `path_log.csv`: per-step movement log with position, action, target, cell metadata, step cost, cumulative cost, planner mode, coverage, recovery id, and reason.

The terminal also prints the metrics above.

`final_map.png` is the best quick-look artifact. `trajectory_order_map.png` is for understanding when cells were visited. `recovery_debug_map.png` is for checking whether backtracking/recovery events occurred and where. v0.2.3 phase 1 does not generate `trajectory.gif` yet.

## Run

Install and run tests:

```powershell
python -m pip install -r requirements.txt
python -m pip install -e .
python -m pytest -q
```

Run the original v0.1.1-style baseline behavior:

```powershell
python examples/run_mvp.py --map maps/u_trap_room.json --output outputs/u_trap --planner-mode baseline_2d
```

Run the v0.2 cost-choice comparison:

```powershell
python examples/run_mvp.py --map maps/cost_choice_room.json --output outputs/cost_choice_baseline --planner-mode baseline_2d
python examples/run_mvp.py --map maps/cost_choice_room.json --output outputs/cost_choice_2d5 --planner-mode cost_2d5
```

Run the v0.2.2 complex indoor validation:

```powershell
python examples/run_mvp.py --map maps/complex_indoor_room.json --output outputs/complex_indoor_baseline --planner-mode baseline_2d
python examples/run_mvp.py --map maps/complex_indoor_room.json --output outputs/complex_indoor_2d5 --planner-mode cost_2d5
```

## Limitations

- v0.2 is not full coverage-order optimization. The coverage target order is still mostly the v0.1.1 deterministic coverage behavior. Full coverage-order optimization is a future v0.3 topic.
- Planning is still grid-based with 4-neighbor connectivity.
- Costs are simulated rule-based semantic costs, not real perception-derived terrain estimates.
- Obstacles are static rectangles loaded from JSON.
- Recovery is a planner-level safe-path backtrack, not a real vehicle dynamics maneuver.
- No real perception, localization, SLAM, ROS, Gazebo, camera processing, deep learning, or hardware integration is included.

## Future 2.5D Extension

The next research step should keep v0.1.1 and v0.2 as baselines, then add a planner/coverage strategy that optimizes coverage order with 2.5D costs rather than only point-to-point A* route choice. Later stages can replace the simulated cost rules with LiDAR/camera-derived height, slope, roughness, and semantic estimates without changing the public map API.
