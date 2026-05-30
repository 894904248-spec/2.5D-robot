# Robot Navigation MVP

This is v0.1 of a minimal Python simulation for a cleaning-robot-like rover. It is intentionally a **2D-core / 2.5D-ready** MVP: planning runs on a 2D grid, while every cell already exposes fields that future 2.5D traversability logic can use.

This is not a full 2.5D planner yet. It is a reproducible research baseline for validating the control loop: reachable-cell coverage, A* navigation, obstacle avoidance, safe-path recovery, metrics, and visualization.

## Scope

- 2D grid map loaded from JSON.
- 2.5D-ready cell schema: occupancy, height, slope, roughness, semantic class, traversability cost.
- 4-neighbor A* path planning.
- Reachable-cell coverage planning.
- Safe-path backtracking recovery from dead ends.
- Terminal metrics, CSV path log, and final PNG visualization.

This project does not include ROS, Gazebo, MAVLink, ArduPilot, SLAM, deep learning, camera processing, or hardware support.

## Algorithm Overview

1. Load a JSON grid map and expand rectangular semantic features into cells.
2. Compute traversability from occupancy, semantic class, height, slope, and roughness.
3. Flood-fill from the start cell to identify reachable traversable cells.
4. Generate a deterministic serpentine target order across reachable cells.
5. Move cell-by-cell using 4-neighbor A* between the current position and the next target or recovery branch.
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

## Metrics Definition

- `coverage_rate`: visited reachable traversable cells divided by total reachable traversable cells. Blocked cells are not included.
- `visited_cells`: number of reachable traversable cells visited at least once.
- `total_reachable_cells`: flood-fill count from the start over traversable cells.
- `steps`: number of executed cell-to-cell moves.
- `collision_count`: attempted moves into blocked or non-traversable cells. This should stay `0`.
- `recovery_count`: number of safe-path backtracking recovery events.
- `unreachable_targets`: targets skipped because A* could not find a route.
- `path_length`: number of executed movement edges. In v0.1 this matches `steps`.
- `turn_count`: number of direction changes between consecutive movement edges.

## Recovery Strategy

The simulator records a safe path history and branch candidates around each visited cell. If the robot reaches a dead end or has no immediate unvisited neighbor, it scans the safe path backward, finds the nearest previous cell with an unvisited traversable branch, moves back along known safe cells, and replans to that branch. This is a simple baseline version of safe-path left/right branch marking, not a dynamic physical recovery controller.

## Output Files

Each run writes an output directory containing:

- `final_map.png`: final visualization with map semantics, visited cells, path, start, and end.
- `path_log.csv`: per-step movement log with position, action, target, cell metadata, cost, coverage, recovery id, and reason.

The terminal also prints the metrics above.

## Run

```powershell
python -m pip install -r requirements.txt
python -m pip install -e .
python examples/run_mvp.py --map maps/u_trap_room.json --output outputs/u_trap
python -m pytest -q
```

The demo writes:

- `outputs/u_trap/final_map.png`
- `outputs/u_trap/path_log.csv`

## Limitations

- Planning is 2D and grid-based.
- Movement is cell-to-cell with 4-neighbor connectivity.
- Costs are simple rule-based semantic costs.
- Obstacles are static rectangles loaded from JSON.
- Recovery is a planner-level safe-path backtrack, not a real vehicle dynamics maneuver.
- No real perception, localization, SLAM, ROS, Gazebo, camera processing, or hardware integration is included.

## Future 2.5D Extension

The first version uses simple rule-based traversability. Later work can replace `GridMap2D5.is_traversable()` and `GridMap2D5.cost()` with LiDAR/camera-derived height, slope, roughness, and semantic estimates without changing the public planner or simulator API. A future planner can compare a pure 2D baseline, 2D-core with semantic cost rules, and a true 2.5D traversability-cost planner.
