# v0.1.1 Baseline

This document freezes the current v0.1.1 research baseline before starting v0.2 work.

## MVP Scope

v0.1.1 is a Python-only **2D-core / 2.5D-ready** cleaning robot navigation MVP. It demonstrates a reproducible baseline for:

- Grid-map loading from JSON.
- 2.5D-ready cell metadata: occupancy, height, slope, roughness, semantic class, and traversability cost.
- 4-neighbor A* path planning.
- Reachable-cell coverage.
- Obstacle avoidance by refusing blocked-cell movement.
- Safe-path backtracking recovery from dead ends or missed branches.
- Terminal metrics, CSV path logging, and final PNG visualization.

It is not a full 2.5D planner yet. The planner still operates on a 2D grid with simple rule-based traversability.

## Project Structure

```text
robot_nav_mvp/
  README.md
  requirements.txt
  pyproject.toml
  docs/
    VERSION_0_1_1_BASELINE.md
  examples/
    run_mvp.py
  maps/
    baseline_room.json
    u_trap_room.json
    narrow_passage_room.json
  outputs/
    baseline/
    u_trap/
    narrow_passage/
  src/
    robot_nav_sim/
      __init__.py
      map2d5.py
      metrics.py
      planner.py
      robot.py
      simulator.py
      visualization.py
  tests/
    test_outputs.py
    test_planner.py
    test_simulator.py
```

## Implemented Modules

- `map2d5.py`: `Cell` schema, JSON loading, rectangular feature expansion, traversability rules, cost rules, 4-neighbor lookup, reachable-cell flood fill, and serpentine target ordering.
- `planner.py`: deterministic 4-neighbor A* with Manhattan distance.
- `robot.py`: robot state, visited cells, and safe path history.
- `simulator.py`: coverage loop, A* movement execution, branch candidate tracking, safe-path backtracking recovery, metrics, and enhanced CSV path log output.
- `metrics.py`: simulation metric dataclass and dictionary export.
- `visualization.py`: headless matplotlib PNG output with clearer legend entries.
- `examples/run_mvp.py`: CLI entry point for running one map and writing outputs.

## Map Validation

Validation was run on all included maps after v0.1.1 hardening.

| Map | coverage_rate | collision_count | recovery_count | unreachable_targets |
|---|---:|---:|---:|---:|
| `baseline_room` | `1.0` | `0` | `0` | `0` |
| `u_trap_room` | `1.0` | `0` | `3` | `0` |
| `narrow_passage_room` | `1.0` | `0` | `2` | `0` |

## Pytest Result

```text
9 passed in 1.04s
```

## Coverage Result

```text
TOTAL coverage: 91%
```

Known remaining uncovered lines are mainly defensive or edge-case branches in map loading and simulator failure paths. v0.1.1 does not chase 100% coverage with artificial tests.

## Output File Locations

```text
D:\pytorch\codex_robot_mvp_context\robot_nav_mvp\outputs\baseline\final_map.png
D:\pytorch\codex_robot_mvp_context\robot_nav_mvp\outputs\baseline\path_log.csv
D:\pytorch\codex_robot_mvp_context\robot_nav_mvp\outputs\u_trap\final_map.png
D:\pytorch\codex_robot_mvp_context\robot_nav_mvp\outputs\u_trap\path_log.csv
D:\pytorch\codex_robot_mvp_context\robot_nav_mvp\outputs\narrow_passage\final_map.png
D:\pytorch\codex_robot_mvp_context\robot_nav_mvp\outputs\narrow_passage\path_log.csv
```

Current generated file sizes:

| File | Size |
|---|---:|
| `outputs/baseline/final_map.png` | `62524` bytes |
| `outputs/baseline/path_log.csv` | `38463` bytes |
| `outputs/u_trap/final_map.png` | `76132` bytes |
| `outputs/u_trap/path_log.csv` | `111025` bytes |
| `outputs/narrow_passage/final_map.png` | `71975` bytes |
| `outputs/narrow_passage/path_log.csv` | `48081` bytes |

## Known Limitations

- Planning is still 2D grid planning, not full 2.5D traversability-aware planning.
- Traversability and costs are simple semantic/rule-based values.
- Obstacles are static JSON rectangles.
- Recovery is a planner-level safe-path backtracking behavior, not a physical vehicle controller.
- No real localization, SLAM, perception, sensors, camera processing, deep learning, ROS, Gazebo, ArduPilot, MAVLink, or hardware integration is included.
- Movement is cell-by-cell with 4-neighbor connectivity.

## Next Planned Step

v0.2 should add a **cost-aware 2.5D simulation layer** while keeping the current v0.1.1 baseline available for comparison. The next step should focus on simulated height, slope, roughness, and semantic cost effects in planning, not on ROS/Gazebo/hardware integration.
