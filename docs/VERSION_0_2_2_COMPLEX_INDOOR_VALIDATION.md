# v0.2.2 Complex Indoor Validation

## Purpose

This validation adds `maps/complex_indoor_room.json` as a more realistic indoor cleaning stress test for the existing v0.2 planner modes. It is a validation map, not a new algorithm.

## Map Features

The map includes:

- room boundary walls,
- table-leg-like obstacle blocks,
- larger furniture blocks,
- one narrow passage,
- one U-shaped / dead-end region,
- one carpet high-cost area,
- low-bump and curtain-soft cautious areas,
- normal floor cells.

## Metrics

| Mode | coverage_rate | collision_count | recovery_count | unreachable_targets | path_length | turn_count | total_cost | high_cost_cells_entered | cautious_cells_entered |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| `baseline_2d` | `1.0` | `0` | `3` | `0` | `1112` | `202` | `1112.0` | `127` | `127` |
| `cost_2d5` | `1.0` | `0` | `3` | `0` | `1112` | `202` | `1288.0` | `127` | `127` |

## Interpretation

This map is intended to check that v0.2 remains stable on a denser indoor layout. It does not add coverage-order optimization or a new recovery algorithm.

During full coverage, both planner modes may enter high-cost cells because carpet, low-bump, and curtain-soft cells are reachable cleaning targets, not forbidden obstacles. v0.2 still demonstrates cost-aware A* route choice; full coverage-order optimization remains a future v0.3 topic.
