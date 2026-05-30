# v0.2.3 Visualization Design Note

This note sketches visualization improvements for the cleaning robot simulation. It does not propose new planning algorithms, robotics middleware, real sensors, SLAM, Gazebo, ROS, ArduPilot, Mission Planner, or hardware support.

## 1. Clearer Plot Title

The current `final_map.png` title is simple. v0.2.3 should make each plot self-describing by including:

- `map_name`
- `planner_mode`
- `task_type`
- `coverage_rate`
- `collision_count`
- `recovery_count`

Example:

```text
complex_indoor_room | planner=cost_2d5 | task=full_coverage | coverage=1.000 | collisions=0 | recoveries=3
```

This helps screenshots stand alone in reports and slide decks.

## 2. Dock / Start / Final Pose / Return To Dock

The current plot marks `start` and `end`. For a cleaning-robot framing, v0.2.3 should rename or add labels:

- `dock/start`: the initial pose, treated as the dock location in simulation.
- `final_pose`: the final robot pose after coverage completes.
- `return_to_dock`: optional future task state if a return route is implemented.

For v0.2.3, do not implement return-to-dock logic unless it already exists. The visualization can reserve the naming convention without changing behavior.

## 3. `trajectory_order_map.png`

Add an optional second PNG showing trajectory order more clearly than a single red path line.

Two simple approaches:

- Sparse arrows every N steps, e.g. every 20 or 40 path points.
- Step-color gradient from early path to late path.

The arrow approach is easier to read on dense coverage maps. The gradient approach is useful for seeing overall temporal progression.

Suggested default:

- Keep `final_map.png` as the main report artifact.
- Add `trajectory_order_map.png` only when requested or as a standard extra output if it stays fast.

## 4. `recovery_debug_map.png`

Add an optional debug map that highlights recovery/backtracking behavior:

- normal coverage path in muted red or gray,
- recovery backtracking segments in blue,
- recovery branch segments in orange,
- recovery start/end points as small markers.

This should use `path_log.csv` actions such as:

- `recover_backtrack`
- `recover_branch`

The goal is explanation, not new behavior.

## 5. Offset Overlapping Forward / Backward Edges

Coverage and recovery often traverse the same edge in both directions. A single line hides this.

v0.2.3 can slightly offset repeated or reverse-direction edges in visualization only:

- Compute edge direction `(a, b)`.
- If reverse edge `(b, a)` also appears, draw one edge with a small perpendicular offset.
- Keep the offset small enough that the path still aligns visually with cells.

This should be a plotting-only transformation. It must not affect path data, metrics, or planner behavior.

## 6. `trajectory.gif` From `path_log.csv`

An optional GIF can animate the robot trajectory using the existing CSV log.

Design:

- Read `path_log.csv`.
- Draw the static map once.
- Animate a simple Rover icon over logged `(x, y)` positions.
- Use a small triangle, square, or circle for the rover.
- Color or briefly flash the rover during recovery actions.

This should remain a lightweight matplotlib animation. Do not introduce a game engine, robotics simulator, or GUI dependency.

## 7. Frame Count / Sampling Strategy

Coverage logs can be long, so GIF generation needs sampling.

Recommended strategy:

- Set `max_frames`, default `200`.
- If `len(path_log) <= max_frames`, use every row.
- Otherwise sample evenly across the log.
- Always include first and last frames.
- Consider a low FPS such as `8` or `10`.

Example:

```text
sample_stride = ceil(path_length / max_frames)
frames = rows[::sample_stride]
ensure rows[0] and rows[-1] are included
```

This keeps GIFs small enough for GitHub and research notes.

## 8. Files That Would Need To Change

Likely implementation files:

- `src/robot_nav_sim/visualization.py`
  - add richer title formatting,
  - add trajectory order PNG function,
  - add recovery debug PNG function,
  - add optional GIF function,
  - add edge-offset helper for plotting.

- `examples/run_mvp.py`
  - optionally call the extra visualization functions,
  - possibly add flags such as `--extra-visuals` or `--animation`.

- `tests/test_outputs.py`
  - add focused tests for generated PNG/GIF outputs.

Possibly documentation:

- `README.md`
  - document optional visualization outputs and when to use them.

- `docs/VERSION_0_2_3_VISUALIZATION.md`
  - summarize the visualization update if implemented.

No planner, simulator, metrics, map schema, or recovery algorithm changes should be required.

## 9. Testing PNG/GIF Generation

Tests should stay small and avoid visual pixel-perfect assertions.

Recommended tests:

- Use a tiny or existing map.
- Run the simulator once.
- Call each visualization function with a `tmp_path`.
- Assert expected files exist.
- Assert file size is greater than zero.
- For GIF, assert the file exists and has a reasonable nonzero size.

Example expectations:

```text
final_map.png exists
trajectory_order_map.png exists
recovery_debug_map.png exists
trajectory.gif exists when animation is requested
```

Do not test exact colors, font layout, or frame-by-frame image contents unless a future regression requires it.

## Non-Goals

v0.2.3 visualization hardening should not add:

- Mission Planner,
- ArduPilot,
- ROS,
- Gazebo,
- SLAM,
- real LiDAR,
- camera processing,
- deep learning,
- hardware support.

It should remain a local, lightweight, reproducible Python visualization improvement.
