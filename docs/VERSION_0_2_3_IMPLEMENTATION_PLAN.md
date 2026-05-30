# v0.2.3 Implementation Plan

This plan prepares the v0.2.3 visualization and task-semantics refinement. It is not an algorithm upgrade and does not add any robotics stack integration.

Non-goals:

- Mission Planner
- ArduPilot
- MAVLink
- ROS
- Gazebo
- SLAM
- real LiDAR
- camera processing
- deep learning
- hardware support
- planner redesign
- cost-aware A* redesign
- coverage planner redesign
- recovery algorithm redesign

## 1. Current Problem Summary

The current figures and output naming are useful for validation, but several things are unclear for a human reader:

- The plot title does not show `planner_mode` clearly.
- The current `start` / `end` labels are slightly misleading for cleaning tasks. A cleaning run has a dock/start and a final pose after coverage, not necessarily a fixed target destination.
- `final_map.png` shows the visited path but does not show execution order clearly.
- Dense coverage paths overlap, so forward movement and backtracking can hide each other.
- Recovery behavior is hard to see because recovery segments use the same visual style as normal coverage movement.
- The point-to-point route diagnostic (`outputs/cost_choice_route_comparison.png`) and full-room coverage outputs (`final_map.png`) may be confused unless labeled clearly.

## 2. v0.2.3 Goals

v0.2.3 should be defined as visualization and task-semantics refinement.

It should improve how existing simulator behavior is explained:

- clearer map titles,
- clearer cleaning-task labels,
- separate trajectory-order and recovery-debug views,
- optional lightweight animation from existing log data,
- clearer README/docs for interpreting full-coverage outputs.

It should not change planner logic, cost-aware A*, coverage planning, recovery behavior, map semantics, or metrics unless a small plotting/logging bug is found.

## 3. Output Figure Design

### `final_map.png`

Purpose:

- Main report artifact.
- Show map semantics, blocked cells, cautious/high-cost cells, visited cells, overall path, dock/start, and final pose.
- Include title metadata: `map_name`, `planner_mode`, `task_type`, `coverage_rate`, `collision_count`, and `recovery_count`.

Should not try to show:

- detailed execution order,
- every direction change,
- every recovery segment distinctly,
- animation state.

Color / legend proposal:

- normal floor: light neutral
- cautious/high-cost cells: warm tan
- blocked cells: dark gray / black
- visited cells: soft blue overlay
- path: red or muted red
- dock/start: green marker
- final_pose: orange marker

Arrows:

- Not needed by default in `final_map.png`; they can clutter dense maps.

Clutter control:

- Keep path line thin.
- Keep visited overlay transparent.
- Use a compact legend.

### `trajectory_order_map.png`

Purpose:

- Show the temporal order of movement during full coverage.
- Make it easier to understand how the robot progresses through the map.

Should not try to show:

- detailed recovery semantics,
- frame-by-frame animation,
- exact cost accounting.

Color / legend proposal:

- static map colors same as `final_map.png`,
- trajectory colored by step index with a sequential colormap,
- optional sparse arrows every N points.

Arrows:

- Use sparse arrows only, e.g. every 25-50 steps or adaptive based on path length.
- Avoid arrows on every segment.

Clutter control:

- Use step-color gradient as the primary order cue.
- Use arrows sparingly.
- Consider plotting only a sampled subset of arrows.

### `recovery_debug_map.png`

Purpose:

- Explain recovery and backtracking behavior.
- Make `recover_backtrack` and `recover_branch` actions visible.

Should not try to show:

- all temporal detail,
- animation,
- point-to-point cost-choice proof.

Color / legend proposal:

- normal coverage/transit: gray or muted red
- `recover_backtrack`: blue
- `recover_branch`: orange
- recovery event markers: small circles or stars
- blocked/cautious map colors same as other figures

Arrows:

- Optional sparse arrows on recovery segments only.

Clutter control:

- Muting non-recovery path is more important than drawing all movement loudly.
- If there are no recovery actions, still create the file with a title such as `no recovery events`.

### `trajectory.gif`

Purpose:

- Lightweight animation of the existing path log.
- Help visually explain coverage sequence in presentations.

Should not try to show:

- high-fidelity robot geometry,
- physical dynamics,
- sensor simulation,
- real-time control.

Color / legend proposal:

- static background same as final map,
- simple rover marker as a triangle, small square, or circle,
- normal movement marker in red or blue,
- recovery frames briefly highlighted in blue/orange.

Arrows:

- Not needed in the GIF; motion across frames provides direction.

Clutter control:

- Sample frames.
- Keep marker simple.
- Do not redraw heavy labels every frame if avoidable.

## 4. Cleaning-Task Semantics

Recommended labels:

- `dock/start`: the initial cell, treated as a simple simulated dock location.
- `final_pose`: the robot's final pose after the coverage run ends.
- `return_to_dock`: optional future phase if explicit return behavior is implemented.

Full coverage does not have a fixed target destination because the task is to visit all reachable traversable cells. The last pose is a byproduct of coverage order, not a semantic target.

Two possible approaches:

### A. Relabel `end` as `final_pose`

Benefits:

- Pure visualization/semantics change.
- No planner, simulator, metrics, or tests need major changes.
- Avoids implying the robot has a fixed destination target.
- Low risk for v0.2.3.

Risks:

- The robot still does not return to dock after cleaning.
- Some readers may expect a cleaning robot to return to its dock.

### B. Add `return_to_start` / `return_to_dock`

Benefits:

- More realistic cleaning-task story.
- Final pose can become the dock again.

Risks:

- Changes path length, total cost, turn count, and path logs.
- Adds a new task phase and new acceptance questions.
- May blur v0.2.3's scope as a visualization-only refinement.
- Requires careful distinction between coverage metrics and return-home metrics.

Recommendation:

Choose A for v0.2.3. Relabel `end` as `final_pose` and reserve `return_to_dock` as future work. Do not implement return-to-dock in v0.2.3.

## 5. Path Log / Action Label Design

Current `path_log.csv` fields include:

- `step`
- `x`
- `y`
- `action`
- `target_x`
- `target_y`
- `semantic`
- `height`
- `slope`
- `roughness`
- `traversability_class`
- `cost`
- `step_cost`
- `cumulative_cost`
- `planner_mode`
- `coverage_rate`
- `recovery_id`
- `reason`

Current action labels include:

- `start`
- `cover`
- `replan`
- `recover_backtrack`
- `recover_branch`

Proposed semantic labels:

- `start`
- `move`
- `coverage`
- `transit`
- `recovery`
- `return_to_dock`
- `finish`

Mapping and code-change impact:

- `start`: already available.
- `coverage`: current `cover` can be displayed as `coverage` in visualization without changing logs, or renamed in simulator with test updates.
- `transit`: current `replan` roughly maps to transit/replanned movement and would require either a display mapping or simulator action rename.
- `recovery`: current `recover_backtrack` and `recover_branch` are already more precise than a generic `recovery` label.
- `return_to_dock`: not available unless return-to-dock is implemented.
- `finish`: not currently logged; adding it would be a small simulator/logging change but is not required for v0.2.3.

Recommendation:

Do not rename action labels in v0.2.3 unless necessary. Add visualization-side grouping:

- `cover` -> coverage
- `replan` -> transit
- `recover_backtrack` / `recover_branch` -> recovery

This avoids breaking existing logs/tests.

## 6. Overlapping Path Visualization

Options:

### Alpha transparency

Pros:

- Very simple.
- No geometry distortion.

Cons:

- Reverse-direction overlap may still be hidden.

### Sparse arrows

Pros:

- Good direction cue.
- Easy to sample.

Cons:

- Can clutter dense coverage maps.

### Step-index color gradient

Pros:

- Strong temporal cue.
- Good for trajectory order.

Cons:

- Needs a colorbar or legend explanation.
- Still does not fully solve bidirectional overlap.

### Slight offset for repeated/bidirectional edges

Pros:

- Reveals repeated forward/backward traversal.

Cons:

- Can make paths look outside cells.
- More implementation complexity.
- Easy to overdo visually.

Recommendation:

Use a step-index color gradient plus sparse arrows for `trajectory_order_map.png`. Defer edge offsets unless recovery paths remain unreadable after the simpler approach.

For `recovery_debug_map.png`, highlight recovery actions with distinct colors first. Add offsets only if needed later.

## 7. GIF Design

Input source:

- Prefer `path_log.csv` because it is already persisted and contains action labels, planner mode, cost fields, and recovery IDs.
- Simulator path data can remain the internal source for PNGs.

Rover marker:

- Use a simple triangle marker or small circular marker.
- Avoid custom images or heavy assets.

Frame sampling:

- Default `max_frames = 200`.
- If log rows are fewer than `max_frames`, use all rows.
- Otherwise sample evenly.
- Always include the first and last row.

Suggested strategy:

```text
stride = ceil((len(rows) - 1) / (max_frames - 1))
frames = rows[::stride]
append rows[-1] if missing
```

Recovery / return path distinction:

- Normal coverage/transit marker: blue or red.
- Recovery backtrack marker: blue.
- Recovery branch marker: orange.
- `return_to_dock`: reserved for future, not expected in v0.2.3.

File size control:

- Keep figure size modest.
- Use 8-10 FPS.
- Cap frames at 200 by default.
- Prefer matplotlib + Pillow writer.
- Do not add heavy dependencies.

## 8. Files To Modify During Implementation

Expected files:

- `src/robot_nav_sim/visualization.py`
  - Add richer title helper.
  - Relabel `start` / `end` to `dock/start` / `final_pose`.
  - Add `save_trajectory_order_map()`.
  - Add `save_recovery_debug_map()`.
  - Add `save_trajectory_gif()` or equivalent.
  - Add path-log reading helpers if GIF/debug maps use CSV.

- `examples/run_mvp.py`
  - Call new visualization functions after the existing run.
  - Possibly add non-breaking flags such as `--no-extra-visuals` or `--animation`.
  - Keep default behavior simple and reproducible.

- `tests/test_outputs.py`
  - Add file-existence tests for `trajectory_order_map.png`, `recovery_debug_map.png`, and `trajectory.gif`.
  - Avoid pixel-perfect visual assertions.

- `README.md`
  - Explain the new output files and how to interpret them.
  - Clarify that `final_pose` is not a fixed target.

- `docs/VERSION_0_2_2_COMPLEX_INDOOR_VALIDATION.md`
  - Optionally mention the new visual artifacts if complex indoor validation outputs are regenerated.

Potential but probably unnecessary:

- `src/robot_nav_sim/simulator.py`
  - Only if a `finish` log row or action-label change is truly needed.

- `src/robot_nav_sim/metrics.py`
  - No expected change for v0.2.3.

Do not modify planner logic or route-choice logic.

## 9. Tests And Validation

Minimal tests:

- `final_map.png` exists and has nonzero size.
- `trajectory_order_map.png` exists and has nonzero size.
- `recovery_debug_map.png` exists and has nonzero size.
- `trajectory.gif` exists and has nonzero size when GIF generation is enabled.
- `path_log.csv` contains expected columns and existing action labels.
- Existing planner, simulator, and output tests still pass.

Recommended validation commands after implementation:

```powershell
python examples/run_mvp.py --map maps/complex_indoor_room.json --output outputs/complex_indoor_baseline --planner-mode baseline_2d
python examples/run_mvp.py --map maps/complex_indoor_room.json --output outputs/complex_indoor_2d5 --planner-mode cost_2d5
python -m pytest -q
```

In this local environment, plain `python` may not be on PATH, so the bundled Python executable may be needed for validation.

## 10. Acceptance Criteria

v0.2.3 is complete when:

- `pytest` passes.
- Complex indoor baseline and `cost_2d5` outputs are generated.
- Each output directory contains:
  - `final_map.png`
  - `trajectory_order_map.png`
  - `recovery_debug_map.png`
  - `trajectory.gif`
  - `path_log.csv`
- Figure titles include `map_name` and `planner_mode`.
- Figure titles or labels include task context such as `task=full_coverage`.
- Full coverage figures do not imply a fixed target destination.
- `dock/start` and `final_pose` are labeled clearly.
- README explains the difference between:
  - `final_map.png`
  - `trajectory_order_map.png`
  - `recovery_debug_map.png`
  - `trajectory.gif`

## Phase 1 Scope

Phase 1 implements the PNG subset only:

- clearer `final_map.png` title and legend,
- `dock/start` and `final_pose` labels,
- `trajectory_order_map.png`,
- `recovery_debug_map.png`,
- README/docs descriptions for the PNG outputs,
- minimal PNG output existence tests.

Phase 1 deliberately does not implement:

- `trajectory.gif`,
- `return_to_dock`,
- action-label renaming,
- planner or simulator algorithm changes.

## 11. Implementation Order

Step 1: title/legend refinement

- Add a helper that formats figure titles with map name, planner mode, task type, coverage rate, collision count, and recovery count.
- Relabel markers to `dock/start` and `final_pose`.

Step 2: action labels / path log check

- Keep existing action labels.
- Add visualization-side grouping for coverage/transit/recovery.
- Confirm no path-log schema changes are needed.

Step 3: `trajectory_order_map.png`

- Plot static map background.
- Draw path with step-index gradient.
- Add sparse arrows using sampled path points.

Step 4: `recovery_debug_map.png`

- Plot static map background.
- Draw normal path muted.
- Highlight `recover_backtrack` and `recover_branch` segments from path-log action rows.

Step 5: GIF generation

- Read `path_log.csv`.
- Sample frames with `max_frames`.
- Animate a simple rover marker.
- Save `trajectory.gif` with matplotlib + Pillow.

Step 6: README/docs update

- Document new output files.
- Explain that the visualization update does not change planner behavior.

Step 7: tests and validation

- Add output existence tests.
- Run complex indoor demos in both modes.
- Run full pytest.

## 12. Risks

- GIF files may become too large if frame sampling is not capped.
- Arrows can clutter dense coverage figures.
- Adding return-to-dock now would change metrics and confuse v0.2/v0.2.3 comparisons.
- Action labels may not be precise enough for every movement type; visualization-side grouping should be documented.
- Visual offsets can make paths appear outside cells or imply geometry that the simulator does not use.
- Recovery debug maps may look empty on maps with no recovery events; they should still explain that no recovery occurred.
- Extra visual outputs may slow tests if GIF generation is too heavy; tests should use small maps or low frame caps.
