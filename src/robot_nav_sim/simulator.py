from __future__ import annotations

import csv
from dataclasses import dataclass
from pathlib import Path

from .map2d5 import CellPos, GridMap2D5
from .metrics import SimulationMetrics
from .planner import AStarPlanner, manhattan
from .robot import Robot


@dataclass
class SimulationResult:
    metrics: SimulationMetrics
    visited: set[CellPos]
    path: list[CellPos]
    log_rows: list[dict[str, int | float | str]]


class Simulator:
    LOG_FIELDS = [
        "step",
        "x",
        "y",
        "action",
        "target_x",
        "target_y",
        "semantic",
        "height",
        "slope",
        "roughness",
        "traversability_class",
        "cost",
        "coverage_rate",
        "recovery_id",
        "reason",
    ]

    def __init__(self, grid: GridMap2D5, max_steps: int | None = None):
        self.grid = grid
        self.planner = AStarPlanner(grid)
        self.robot = Robot(grid.start)
        self.reachable = grid.reachable_cells(grid.start)
        self.metrics = SimulationMetrics(total_reachable_cells=len(self.reachable))
        self.branch_candidates: dict[CellPos, set[CellPos]] = {}
        self.log_rows: list[dict[str, int | float | str]] = []
        self.max_steps = max_steps or max(1000, len(self.reachable) * 20)
        self._last_direction: CellPos | None = None
        self._active_target: CellPos | None = None
        self._recovery_id = 0
        self._serpentine_rank = {
            pos: index for index, pos in enumerate(self.grid.serpentine_cells(self.reachable))
        }
        self._log("start", reason="initial_position")
        self._update_branch_candidates(self.robot.position)

    def run(self) -> SimulationResult:
        while len(self.robot.visited & self.reachable) < len(self.reachable):
            if self.metrics.steps >= self.max_steps:
                break

            self._update_branch_candidates(self.robot.position)
            neighbor = self._choose_unvisited_neighbor(self.robot.position)
            if neighbor is not None:
                self._move_via_astar(neighbor, "cover")
                continue

            if self._recover_to_branch():
                continue

            target = self._next_serpentine_target()
            if target is None:
                break
            if not self._move_via_astar(target, "replan"):
                self.metrics.unreachable_targets += 1
                self.robot.visited.add(target)

        self._refresh_metrics()
        return SimulationResult(
            metrics=self.metrics,
            visited=set(self.robot.visited),
            path=list(self.robot.safe_path),
            log_rows=list(self.log_rows),
        )

    def write_path_log(self, output_dir: str | Path) -> Path:
        out = Path(output_dir)
        out.mkdir(parents=True, exist_ok=True)
        path = out / "path_log.csv"
        with path.open("w", newline="", encoding="utf-8") as f:
            writer = csv.DictWriter(f, fieldnames=self.LOG_FIELDS)
            writer.writeheader()
            writer.writerows(self.log_rows)
        return path

    def _move_via_astar(self, target: CellPos, action: str) -> bool:
        previous_target = self._active_target
        self._active_target = target
        path = self.planner.plan(self.robot.position, target)
        if not path:
            self._active_target = previous_target
            return False
        moved = self._execute_path(path, action, reason=f"toward_{action}")
        self._active_target = previous_target
        return moved

    def _execute_path(self, path: list[CellPos], action: str, reason: str = "") -> bool:
        if not path or path[0] != self.robot.position:
            return False
        for next_cell in path[1:]:
            if self.metrics.steps >= self.max_steps:
                return False
            if not self.grid.is_traversable(next_cell):
                self.metrics.collision_count += 1
                return False
            self._record_turn(next_cell)
            self.robot.record_safe_step(next_cell)
            self.metrics.steps += 1
            self.metrics.path_length += 1
            self._update_branch_candidates(next_cell)
            self._log(action, reason=reason)
        return True

    def _record_turn(self, next_cell: CellPos) -> None:
        dx = next_cell[0] - self.robot.position[0]
        dy = next_cell[1] - self.robot.position[1]
        direction = (dx, dy)
        if self._last_direction is not None and direction != self._last_direction:
            self.metrics.turn_count += 1
        self._last_direction = direction

    def _choose_unvisited_neighbor(self, cell: CellPos) -> CellPos | None:
        candidates = [
            neighbor
            for neighbor in self.grid.neighbors4(cell)
            if neighbor in self.reachable and neighbor not in self.robot.visited
        ]
        if not candidates:
            return None
        return min(candidates, key=lambda pos: (self._serpentine_rank.get(pos, 10**9), self.grid.cost(pos)))

    def _update_branch_candidates(self, cell: CellPos) -> None:
        candidates = {
            neighbor
            for neighbor in self.grid.neighbors4(cell)
            if neighbor in self.reachable and neighbor not in self.robot.visited
        }
        if candidates:
            self.branch_candidates.setdefault(cell, set()).update(candidates)

    def _recover_to_branch(self) -> bool:
        for index in range(len(self.robot.safe_path) - 1, -1, -1):
            branch_cell = self.robot.safe_path[index]
            candidates = self._fresh_candidates(branch_cell)
            if not candidates:
                continue

            backtrack_path = list(reversed(self.robot.safe_path[index:]))
            if not backtrack_path or backtrack_path[0] != self.robot.position:
                continue

            self.metrics.recovery_count += 1
            self._recovery_id = self.metrics.recovery_count
            previous_target = self._active_target
            self._active_target = branch_cell
            if len(backtrack_path) > 1 and not self._execute_path(
                backtrack_path, "recover_backtrack", reason="backtrack_to_branch_cell"
            ):
                self._active_target = previous_target
                return False
            target = min(candidates, key=lambda pos: (manhattan(branch_cell, pos), pos[1], pos[0]))
            self._active_target = previous_target
            return self._move_via_astar(target, "recover_branch")
        return False

    def _fresh_candidates(self, cell: CellPos) -> set[CellPos]:
        candidates = self.branch_candidates.get(cell, set())
        fresh = {
            pos
            for pos in candidates
            if pos in self.reachable and pos not in self.robot.visited and self.grid.is_traversable(pos)
        }
        self.branch_candidates[cell] = fresh
        return fresh

    def _next_serpentine_target(self) -> CellPos | None:
        for target in self.grid.serpentine_cells(self.reachable):
            if target not in self.robot.visited:
                return target
        return None

    def _log(self, action: str, reason: str = "") -> None:
        grid_cell = self.grid.cell(*self.robot.position)
        reachable_visited = len(self.robot.visited & self.reachable)
        coverage = reachable_visited / len(self.reachable) if self.reachable else 1.0
        target_x = "" if self._active_target is None else self._active_target[0]
        target_y = "" if self._active_target is None else self._active_target[1]
        self.log_rows.append(
            {
                "step": self.metrics.steps,
                "x": self.robot.position[0],
                "y": self.robot.position[1],
                "action": action,
                "target_x": target_x,
                "target_y": target_y,
                "semantic": grid_cell.semantic,
                "height": grid_cell.height,
                "slope": grid_cell.slope,
                "roughness": grid_cell.roughness,
                "traversability_class": self._traversability_class(self.robot.position),
                "cost": self.grid.cost(self.robot.position),
                "coverage_rate": coverage,
                "recovery_id": self._recovery_id if self._recovery_id else "",
                "reason": reason,
            }
        )

    def _refresh_metrics(self) -> None:
        visited_reachable = self.robot.visited & self.reachable
        self.metrics.visited_cells = len(visited_reachable)
        self.metrics.total_reachable_cells = len(self.reachable)
        self.metrics.coverage_rate = (
            len(visited_reachable) / len(self.reachable) if self.reachable else 1.0
        )

    def _traversability_class(self, cell: CellPos) -> str:
        if not self.grid.is_traversable(cell):
            return "blocked"
        cost = self.grid.cost(cell)
        if cost <= 1.0:
            return "normal"
        if cost <= 1.5:
            return "high_cost"
        return "cautious"
