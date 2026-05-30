from __future__ import annotations

from dataclasses import dataclass
import heapq

from .map2d5 import CellPos, GridMap2D5


def manhattan(a: CellPos, b: CellPos) -> int:
    return abs(a[0] - b[0]) + abs(a[1] - b[1])


@dataclass
class AStarPlanner:
    grid: GridMap2D5
    planner_mode: str = "baseline_2d"

    def plan(self, start: CellPos, goal: CellPos) -> list[CellPos]:
        if not self.grid.is_traversable(start) or not self.grid.is_traversable(goal):
            return []
        if start == goal:
            return [start]

        frontier: list[tuple[float, int, CellPos]] = []
        sequence = 0
        heapq.heappush(frontier, (0.0, sequence, start))
        came_from: dict[CellPos, CellPos | None] = {start: None}
        cost_so_far: dict[CellPos, float] = {start: 0.0}

        while frontier:
            _, _, current = heapq.heappop(frontier)
            if current == goal:
                break

            for neighbor in self.grid.neighbors4(current):
                if not self.grid.is_traversable(neighbor):
                    continue
                new_cost = cost_so_far[current] + self.grid.cost(neighbor, self.planner_mode)
                if neighbor not in cost_so_far or new_cost < cost_so_far[neighbor]:
                    cost_so_far[neighbor] = new_cost
                    sequence += 1
                    priority = new_cost + manhattan(neighbor, goal)
                    heapq.heappush(frontier, (priority, sequence, neighbor))
                    came_from[neighbor] = current

        if goal not in came_from:
            return []

        path = [goal]
        current = goal
        while came_from[current] is not None:
            current = came_from[current]  # type: ignore[assignment]
            path.append(current)
        path.reverse()
        return path
