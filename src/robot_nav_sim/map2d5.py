from __future__ import annotations

from collections import deque
from dataclasses import dataclass, replace
import json
from pathlib import Path
from typing import Iterable

CellPos = tuple[int, int]
PlannerMode = str

BLOCKED_SEMANTICS = {"wall", "hard_obstacle", "pit", "too_high", "too_steep"}
SOFT_COSTS = {
    "floor": 1.0,
    "carpet": 1.5,
    "low_bump": 2.0,
    "curtain_soft": 2.0,
    "unknown": 3.0,
}
BASELINE_STEP_COST = 1.0
MAX_HEIGHT = 2.5
MAX_SLOPE = 20.0
MAX_ROUGHNESS = 0.4


@dataclass(frozen=True)
class Cell:
    x: int
    y: int
    occupied: bool = False
    height: float = 0.0
    slope: float = 0.0
    roughness: float = 0.0
    semantic: str = "floor"
    traversability_cost: float = 1.0


class GridMap2D5:
    """2D planning surface whose cells keep 2.5D traversability fields."""

    def __init__(self, name: str, width: int, height: int, start: CellPos, cells: list[list[Cell]]):
        self.name = name
        self.width = width
        self.height = height
        self.start = start
        self._cells = cells
        if not self.in_bounds(start):
            raise ValueError(f"start {start} is outside map bounds")
        if not self.is_traversable(start):
            raise ValueError(f"start {start} is not traversable")

    @classmethod
    def from_json(cls, path: str | Path) -> GridMap2D5:
        with Path(path).open("r", encoding="utf-8") as f:
            return cls.from_dict(json.load(f))

    @classmethod
    def from_dict(cls, data: dict) -> GridMap2D5:
        width = int(data["width"])
        height = int(data["height"])
        start = (int(data["start"][0]), int(data["start"][1]))
        defaults = data.get("defaults", {})
        semantic = str(defaults.get("semantic", "floor"))
        default_cost = float(defaults.get("traversability_cost", SOFT_COSTS.get(semantic, 1.0)))

        cells = [
            [
                Cell(
                    x=x,
                    y=y,
                    occupied=bool(defaults.get("occupied", False)),
                    height=float(defaults.get("height", 0.0)),
                    slope=float(defaults.get("slope", 0.0)),
                    roughness=float(defaults.get("roughness", 0.0)),
                    semantic=semantic,
                    traversability_cost=default_cost,
                )
                for x in range(width)
            ]
            for y in range(height)
        ]

        for feature in data.get("features", []):
            cls._apply_feature(cells, width, height, feature)
        for rectangle in data.get("rectangles", []):
            cls._apply_feature(cells, width, height, rectangle)

        return cls(str(data.get("name", "unnamed")), width, height, start, cells)

    @staticmethod
    def _apply_feature(cells: list[list[Cell]], width: int, height: int, feature: dict) -> None:
        semantic = str(feature.get("semantic", feature.get("type", "floor")))
        rect = feature.get("rect")
        if rect is None:
            rect = [feature["x"], feature["y"], feature["w"], feature["h"]]
        x0, y0, rect_w, rect_h = [int(value) for value in rect]
        occupied = bool(feature.get("occupied", semantic in BLOCKED_SEMANTICS))
        cost = float(feature.get("traversability_cost", SOFT_COSTS.get(semantic, 1.0)))

        for y in range(max(0, y0), min(height, y0 + rect_h)):
            for x in range(max(0, x0), min(width, x0 + rect_w)):
                cells[y][x] = replace(
                    cells[y][x],
                    occupied=occupied,
                    height=float(feature.get("height", cells[y][x].height)),
                    slope=float(feature.get("slope", cells[y][x].slope)),
                    roughness=float(feature.get("roughness", cells[y][x].roughness)),
                    semantic=semantic,
                    traversability_cost=cost,
                )

    def in_bounds(self, cell: CellPos) -> bool:
        x, y = cell
        return 0 <= x < self.width and 0 <= y < self.height

    def cell(self, x: int, y: int) -> Cell:
        if not self.in_bounds((x, y)):
            raise IndexError(f"cell {(x, y)} is outside map bounds")
        return self._cells[y][x]

    def neighbors4(self, cell: CellPos) -> list[CellPos]:
        x, y = cell
        candidates = [(x + 1, y), (x, y + 1), (x - 1, y), (x, y - 1)]
        return [candidate for candidate in candidates if self.in_bounds(candidate)]

    def is_traversable(self, cell: CellPos) -> bool:
        if not self.in_bounds(cell):
            return False
        grid_cell = self.cell(*cell)
        if grid_cell.occupied or grid_cell.semantic in BLOCKED_SEMANTICS:
            return False
        return (
            grid_cell.height <= MAX_HEIGHT
            and grid_cell.slope <= MAX_SLOPE
            and grid_cell.roughness <= MAX_ROUGHNESS
        )

    def traversability_class(self, cell: CellPos) -> str:
        if not self.is_traversable(cell):
            return "blocked"
        grid_cell = self.cell(*cell)
        if grid_cell.traversability_cost > 1.5:
            return "cautious"
        return "free"

    def cost(self, cell: CellPos, planner_mode: PlannerMode = "cost_2d5") -> float:
        if not self.is_traversable(cell):
            return float("inf")
        if planner_mode == "baseline_2d":
            return BASELINE_STEP_COST
        if planner_mode != "cost_2d5":
            raise ValueError(f"unknown planner_mode: {planner_mode}")
        grid_cell = self.cell(*cell)
        return float(grid_cell.traversability_cost or SOFT_COSTS.get(grid_cell.semantic, 1.0))

    def reachable_cells(self, start: CellPos | None = None) -> set[CellPos]:
        origin = self.start if start is None else start
        if not self.is_traversable(origin):
            return set()
        reached = {origin}
        frontier: deque[CellPos] = deque([origin])
        while frontier:
            current = frontier.popleft()
            for neighbor in self.neighbors4(current):
                if neighbor not in reached and self.is_traversable(neighbor):
                    reached.add(neighbor)
                    frontier.append(neighbor)
        return reached

    def traversable_cells(self) -> Iterable[CellPos]:
        for y in range(self.height):
            for x in range(self.width):
                pos = (x, y)
                if self.is_traversable(pos):
                    yield pos

    def serpentine_cells(self, allowed: set[CellPos] | None = None) -> list[CellPos]:
        ordered: list[CellPos] = []
        for y in range(self.height):
            xs = range(self.width) if y % 2 == 0 else range(self.width - 1, -1, -1)
            for x in xs:
                pos = (x, y)
                if self.is_traversable(pos) and (allowed is None or pos in allowed):
                    ordered.append(pos)
        return ordered
