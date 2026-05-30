from __future__ import annotations

from dataclasses import dataclass


@dataclass
class SimulationMetrics:
    coverage_rate: float = 0.0
    visited_cells: int = 0
    total_reachable_cells: int = 0
    steps: int = 0
    collision_count: int = 0
    recovery_count: int = 0
    unreachable_targets: int = 0
    path_length: int = 0
    turn_count: int = 0
    total_cost: float = 0.0
    high_cost_cells_entered: int = 0
    cautious_cells_entered: int = 0

    def as_dict(self) -> dict[str, float | int]:
        return {
            "coverage_rate": self.coverage_rate,
            "visited_cells": self.visited_cells,
            "total_reachable_cells": self.total_reachable_cells,
            "steps": self.steps,
            "collision_count": self.collision_count,
            "recovery_count": self.recovery_count,
            "unreachable_targets": self.unreachable_targets,
            "path_length": self.path_length,
            "turn_count": self.turn_count,
            "total_cost": self.total_cost,
            "high_cost_cells_entered": self.high_cost_cells_entered,
            "cautious_cells_entered": self.cautious_cells_entered,
        }
