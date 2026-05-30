from __future__ import annotations

from dataclasses import dataclass, field

from .map2d5 import CellPos


@dataclass
class Robot:
    position: CellPos
    safe_path: list[CellPos] = field(default_factory=list)
    visited: set[CellPos] = field(default_factory=set)

    def __post_init__(self) -> None:
        self.safe_path.append(self.position)
        self.visited.add(self.position)

    def record_safe_step(self, position: CellPos) -> None:
        self.position = position
        self.safe_path.append(position)
        self.visited.add(position)
