import csv
from pathlib import Path

from robot_nav_sim import GridMap2D5, SimulationMetrics, Simulator
from robot_nav_sim.visualization import save_final_map

ROOT = Path(__file__).resolve().parents[1]


def test_metrics_as_dict() -> None:
    metrics = SimulationMetrics(
        coverage_rate=0.9,
        visited_cells=9,
        total_reachable_cells=10,
        steps=12,
        collision_count=0,
        recovery_count=1,
        unreachable_targets=2,
        path_length=12,
        turn_count=3,
    )

    assert metrics.as_dict() == {
        "coverage_rate": 0.9,
        "visited_cells": 9,
        "total_reachable_cells": 10,
        "steps": 12,
        "collision_count": 0,
        "recovery_count": 1,
        "unreachable_targets": 2,
        "path_length": 12,
        "turn_count": 3,
    }


def test_visualization_creates_png(tmp_path: Path) -> None:
    grid = GridMap2D5.from_json(ROOT / "maps" / "baseline_room.json")
    simulator = Simulator(grid)
    result = simulator.run()

    image_path = save_final_map(grid, result.visited, result.path, tmp_path)

    assert image_path.exists()
    assert image_path.name == "final_map.png"
    assert image_path.stat().st_size > 0


def test_path_log_contains_expected_columns(tmp_path: Path) -> None:
    grid = GridMap2D5.from_json(ROOT / "maps" / "baseline_room.json")
    simulator = Simulator(grid)
    simulator.run()

    log_path = simulator.write_path_log(tmp_path)

    with log_path.open("r", newline="", encoding="utf-8") as f:
        reader = csv.DictReader(f)
        assert reader.fieldnames == Simulator.LOG_FIELDS
        first_row = next(reader)

    assert first_row["step"] == "0"
    assert first_row["action"] == "start"
    assert "coverage_rate" in first_row
