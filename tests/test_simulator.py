from pathlib import Path

from robot_nav_sim import GridMap2D5, Simulator

ROOT = Path(__file__).resolve().parents[1]


def run_case(name: str):
    grid = GridMap2D5.from_json(ROOT / "maps" / name)
    simulator = Simulator(grid)
    return simulator.run()


def test_simulator_no_collision() -> None:
    result = run_case("baseline_room.json")

    assert result.metrics.collision_count == 0
    assert result.metrics.coverage_rate >= 0.85


def test_recovery_triggered_in_u_trap() -> None:
    result = run_case("u_trap_room.json")

    assert result.metrics.collision_count == 0
    assert result.metrics.recovery_count >= 1
    assert result.metrics.coverage_rate >= 0.85
