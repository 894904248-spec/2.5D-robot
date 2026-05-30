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


def test_blocked_cells_are_never_entered() -> None:
    grid = GridMap2D5.from_json(ROOT / "maps" / "u_trap_room.json")
    result = Simulator(grid, planner_mode="cost_2d5").run()

    assert result.metrics.collision_count == 0
    assert all(grid.is_traversable(cell) for cell in result.path)


def test_baseline_and_cost_modes_differ_on_cost_choice_room() -> None:
    grid = GridMap2D5.from_json(ROOT / "maps" / "cost_choice_room.json")
    goal = (10, 3)
    baseline = Simulator(grid, planner_mode="baseline_2d")
    cost_aware = Simulator(grid, planner_mode="cost_2d5")

    baseline_path = baseline.planner.plan(grid.start, goal)
    cost_path = cost_aware.planner.plan(grid.start, goal)

    assert baseline_path != cost_path
    assert len(baseline_path) < len(cost_path)
