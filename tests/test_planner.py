from pathlib import Path

from robot_nav_sim import AStarPlanner, GridMap2D5

ROOT = Path(__file__).resolve().parents[1]


def load_map(name: str) -> GridMap2D5:
    return GridMap2D5.from_json(ROOT / "maps" / name)


def test_traversability_rules() -> None:
    grid = GridMap2D5.from_dict(
        {
            "name": "rules",
            "width": 7,
            "height": 2,
            "start": [0, 0],
            "features": [
                {"type": "wall", "rect": [1, 0, 1, 1]},
                {"type": "hard_obstacle", "rect": [2, 0, 1, 1]},
                {"type": "pit", "rect": [3, 0, 1, 1]},
                {"type": "carpet", "rect": [4, 0, 1, 1]},
                {"type": "low_bump", "rect": [5, 0, 1, 1]},
                {"type": "too_steep", "rect": [6, 0, 1, 1]},
            ],
        }
    )

    assert grid.is_traversable((0, 0))
    assert not grid.is_traversable((1, 0))
    assert not grid.is_traversable((2, 0))
    assert not grid.is_traversable((3, 0))
    assert not grid.is_traversable((6, 0))
    assert grid.is_traversable((4, 0))
    assert grid.is_traversable((5, 0))
    assert grid.cost((4, 0)) > grid.cost((0, 0))
    assert grid.cost((5, 0)) > grid.cost((0, 0))


def test_astar_finds_path() -> None:
    grid = load_map("baseline_room.json")
    planner = AStarPlanner(grid)
    goal = (27, 17)
    path = planner.plan(grid.start, goal)

    assert path
    assert path[0] == grid.start
    assert path[-1] == goal
    assert all(grid.is_traversable(cell) for cell in path)


def test_astar_returns_empty_for_blocked_goal() -> None:
    grid = load_map("baseline_room.json")
    planner = AStarPlanner(grid)

    assert planner.plan(grid.start, (0, 0)) == []


def test_astar_start_equals_goal() -> None:
    grid = load_map("baseline_room.json")
    planner = AStarPlanner(grid)

    assert planner.plan(grid.start, grid.start) == [grid.start]


def test_cost_2d5_prefers_lower_cost_route() -> None:
    grid = load_map("cost_choice_room.json")
    start = grid.start
    goal = (10, 3)
    baseline_path = AStarPlanner(grid, planner_mode="baseline_2d").plan(start, goal)
    cost_path = AStarPlanner(grid, planner_mode="cost_2d5").plan(start, goal)

    baseline_high_cost = sum(1 for cell in baseline_path if grid.cost(cell, "cost_2d5") > 1.0)
    cost_high_cost = sum(1 for cell in cost_path if grid.cost(cell, "cost_2d5") > 1.0)
    baseline_total = sum(grid.cost(cell, "cost_2d5") for cell in baseline_path[1:])
    cost_total = sum(grid.cost(cell, "cost_2d5") for cell in cost_path[1:])

    assert baseline_path
    assert cost_path
    assert len(baseline_path) < len(cost_path)
    assert baseline_high_cost > cost_high_cost
    assert cost_total < baseline_total


def test_high_cost_cells_are_traversable() -> None:
    grid = load_map("cost_choice_room.json")

    assert grid.is_traversable((2, 3))
    assert grid.cost((2, 3), "cost_2d5") > 1.0
    assert grid.traversability_class((2, 3)) == "cautious"
