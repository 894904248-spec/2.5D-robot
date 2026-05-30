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
