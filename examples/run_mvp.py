from __future__ import annotations

import argparse
from pathlib import Path

from robot_nav_sim import GridMap2D5, Simulator
from robot_nav_sim.visualization import save_final_map


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Run the robot navigation MVP simulation.")
    parser.add_argument("--map", required=True, help="Path to a JSON map file.")
    parser.add_argument("--output", required=True, help="Output directory for PNG and CSV files.")
    parser.add_argument(
        "--planner-mode",
        choices=["baseline_2d", "cost_2d5"],
        default="baseline_2d",
        help="Planning mode: baseline_2d ignores soft costs, cost_2d5 uses traversability costs.",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    grid = GridMap2D5.from_json(args.map)
    simulator = Simulator(grid, planner_mode=args.planner_mode)
    result = simulator.run()

    output_dir = Path(args.output)
    path_log = simulator.write_path_log(output_dir)
    final_map = save_final_map(grid, result.visited, result.path, output_dir)

    print(f"map_name: {grid.name}")
    print(f"planner_mode: {args.planner_mode}")
    for key, value in result.metrics.as_dict().items():
        print(f"{key}: {value}")
    print(f"final_map: {final_map}")
    print(f"path_log: {path_log}")


if __name__ == "__main__":
    main()
