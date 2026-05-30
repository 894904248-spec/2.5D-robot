from __future__ import annotations

import os
from pathlib import Path
import tempfile
from typing import Iterable

os.environ.setdefault("MPLCONFIGDIR", str(Path(tempfile.gettempdir()) / "robot_nav_mvp_mpl"))

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.colors import ListedColormap
from matplotlib.collections import LineCollection
from matplotlib.lines import Line2D
from matplotlib.patches import Patch

from .map2d5 import BLOCKED_SEMANTICS, CellPos, GridMap2D5
from .metrics import SimulationMetrics

SEMANTIC_VALUES = {
    "floor": 0,
    "carpet": 1,
    "low_bump": 1,
    "curtain_soft": 1,
    "unknown": 4,
}


def _map_layers(grid: GridMap2D5) -> tuple[np.ndarray, ListedColormap, list[str]]:
    data = np.zeros((grid.height, grid.width), dtype=float)
    for y in range(grid.height):
        for x in range(grid.width):
            cell = grid.cell(x, y)
            if cell.occupied or cell.semantic in BLOCKED_SEMANTICS:
                data[y, x] = 5
            else:
                data[y, x] = SEMANTIC_VALUES.get(cell.semantic, 4)
    colors = ["#f7f7f2", "#d7bd82", "#d7bd82", "#d7bd82", "#c8c8c8", "#242424"]
    return data, ListedColormap(colors), colors


def _title(
    grid: GridMap2D5,
    planner_mode: str = "",
    task_type: str = "full_coverage",
    metrics: SimulationMetrics | None = None,
    suffix: str = "",
) -> str:
    parts = [grid.name]
    if planner_mode:
        parts.append(f"planner={planner_mode}")
    if task_type:
        parts.append(f"task={task_type}")
    if metrics is not None:
        parts.extend(
            [
                f"coverage={metrics.coverage_rate:.3f}",
                f"collisions={metrics.collision_count}",
                f"recoveries={metrics.recovery_count}",
            ]
        )
    if suffix:
        parts.append(suffix)
    return " | ".join(parts)


def _draw_base(ax, grid: GridMap2D5) -> list[str]:
    data, cmap, colors = _map_layers(grid)
    ax.imshow(data, cmap=cmap, origin="upper", vmin=0, vmax=5)
    ax.set_xticks(range(grid.width))
    ax.set_yticks(range(grid.height))
    ax.grid(color="#dddddd", linewidth=0.4)
    ax.set_xlim(-0.5, grid.width - 0.5)
    ax.set_ylim(grid.height - 0.5, -0.5)
    ax.set_aspect("equal")
    return colors


def _path_segments(path: list[CellPos]) -> np.ndarray:
    if len(path) < 2:
        return np.empty((0, 2, 2))
    return np.array([[[a[0], a[1]], [b[0], b[1]]] for a, b in zip(path, path[1:])], dtype=float)


def _sparse_arrow_indices(path: list[CellPos], max_arrows: int = 30) -> Iterable[int]:
    if len(path) < 2:
        return []
    stride = max(1, len(path) // max_arrows)
    return range(0, len(path) - 1, stride)


def save_final_map(
    grid: GridMap2D5,
    visited: set[CellPos],
    path: list[CellPos],
    output_dir: str | Path,
    metrics: SimulationMetrics | None = None,
    planner_mode: str = "",
    task_type: str = "full_coverage",
) -> Path:
    out = Path(output_dir)
    out.mkdir(parents=True, exist_ok=True)
    image_path = out / "final_map.png"

    fig, ax = plt.subplots(figsize=(10, 7))
    colors = _draw_base(ax, grid)

    if visited:
        xs = [x for x, _ in visited]
        ys = [y for _, y in visited]
        ax.scatter(xs, ys, s=12, c="#4f7dde", alpha=0.45, marker="s", linewidths=0)

    if path:
        px = [x for x, _ in path]
        py = [y for _, y in path]
        ax.plot(px, py, color="#d64045", linewidth=1.2, alpha=0.85)
        ax.scatter([path[0][0]], [path[0][1]], c="#2f9e44", s=70)
        ax.scatter([path[-1][0]], [path[-1][1]], c="#f08c00", s=70)

    ax.set_title(_title(grid, planner_mode, task_type, metrics))
    legend_items = [
        Patch(facecolor=colors[5], label="obstacle / blocked"),
        Patch(facecolor=colors[0], label="normal floor"),
        Patch(facecolor=colors[1], label="cautious / high-cost cells"),
        Line2D([0], [0], color="#d64045", lw=1.5, label="path"),
        Line2D([0], [0], marker="o", color="w", markerfacecolor="#2f9e44", markersize=8, label="dock/start"),
        Line2D([0], [0], marker="o", color="w", markerfacecolor="#f08c00", markersize=8, label="final_pose"),
    ]
    ax.legend(handles=legend_items, loc="upper right", fontsize=8)
    fig.tight_layout()
    fig.savefig(image_path, dpi=150)
    plt.close(fig)
    return image_path


def save_trajectory_order_map(
    grid: GridMap2D5,
    path: list[CellPos],
    output_dir: str | Path,
    metrics: SimulationMetrics | None = None,
    planner_mode: str = "",
    task_type: str = "full_coverage",
) -> Path:
    out = Path(output_dir)
    out.mkdir(parents=True, exist_ok=True)
    image_path = out / "trajectory_order_map.png"

    fig, ax = plt.subplots(figsize=(10, 7))
    colors = _draw_base(ax, grid)
    segments = _path_segments(path)
    if len(segments):
        values = np.linspace(0, 1, len(segments))
        collection = LineCollection(segments, cmap="viridis", array=values, linewidths=1.4, alpha=0.9)
        ax.add_collection(collection)
        fig.colorbar(collection, ax=ax, fraction=0.035, pad=0.02, label="trajectory order")
        for index in _sparse_arrow_indices(path):
            start = path[index]
            end = path[index + 1]
            ax.annotate(
                "",
                xy=end,
                xytext=start,
                arrowprops={"arrowstyle": "->", "color": "#333333", "lw": 0.7, "alpha": 0.65},
            )
        ax.scatter([path[0][0]], [path[0][1]], c="#2f9e44", s=70)
        ax.scatter([path[-1][0]], [path[-1][1]], c="#f08c00", s=70)

    ax.set_title(_title(grid, planner_mode, task_type, metrics, "trajectory_order"))
    legend_items = [
        Patch(facecolor=colors[5], label="obstacle / blocked"),
        Patch(facecolor=colors[0], label="normal floor"),
        Patch(facecolor=colors[1], label="cautious / high-cost cells"),
        Line2D([0], [0], marker="o", color="w", markerfacecolor="#2f9e44", markersize=8, label="dock/start"),
        Line2D([0], [0], marker="o", color="w", markerfacecolor="#f08c00", markersize=8, label="final_pose"),
    ]
    ax.legend(handles=legend_items, loc="upper right", fontsize=8)
    fig.tight_layout()
    fig.savefig(image_path, dpi=150)
    plt.close(fig)
    return image_path


def save_recovery_debug_map(
    grid: GridMap2D5,
    log_rows: list[dict[str, int | float | str]],
    output_dir: str | Path,
    metrics: SimulationMetrics | None = None,
    planner_mode: str = "",
    task_type: str = "full_coverage",
) -> Path:
    out = Path(output_dir)
    out.mkdir(parents=True, exist_ok=True)
    image_path = out / "recovery_debug_map.png"

    points = [(int(row["x"]), int(row["y"]), str(row["action"])) for row in log_rows]
    fig, ax = plt.subplots(figsize=(10, 7))
    colors = _draw_base(ax, grid)

    for (x0, y0, action0), (x1, y1, action1) in zip(points, points[1:]):
        action = action1 or action0
        if action == "recover_backtrack":
            color = "#2f80ed"
            width = 1.8
            alpha = 0.95
        elif action == "recover_branch":
            color = "#f08c00"
            width = 1.8
            alpha = 0.95
        else:
            color = "#777777"
            width = 0.9
            alpha = 0.35
        ax.plot([x0, x1], [y0, y1], color=color, linewidth=width, alpha=alpha)

    recovery_points = [(x, y) for x, y, action in points if action in {"recover_backtrack", "recover_branch"}]
    if recovery_points:
        ax.scatter(
            [x for x, _ in recovery_points],
            [y for _, y in recovery_points],
            c="#2f80ed",
            s=18,
            alpha=0.7,
            marker="o",
        )

    if points:
        ax.scatter([points[0][0]], [points[0][1]], c="#2f9e44", s=70)
        ax.scatter([points[-1][0]], [points[-1][1]], c="#f08c00", s=70)

    suffix = "recovery_debug" if recovery_points else "recovery_debug | no recovery events"
    ax.set_title(_title(grid, planner_mode, task_type, metrics, suffix))
    legend_items = [
        Patch(facecolor=colors[5], label="obstacle / blocked"),
        Patch(facecolor=colors[0], label="normal floor"),
        Patch(facecolor=colors[1], label="cautious / high-cost cells"),
        Line2D([0], [0], color="#777777", lw=1.2, label="coverage / transit"),
        Line2D([0], [0], color="#2f80ed", lw=1.8, label="recovery backtrack"),
        Line2D([0], [0], color="#f08c00", lw=1.8, label="recovery branch"),
        Line2D([0], [0], marker="o", color="w", markerfacecolor="#2f9e44", markersize=8, label="dock/start"),
        Line2D([0], [0], marker="o", color="w", markerfacecolor="#f08c00", markersize=8, label="final_pose"),
    ]
    ax.legend(handles=legend_items, loc="upper right", fontsize=8)
    fig.tight_layout()
    fig.savefig(image_path, dpi=150)
    plt.close(fig)
    return image_path
