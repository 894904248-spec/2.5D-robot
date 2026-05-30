from __future__ import annotations

import os
from pathlib import Path
import tempfile

os.environ.setdefault("MPLCONFIGDIR", str(Path(tempfile.gettempdir()) / "robot_nav_mvp_mpl"))

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.colors import ListedColormap
from matplotlib.lines import Line2D
from matplotlib.patches import Patch

from .map2d5 import BLOCKED_SEMANTICS, CellPos, GridMap2D5

SEMANTIC_VALUES = {
    "floor": 0,
    "carpet": 1,
    "low_bump": 2,
    "curtain_soft": 3,
    "unknown": 4,
}


def save_final_map(
    grid: GridMap2D5,
    visited: set[CellPos],
    path: list[CellPos],
    output_dir: str | Path,
) -> Path:
    out = Path(output_dir)
    out.mkdir(parents=True, exist_ok=True)
    image_path = out / "final_map.png"

    data = np.zeros((grid.height, grid.width), dtype=float)
    for y in range(grid.height):
        for x in range(grid.width):
            cell = grid.cell(x, y)
            if cell.occupied or cell.semantic in BLOCKED_SEMANTICS:
                data[y, x] = 5
            else:
                data[y, x] = SEMANTIC_VALUES.get(cell.semantic, 4)

    colors = ["#f7f7f2", "#d7bd82", "#b9d99c", "#9bd7d5", "#c8c8c8", "#242424"]
    cmap = ListedColormap(colors)
    fig, ax = plt.subplots(figsize=(10, 7))
    ax.imshow(data, cmap=cmap, origin="upper", vmin=0, vmax=5)

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

    ax.set_title(f"{grid.name} coverage result")
    ax.set_xticks(range(grid.width))
    ax.set_yticks(range(grid.height))
    ax.grid(color="#dddddd", linewidth=0.4)
    ax.set_xlim(-0.5, grid.width - 0.5)
    ax.set_ylim(grid.height - 0.5, -0.5)
    ax.set_aspect("equal")
    legend_items = [
        Patch(facecolor=colors[5], label="obstacle / blocked"),
        Patch(facecolor=colors[0], label="normal floor"),
        Patch(facecolor=colors[1], label="carpet / high-cost area"),
        Patch(facecolor=colors[2], label="low-bump / cautious area"),
        Line2D([0], [0], color="#d64045", lw=1.5, label="path"),
        Line2D([0], [0], marker="o", color="w", markerfacecolor="#2f9e44", markersize=8, label="start"),
        Line2D([0], [0], marker="o", color="w", markerfacecolor="#f08c00", markersize=8, label="end"),
    ]
    ax.legend(handles=legend_items, loc="upper right", fontsize=8)
    fig.tight_layout()
    fig.savefig(image_path, dpi=150)
    plt.close(fig)
    return image_path
