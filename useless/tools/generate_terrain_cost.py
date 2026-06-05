#!/usr/bin/env python3
"""Regenerate maps/terrain_cost.pgm with random high-cost (grass) circles."""

from __future__ import annotations

import argparse
from pathlib import Path

import numpy as np
from PIL import Image


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--maps-dir",
        type=Path,
        default=Path(__file__).resolve().parents[1] / "maps",
    )
    parser.add_argument("--seed", type=int, default=42)
    parser.add_argument("--circles", type=int, default=18)
    parser.add_argument("--min-radius-m", type=float, default=0.4)
    parser.add_argument("--max-radius-m", type=float, default=1.1)
    parser.add_argument("--grass-value", type=int, default=200, help="PGM pixel for grass")
    args = parser.parse_args()

    resolution = 0.05
    arena_path = args.maps_dir / "arena_map.pgm"
    out_path = args.maps_dir / "terrain_cost.pgm"

    arena = np.array(Image.open(arena_path))
    height, width = arena.shape
    free_mask = arena >= 196

    cost = np.zeros((height, width), dtype=np.uint8)
    rng = np.random.default_rng(args.seed)
    min_r_px = max(1, int(args.min_radius_m / resolution))
    max_r_px = max(min_r_px, int(args.max_radius_m / resolution))

    free_indices = np.argwhere(free_mask)
    placed = 0
    attempts = 0
    while placed < args.circles and attempts < 500:
        attempts += 1
        cy, cx = free_indices[rng.integers(0, len(free_indices))]
        radius = int(rng.integers(min_r_px, max_r_px + 1))
        y0, y1 = max(0, cy - radius), min(height, cy + radius + 1)
        x0, x1 = max(0, cx - radius), min(width, cx + radius + 1)
        yy, xx = np.ogrid[y0:y1, x0:x1]
        disk = (yy - cy) ** 2 + (xx - cx) ** 2 <= radius ** 2
        grass = disk & free_mask[y0:y1, x0:x1]
        if not np.any(grass):
            continue
        patch = cost[y0:y1, x0:x1]
        patch[grass] = np.uint8(args.grass_value)
        cost[y0:y1, x0:x1] = patch
        placed += 1

    Image.fromarray(cost, mode="L").save(out_path)
    print(f"Wrote {out_path} ({width}x{height}), circles={placed}, grass_cells={np.sum(cost == args.grass_value)}")


if __name__ == "__main__":
    main()
