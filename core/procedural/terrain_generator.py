"""
Parametric Terrain & Heightmap Generator (`core/procedural/terrain_generator.py`).

Generates:
  1. A multi-cell 3D terrain relief node (`terrain_patch`) with configurable hills/dunes/ramps
     for testing outdoor off-road robot navigation.
  2. Exports a corresponding 8-bit/16-bit grayscale `.pgm` heightmap image in `assets/heightmaps/`.
"""

from __future__ import annotations

import math
from pathlib import Path
from typing import Any, Dict, List

from core.math_physics.transforms import clean_float
from core.schema.scene_graph import generate_node_id, normalize_node_in_place


def create_parametric_terrain_node(
    name: str = "terreno_irregular",
    size_x: float = 12.0,
    size_y: float = 12.0,
    max_height: float = 0.80,
    grid_res: int = 8,
    profile: str = "hills",  # "hills" | "crater" | "ridge"
    heightmaps_dir: Path | None = None,
) -> Dict[str, Any]:
    """
    Build a composite terrain patch made of smooth stepped relief blocks and save
    its corresponding grayscale `.pgm` heightmap.
    """
    nx = max(min(int(grid_res), 16), 3)
    ny = max(min(int(grid_res), 16), 3)
    sx = max(float(size_x), 2.0)
    sy = max(float(size_y), 2.0)
    mh = max(float(max_height), 0.10)

    cell_w = sx / float(nx)
    cell_d = sy / float(ny)

    parts: List[Dict[str, Any]] = []
    pgm_rows: List[List[int]] = []

    for iy in range(ny):
        v_norm = (iy + 0.5) / float(ny) * 2.0 - 1.0  # [-1, 1]
        row_pixels: List[int] = []
        for ix in range(nx):
            u_norm = (ix + 0.5) / float(nx) * 2.0 - 1.0  # [-1, 1]
            r2 = u_norm * u_norm + v_norm * v_norm

            if profile == "crater":
                h_ratio = min(max(abs(math.sqrt(r2) - 0.65) * 1.8, 0.08), 1.0)
            elif profile == "ridge":
                h_ratio = max(0.08, math.exp(-3.0 * (u_norm * u_norm)))
            else:
                # Smooth sinusoidal hills
                h_ratio = 0.5 + 0.45 * math.cos(u_norm * math.pi) * math.cos(v_norm * math.pi)
                h_ratio = max(0.08, min(1.0, h_ratio))

            cell_h = max(clean_float(h_ratio * mh, 4), 0.04)
            px = -sx * 0.5 + (ix + 0.5) * cell_w
            py = -sy * 0.5 + (iy + 0.5) * cell_d
            # Base rests at local z = -mh/2
            pz = -mh * 0.5 + cell_h * 0.5

            # Earthy / sandy terrain color gradient by elevation
            color = [
                clean_float(0.38 + 0.25 * h_ratio, 3),
                clean_float(0.48 + 0.14 * h_ratio, 3),
                clean_float(0.28 + 0.08 * h_ratio, 3),
                1.0,
            ]
            parts.append(
                {
                    "name": f"cell_{ix}_{iy}",
                    "shape": "box",
                    "local_pose": [clean_float(px, 4), clean_float(py, 4), clean_float(pz, 4), 0.0, 0.0, 0.0],
                    "size": [clean_float(cell_w, 4), clean_float(cell_d, 4), clean_float(cell_h, 4)],
                    "radius": 0.1,
                    "length": 1.0,
                    "color": color,
                }
            )
            row_pixels.append(min(255, max(0, int(round(h_ratio * 255.0)))))
        pgm_rows.append(row_pixels)

    if heightmaps_dir is not None:
        heightmaps_dir.mkdir(parents=True, exist_ok=True)
        pgm_path = heightmaps_dir / f"{name}.pgm"
        header = f"P5\n{nx} {ny}\n255\n".encode("ascii")
        body = bytes(px_val for row in pgm_rows for px_val in row)
        pgm_path.write_bytes(header + body)

    node: Dict[str, Any] = {
        "id": generate_node_id("terrain"),
        "name": name,
        "type": "primitive",
        "folder_id": "folder_env",
        "visible": True,
        "locked": False,
        "static": True,
        "pose": [0.0, 0.0, clean_float(mh * 0.5, 4), 0.0, 0.0, 0.0],
        "scale": [1.0, 1.0, 1.0],
        "geometry": {"shape": "box", "size": [sx, sy, mh]},
        "material": {"color": [0.46, 0.54, 0.32, 1.0], "roughness": 0.9, "metalness": 0.05},
        "collision": {"enabled": True, "strategy": "primitive", "surface_preset": "dirt", "mu1": 0.75, "mu2": 0.75},
        "parts": parts,
    }
    return normalize_node_in_place(node)
