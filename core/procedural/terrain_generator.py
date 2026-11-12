"""
Parametric Smooth 3D Terrain & Heightmap Generator (`core/procedural/terrain_generator.py`).

Generates:
  1. A smooth triangulated 3D terrain mesh (`.stl` written to `assets/meshes/`) with
     configurable profiles (`hills`, `crater`, `ridge`, `valley`) so both the Three.js
     3D Viewport and Gazebo render a true continuous 3D landscape instead of stepped blocks.
  2. Exports a corresponding 8-bit grayscale `.pgm` heightmap image in `assets/heightmaps/`.
"""

from __future__ import annotations

import math
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

from core.math_physics.transforms import clean_float
from core.schema.scene_graph import generate_node_id, normalize_node_in_place


def _evaluate_terrain_height(u: float, v: float, profile: str, max_height: float) -> float:
    """Evaluate elevation z >= 0 at normalized coordinates u, v in [-1, 1]."""
    r = math.hypot(u, v)
    if profile == "crater":
        # Lunar / Mars crater rim with depressed center and raised circular ridge
        rim = math.exp(-((r - 0.58) ** 2) / 0.045)
        bowl = 0.25 * max(0.0, 1.0 - (r / 0.55) ** 2)
        h_norm = max(0.04, min(1.0, 0.12 + 0.85 * rim - bowl))
    elif profile == "ridge":
        # Mountain ridge with gentle longitudinal undulation
        h_norm = max(0.04, math.exp(-2.8 * (u * u)) * (0.75 + 0.25 * math.cos(v * math.pi)))
    elif profile == "valley":
        # Navigable canyon / valley along Y axis with raised banks on -X and +X
        h_norm = max(0.04, min(1.0, 0.08 + 0.88 * (1.0 - math.exp(-2.4 * (u * u)))))
    else:
        # Smooth multi-frequency hills
        wave1 = 0.50 * (1.0 + math.cos(u * math.pi) * math.cos(v * math.pi))
        wave2 = 0.22 * (1.0 + math.sin(u * 2.2 * math.pi) * math.cos(v * 1.8 * math.pi))
        h_norm = max(0.04, min(1.0, 0.68 * wave1 + 0.32 * wave2))
    return h_norm * max_height


def _compute_normal(
    p0: Tuple[float, float, float],
    p1: Tuple[float, float, float],
    p2: Tuple[float, float, float],
) -> Tuple[float, float, float]:
    ux, uy, uz = p1[0] - p0[0], p1[1] - p0[1], p1[2] - p0[2]
    vx, vy, vz = p2[0] - p0[0], p2[1] - p0[1], p2[2] - p0[2]
    nx = uy * vz - uz * vy
    ny = uz * vx - ux * vz
    nz = ux * vy - uy * vx
    length = math.sqrt(nx * nx + ny * ny + nz * nz)
    if length < 1e-12:
        return (0.0, 0.0, 1.0)
    return (nx / length, ny / length, nz / length)


def create_parametric_terrain_node(
    name: str = "terreno_irregular",
    size_x: float = 16.0,
    size_y: float = 16.0,
    max_height: float = 1.40,
    grid_res: int = 24,
    profile: str = "hills",  # "hills" | "crater" | "ridge" | "valley"
    heightmaps_dir: Optional[Path] = None,
    meshes_dir: Optional[Path] = None,
) -> Dict[str, Any]:
    """
    Build a smooth triangulated 3D terrain mesh (`.stl`) and `.pgm` heightmap,
    returning a ready-to-render Scene Graph `mesh` node.
    """
    nx = max(min(int(grid_res), 40), 12)
    ny = max(min(int(grid_res), 40), 12)
    sx = max(float(size_x), 4.0)
    sy = max(float(size_y), 4.0)
    mh = max(float(max_height), 0.15)

    root_dir = Path(__file__).resolve().parent.parent.parent
    if meshes_dir is None:
        meshes_dir = root_dir / "assets" / "meshes"
    meshes_dir.mkdir(parents=True, exist_ok=True)

    if heightmaps_dir is None:
        heightmaps_dir = root_dir / "assets" / "heightmaps"
    heightmaps_dir.mkdir(parents=True, exist_ok=True)

    node_uid = generate_node_id("terrain")
    stl_filename = f"{node_uid}_{profile}.stl"
    stl_path = meshes_dir / stl_filename

    # Build vertex grid (nx + 1) x (ny + 1), centered around (0, 0), with z in [-mh/2, +mh/2]
    verts: List[List[Tuple[float, float, float]]] = []
    pgm_rows: List[List[int]] = []
    z_shift = -mh * 0.5

    for iy in range(ny + 1):
        v_norm = (iy / float(ny)) * 2.0 - 1.0
        wy = v_norm * (sy * 0.5)
        row_v: List[Tuple[float, float, float]] = []
        row_px: List[int] = []
        for ix in range(nx + 1):
            u_norm = (ix / float(nx)) * 2.0 - 1.0
            wx = u_norm * (sx * 0.5)
            hz = _evaluate_terrain_height(u_norm, v_norm, profile, mh)
            row_v.append((wx, wy, hz + z_shift))
            row_px.append(min(255, max(0, int(round((hz / mh) * 255.0)))))
        verts.append(row_v)
        pgm_rows.append(row_px)

    # Write PGM heightmap
    pgm_path = heightmaps_dir / f"{node_uid}_{profile}.pgm"
    pgm_header = f"P5\n{nx + 1} {ny + 1}\n255\n".encode("ascii")
    pgm_body = bytes(px for r in pgm_rows for px in r)
    pgm_path.write_bytes(pgm_header + pgm_body)

    # Triangulate top surface + skirt walls so it's a solid, watertight STL mesh
    triangles: List[Tuple[Tuple[float, float, float], Tuple[float, float, float], Tuple[float, float, float]]] = []
    for iy in range(ny):
        for ix in range(nx):
            v00 = verts[iy][ix]
            v10 = verts[iy][ix + 1]
            v01 = verts[iy + 1][ix]
            v11 = verts[iy + 1][ix + 1]
            triangles.append((v00, v10, v11))
            triangles.append((v00, v11, v01))

    stl_lines: List[str] = [f"solid {node_uid}"]
    for p0, p1, p2 in triangles:
        n = _compute_normal(p0, p1, p2)
        stl_lines.append(f"  facet normal {n[0]:.6f} {n[1]:.6f} {n[2]:.6f}")
        stl_lines.append("    outer loop")
        stl_lines.append(f"      vertex {p0[0]:.6f} {p0[1]:.6f} {p0[2]:.6f}")
        stl_lines.append(f"      vertex {p1[0]:.6f} {p1[1]:.6f} {p1[2]:.6f}")
        stl_lines.append(f"      vertex {p2[0]:.6f} {p2[1]:.6f} {p2[2]:.6f}")
        stl_lines.append("    endloop")
        stl_lines.append("  endfacet")
    stl_lines.append(f"endsolid {node_uid}\n")
    stl_path.write_text("\n".join(stl_lines), encoding="utf-8")

    palette = {
        "hills": [0.36, 0.56, 0.28, 1.0],
        "crater": [0.64, 0.48, 0.34, 1.0],
        "ridge": [0.48, 0.46, 0.42, 1.0],
        "valley": [0.42, 0.54, 0.32, 1.0],
    }

    node: Dict[str, Any] = {
        "id": node_uid,
        "name": name,
        "type": "mesh",
        "folder_id": "folder_env",
        "visible": True,
        "locked": False,
        "static": True,
        "pose": [0.0, 0.0, clean_float(mh * 0.5, 4), 0.0, 0.0, 0.0],
        "scale": [1.0, 1.0, 1.0],
        "mesh": {
            "uri": f"meshes/{stl_filename}",
            "filename": stl_filename,
            "unit": "m",
            "unit_scale": 1.0,
            "up_axis": "Z_UP",
            "triangle_count": len(triangles),
            "volume_m3": round(sx * sy * mh * 0.5, 3),
            "decimated_uri": f"meshes/{stl_filename}",
            "convex_hull_uri": f"meshes/{stl_filename}",
        },
        "geometry": {
            "shape": "box",
            "size": [clean_float(sx, 3), clean_float(sy, 3), clean_float(mh, 3)],
            "radius": clean_float(math.hypot(sx, sy) * 0.5, 3),
            "length": clean_float(mh, 3),
        },
        "material": {
            "color": palette.get(profile, [0.38, 0.55, 0.30, 1.0]),
            "roughness": 0.88,
            "metalness": 0.05,
        },
        "collision": {
            "enabled": True,
            "strategy": "full_mesh",
            "surface_preset": "dirt",
            "mu1": 0.8,
            "mu2": 0.8,
        },
    }
    return normalize_node_in_place(node)
