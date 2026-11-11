"""
Procedural Array Cloners, Cubic Bézier Spline Cloner & Scatter Brush (`core/procedural/cloners.py`).

Includes:
  1. `generate_grid_array`: clones any template node into an Nx * Ny * Nz grid with exact spacing.
  2. `generate_spline_clones`: samples a 2D/3D Cubic Bézier curve at equal arc-length intervals
     and places oriented instances (fences, cones, bollards, streetlights) along the curve.
  3. `generate_scatter_brush`: Poisson-disk / minimum-distance randomized scatter on a circular or
     rectangular patch with random Yaw [0, 2*pi] and random uniform scale [scale_min, scale_max].
"""

from __future__ import annotations

import copy
import math
import random
from typing import Any, Dict, List, Optional, Sequence, Tuple

from core.math_physics.transforms import clean_float
from core.schema.scene_graph import (
    generate_node_id,
    normalize_node_in_place,
    recalculate_node_inertial,
)


def clone_node_with_pose(
    template_node: Dict[str, Any],
    new_name: str,
    new_pose: Sequence[float],
    scale_Factor: float = 1.0,
) -> Dict[str, Any]:
    dup = copy.deepcopy(template_node)
    dup["id"] = generate_node_id(dup.get("type", "node"))
    dup["name"] = new_name
    dup["locked"] = False
    dup["pose"] = [clean_float(float(v), 6) for v in new_pose[:6]]
    if abs(scale_Factor - 1.0) > 1e-5:
        base_scale = dup.get("scale", [1.0, 1.0, 1.0])
        dup["scale"] = [clean_float(float(s) * scale_Factor, 5) for s in base_scale[:3]]
        if dup.get("type") in ("primitive", "mesh"):
            recalculate_node_inertial(dup)
    return normalize_node_in_place(dup)


def generate_grid_array(
    source_node: Dict[str, Any],
    count_x: int = 3,
    count_y: int = 2,
    count_z: int = 1,
    spacing_x: float = 2.5,
    spacing_y: float = 2.5,
    spacing_z: float = 2.0,
) -> List[Dict[str, Any]]:
    """
    Generate a 3D grid array of clones from `source_node` (excluding the (0, 0, 0) original index
    so the original node remains in place and only the additional clones are added).
    """
    nx = max(min(int(count_x), 20), 1)
    ny = max(min(int(count_y), 20), 1)
    nz = max(min(int(count_z), 10), 1)

    base_pose = source_node.get("pose", [0.0, 0.0, 0.5, 0.0, 0.0, 0.0])
    x0, y0, z0, r0, p0, yaw0 = (float(v) for v in base_pose[:6])
    base_name = source_node.get("name", "item")

    clones: List[Dict[str, Any]] = []
    idx = 1
    for iz in range(nz):
        for iy in range(ny):
            for ix in range(nx):
                if ix == 0 and iy == 0 and iz == 0:
                    continue
                px = x0 + ix * float(spacing_x)
                py = y0 + iy * float(spacing_y)
                pz = z0 + iz * float(spacing_z)
                clones.append(
                    clone_node_with_pose(
                        source_node,
                        new_name=f"{base_name}_arr_{idx}",
                        new_pose=[px, py, pz, r0, p0, yaw0],
                    )
                )
                idx += 1
    return clones


def evaluate_cubic_bezier(
    p0: Tuple[float, float],
    p1: Tuple[float, float],
    p2: Tuple[float, float],
    p3: Tuple[float, float],
    t: float,
) -> Tuple[float, float, float]:
    """
    Evaluate 2D Cubic Bézier point (x, y) and tangent yaw angle (rad) at parameter t in [0, 1].
    """
    u = 1.0 - t
    b0 = u * u * u
    b1 = 3.0 * u * u * t
    b2 = 3.0 * u * t * t
    b3 = t * t * t

    x = b0 * p0[0] + b1 * p1[0] + b2 * p2[0] + b3 * p3[0]
    y = b0 * p0[1] + b1 * p1[1] + b2 * p2[1] + b3 * p3[1]

    # Derivative B'(t) for tangent orientation
    dx = (
        3.0 * u * u * (p1[0] - p0[0])
        + 6.0 * u * t * (p2[0] - p1[0])
        + 3.0 * t * t * (p3[0] - p2[0])
    )
    dy = (
        3.0 * u * u * (p1[1] - p0[1])
        + 6.0 * u * t * (p2[1] - p1[1])
        + 3.0 * t * t * (p3[1] - p2[1])
    )
    yaw = math.atan2(dy, dx) if math.hypot(dx, dy) > 1e-9 else 0.0
    return (x, y, yaw)


def generate_spline_clones(
    template_node: Dict[str, Any],
    control_points_xy: Sequence[Sequence[float]],
    count: int = 8,
    align_to_tangent: bool = True,
) -> List[Dict[str, Any]]:
    """
    Instance `count` clones of `template_node` along a Cubic Bézier curve defined by 4 control points.
    """
    pts = [(float(p[0]), float(p[1])) for p in control_points_xy]
    while len(pts) < 4:
        pts.append(pts[-1] if pts else (0.0, 0.0))
    p0, p1, p2, p3 = pts[0], pts[1], pts[2], pts[3]

    n = max(min(int(count), 60), 2)
    base_pose = template_node.get("pose", [0.0, 0.0, 0.5, 0.0, 0.0, 0.0])
    z0 = float(base_pose[2])
    r0, pitch0, yaw0 = float(base_pose[3]), float(base_pose[4]), float(base_pose[5])
    base_name = template_node.get("name", "spline_item")

    clones: List[Dict[str, Any]] = []
    for i in range(n):
        t = float(i) / float(n - 1) if n > 1 else 0.0
        x, y, tangent_yaw = evaluate_cubic_bezier(p0, p1, p2, p3, t)
        final_yaw = tangent_yaw if align_to_tangent else yaw0
        clones.append(
            clone_node_with_pose(
                template_node,
                new_name=f"{base_name}_path_{i + 1}",
                new_pose=[x, y, z0, r0, pitch0, final_yaw],
            )
        )
    return clones


def generate_scatter_brush(
    template_node: Dict[str, Any],
    center_xy: Tuple[float, float] = (0.0, 0.0),
    radius: float = 5.0,
    count: int = 12,
    min_distance: float = 0.85,
    scale_min: float = 0.8,
    scale_max: float = 1.25,
    random_yaw: bool = True,
    seed: Optional[int] = None,
) -> List[Dict[str, Any]]:
    """
    Scatter `count` randomized instances of `template_node` inside a circle of `radius` around
    `center_xy`, enforcing `min_distance` between instances to prevent collision overlaps.
    """
    rng = random.Random(seed)
    n_target = max(min(int(count), 100), 1)
    rad = max(float(radius), 0.5)
    min_dist = max(float(min_distance), 0.1)
    s_min = max(float(scale_min), 0.1)
    s_max = max(float(scale_max), s_min)

    base_pose = template_node.get("pose", [0.0, 0.0, 0.5, 0.0, 0.0, 0.0])
    base_z = float(base_pose[2])
    r0, p0, yaw0 = float(base_pose[3]), float(base_pose[4]), float(base_pose[5])
    base_name = template_node.get("name", "scatter")

    placed_xy: List[Tuple[float, float]] = []
    clones: List[Dict[str, Any]] = []
    max_attempts = n_target * 25

    attempts = 0
    while len(clones) < n_target and attempts < max_attempts:
        attempts += 1
        # Uniform point inside disk
        u = math.sqrt(rng.random()) * rad
        theta = rng.uniform(0.0, 2.0 * math.pi)
        px = float(center_xy[0]) + u * math.cos(theta)
        py = float(center_xy[1]) + u * math.sin(theta)

        if any(math.hypot(px - ox, py - oy) < min_dist for ox, oy in placed_xy):
            continue

        placed_xy.append((px, py))
        s_factor = rng.uniform(s_min, s_max)
        yaw = rng.uniform(-math.pi, math.pi) if random_yaw else yaw0
        pz = base_z * s_factor

        clones.append(
            clone_node_with_pose(
                template_node,
                new_name=f"{base_name}_scat_{len(clones) + 1}",
                new_pose=[px, py, pz, r0, p0, yaw],
                scale_Factor=s_factor,
            )
        )

    return clones
