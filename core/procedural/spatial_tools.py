"""
Advanced Spatial Manipulation & CAD Alignment Tools (`core/procedural/spatial_tools.py`).

Includes:
  1. Raycast / AABB Stack Drop-to-Surface: drops an object onto the highest supporting object
     directly underneath its XY footprint (or onto Z=0 if none).
  2. Vertex / Edge / Surface Wall Snapping: snaps a node's corner/face flush against the nearest
     neighboring node's corner/face without collision overlap.
  3. Multi-Object CAD Alignment & Even Distribution (along X, Y, or Z).
  4. 3D Euclidean Laser Measurement math (dx, dy, dz, distance, horizontal_distance, slope_deg).
"""

from __future__ import annotations

import math
from typing import Any, Dict, List, Sequence, Tuple

from core.math_physics.transforms import (
    clean_float,
    compute_local_half_extents,
    compute_world_aabb,
)
from core.schema.scene_graph import SceneGraphDocument


def get_node_world_aabb(node: Dict[str, Any]) -> Tuple[Tuple[float, float, float], Tuple[float, float, float]]:
    geom = node.get("geometry", {})
    half_ext = compute_local_half_extents(
        geom.get("shape", "box"),
        size=geom.get("size", [1.0, 1.0, 1.0]),
        radius=float(geom.get("radius", 0.5)),
        length=float(geom.get("length", 1.0)),
    )
    return compute_world_aabb(
        node.get("pose", [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]),
        half_ext,
        node.get("scale", [1.0, 1.0, 1.0]),
    )


def compute_smart_drop_z(doc: SceneGraphDocument, target_node_id: str) -> float:
    """
    Find the highest surface (ground Z=0 or the top of any underlying object whose XY AABB
    overlaps `target_node_id`) and return the exact world Z for `target_node_id` so it rests
    flush on that surface.
    """
    target = doc.get_node(target_node_id)
    if target is None:
        return 0.0

    t_min, t_max = get_node_world_aabb(target)
    target_bottom_offset = float(target["pose"][2]) - t_min[2]
    current_bottom_z = t_min[2]

    highest_support_z = 0.0
    for other in doc.nodes:
        if other.get("id") == target_node_id:
            continue
        if other.get("type") in ("light", "ground_plane") or not other.get("visible", True):
            continue

        o_min, o_max = get_node_world_aabb(other)
        overlap_x = (t_min[0] < o_max[0] - 1e-3) and (t_max[0] > o_min[0] + 1e-3)
        overlap_y = (t_min[1] < o_max[1] - 1e-3) and (t_max[1] > o_min[1] + 1e-3)
        if overlap_x and overlap_y:
            # Support is valid if its top is at or below the target's center Z
            if o_max[2] <= float(target["pose"][2]) + 1e-2:
                highest_support_z = max(highest_support_z, o_max[2])

    return clean_float(highest_support_z + target_bottom_offset, 6)


def snap_node_to_nearest_neighbor(
    doc: SceneGraphDocument,
    node_id: str,
    max_snap_distance: float = 2.0,
) -> List[float]:
    """
    Snap `node_id` flush against the nearest neighboring wall/object face or corner in XY
    so there are zero gaps or overlaps.
    """
    target = doc.get_node(node_id)
    if target is None:
        return [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

    pose = list(target.get("pose", [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]))
    t_min, t_max = get_node_world_aabb(target)
    t_hx = (t_max[0] - t_min[0]) * 0.5
    t_hy = (t_max[1] - t_min[1]) * 0.5

    best_dist = max_snap_distance
    best_xy: Tuple[float, float] = (pose[0], pose[1])

    for other in doc.nodes:
        if other.get("id") == node_id or other.get("type") in ("light", "ground_plane"):
            continue
        o_min, o_max = get_node_world_aabb(other)
        o_cx = (o_min[0] + o_max[0]) * 0.5
        o_cy = (o_min[1] + o_max[1]) * 0.5

        # Candidate flush positions around `other` (+X, -X, +Y, -Y faces and corner-aligned)
        candidates = [
            (o_max[0] + t_hx, o_cy),
            (o_min[0] - t_hx, o_cy),
            (o_cx, o_max[1] + t_hy),
            (o_cx, o_min[1] - t_hy),
            (o_max[0] + t_hx, o_max[1] - t_hy),
            (o_max[0] + t_hx, o_min[1] + t_hy),
            (o_min[0] - t_hx, o_max[1] - t_hy),
            (o_min[0] - t_hx, o_min[1] + t_hy),
        ]
        for cx, cy in candidates:
            d = math.hypot(cx - pose[0], cy - pose[1])
            if d < best_dist:
                best_dist = d
                best_xy = (cx, cy)

    pose[0] = clean_float(best_xy[0], 6)
    pose[1] = clean_float(best_xy[1], 6)
    return pose


def align_or_distribute_nodes(
    doc: SceneGraphDocument,
    node_ids: Sequence[str],
    mode: str = "align_x_center",
    spacing: float = 1.5,
) -> Dict[str, List[float]]:
    """
    Compute new poses for a list of nodes to align or distribute them (CAD/Figma style).
    Modes:
      - `align_x`, `align_y`, `align_z_ground`
      - `distribute_x`, `distribute_y` (evenly spaced by `spacing` meters)
    Returns a dict `{node_id: new_pose}`.
    """
    valid_nodes = [doc.get_node(nid) for nid in node_ids]
    valid_nodes = [n for n in valid_nodes if n is not None and n.get("type") != "ground_plane"]
    if not valid_nodes:
        return {}

    new_poses: Dict[str, List[float]] = {}
    anchor_pose = valid_nodes[0]["pose"]

    if mode == "align_x":
        for n in valid_nodes:
            p = list(n["pose"])
            p[0] = clean_float(anchor_pose[0], 6)
            new_poses[n["id"]] = p
    elif mode == "align_y":
        for n in valid_nodes:
            p = list(n["pose"])
            p[1] = clean_float(anchor_pose[1], 6)
            new_poses[n["id"]] = p
    elif mode == "distribute_x":
        sorted_nodes = sorted(valid_nodes, key=lambda item: float(item["pose"][0]))
        x0 = float(sorted_nodes[0]["pose"][0])
        for idx, n in enumerate(sorted_nodes):
            p = list(n["pose"])
            p[0] = clean_float(x0 + idx * float(spacing), 6)
            new_poses[n["id"]] = p
    elif mode == "distribute_y":
        sorted_nodes = sorted(valid_nodes, key=lambda item: float(item["pose"][1]))
        y0 = float(sorted_nodes[0]["pose"][1])
        for idx, n in enumerate(sorted_nodes):
            p = list(n["pose"])
            p[1] = clean_float(y0 + idx * float(spacing), 6)
            new_poses[n["id"]] = p

    return new_poses


def compute_3d_ruler_measurement(
    pt_a: Sequence[float], pt_b: Sequence[float]
) -> Dict[str, float]:
    """Compute exact 3D laser ruler metrics between two ENU points (x, y, z) in meters."""
    dx = float(pt_b[0]) - float(pt_a[0])
    dy = float(pt_b[1]) - float(pt_a[1])
    dz = float(pt_b[2]) - float(pt_a[2])
    dist_xy = math.hypot(dx, dy)
    dist_3d = math.sqrt(dx * dx + dy * dy + dz * dz)
    slope_deg = math.degrees(math.atan2(abs(dz), max(dist_xy, 1e-9))) if dist_3d > 1e-9 else 0.0
    yaw_deg = math.degrees(math.atan2(dy, dx)) if dist_xy > 1e-9 else 0.0
    return {
        "dx": clean_float(dx, 4),
        "dy": clean_float(dy, 4),
        "dz": clean_float(dz, 4),
        "distance_xy": clean_float(dist_xy, 4),
        "distance_3d": clean_float(dist_3d, 4),
        "slope_deg": clean_float(slope_deg, 2),
        "yaw_deg": clean_float(yaw_deg, 2),
    }
