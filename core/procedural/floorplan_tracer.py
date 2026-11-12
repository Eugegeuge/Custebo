"""
Floorplan 2D-to-3D Wall Tracer & ROS SLAM `.pgm` Map-to-3D Wall Reconstructor
(`core/procedural/floorplan_tracer.py`).

Supports:
  1. Extruding a sequence of 2D polyline points `[(x0, y0), (x1, y1), ...]` into 3D wall segments
     (regular walls, doorway walls, or window walls) with exact length, yaw angle, thickness, and height.
  2. Importing a ROS 2 2D Occupancy Grid `.pgm` (P5 binary or P2 ASCII) + resolution (m/px)
     and automatically merging occupied black pixels into clean 3D wall boxes!
"""

from __future__ import annotations

import math
from typing import Any, Dict, List, Sequence, Tuple

from core.math_physics.transforms import clean_float
from core.schema.presets_library import create_archetype_node
from core.schema.scene_graph import create_primitive_node


def extrude_polyline_to_walls(
    points_xy: Sequence[Sequence[float]],
    thickness: float = 0.20,
    height: float = 2.50,
    closed_loop: bool = False,
    wall_type: str = "wall",
    base_name: str = "muro_trazado",
) -> List[Dict[str, Any]]:
    """
    Convert a list of 2D points `[(x0, y0), (x1, y1), ...]` in meters on the XY plane
    into a list of 3D wall nodes (`wall`, `wall_doorway`, or `wall_window`).
    """
    pts = [(float(p[0]), float(p[1])) for p in points_xy]
    if len(pts) < 2:
        return []

    if closed_loop and pts[0] != pts[-1]:
        pts.append(pts[0])

    walls: List[Dict[str, Any]] = []
    for i in range(len(pts) - 1):
        x0, y0 = pts[i]
        x1, y1 = pts[i + 1]
        dx = x1 - x0
        dy = y1 - y0
        seg_len = math.hypot(dx, dy)
        if seg_len < 0.05:
            continue

        cx = (x0 + x1) * 0.5
        cy = (y0 + y1) * 0.5
        cz = height * 0.5
        yaw = math.atan2(dy, dx)

        arch = wall_type if wall_type in ("wall", "wall_doorway", "wall_window") else "wall"
        if arch in ("wall_doorway", "wall_window") and seg_len < 1.4:
            arch = "wall"

        node = create_archetype_node(
            archetype=arch,
            name=f"{base_name}_{i + 1}",
            pose=[
                clean_float(cx, 5),
                clean_float(cy, 5),
                clean_float(cz, 5),
                0.0,
                0.0,
                clean_float(yaw, 6),
            ],
            custom_params={
                "length": clean_float(seg_len, 4),
                "thickness": clean_float(thickness, 4),
                "height": clean_float(height, 4),
            },
        )
        walls.append(node)

    return walls


def parse_pgm_bytes(raw_bytes: bytes) -> Tuple[int, int, int, List[List[int]]]:
    """
    Parse a P2 (ASCII) or P5 (Binary) `.pgm` image into `(width, height, maxval, grid_2d)`.
    """
    idx = 0
    n = len(raw_bytes)

    def read_token() -> bytes:
        nonlocal idx
        while idx < n:
            b = raw_bytes[idx : idx + 1]
            if b == b"#":
                while idx < n and raw_bytes[idx : idx + 1] not in (b"\n", b"\r"):
                    idx += 1
            elif b in (b" ", b"\t", b"\n", b"\r"):
                idx += 1
            else:
                break
        start = idx
        while idx < n and raw_bytes[idx : idx + 1] not in (b" ", b"\t", b"\n", b"\r", b"#"):
            idx += 1
        return raw_bytes[start:idx]

    magic = read_token()
    if magic not in (b"P5", b"P2"):
        raise ValueError(f"Unsupported PGM format '{magic!r}' (expected P5 or P2).")

    width = int(read_token())
    height = int(read_token())
    maxval = int(read_token())
    # Skip single whitespace byte after maxval
    if idx < n and raw_bytes[idx : idx + 1] in (b" ", b"\t", b"\n", b"\r"):
        idx += 1

    grid: List[List[int]] = []
    if magic == b"P5":
        for r in range(height):
            row = list(raw_bytes[idx : idx + width])
            if len(row) < width:
                row.extend([255] * (width - len(row)))
            grid.append(row)
            idx += width
    else:
        for r in range(height):
            row = []
            for _ in range(width):
                tok = read_token()
                row.append(int(tok) if tok else 255)
            grid.append(row)

    return width, height, maxval, grid


def reconstruct_3d_walls_from_pgm(
    pgm_bytes: bytes,
    resolution: float = 0.05,
    origin_xy: Tuple[float, float] = (-5.0, -5.0),
    wall_height: float = 2.50,
    occupied_threshold: int = 80,
    max_walls: int = 80,
) -> List[Dict[str, Any]]:
    """
    Convert a 2D ROS SLAM occupancy grid `.pgm` into merged 3D wall boxes in ENU coordinates.
    Uses greedy 2D rectangle merging on occupied cells so an entire wall becomes a single
    clean `<model>` box instead of thousands of tiny pixel cubes!
    """
    width, height, _, grid = parse_pgm_bytes(pgm_bytes)
    visited = [[False] * width for _ in range(height)]
    rectangles: List[Tuple[int, int, int, int]] = []  # (r0, c0, r1_excl, c1_excl)

    for r in range(height):
        for c in range(width):
            if visited[r][c] or grid[r][c] > occupied_threshold:
                continue
            # Expand horizontally first
            c_end = c + 1
            while c_end < width and not visited[r][c_end] and grid[r][c_end] <= occupied_threshold:
                c_end += 1

            # Expand vertically while the entire row slice [c : c_end] is occupied
            r_end = r + 1
            while r_end < height:
                row_ok = all(
                    not visited[r_end][cc] and grid[r_end][cc] <= occupied_threshold
                    for cc in range(c, c_end)
                )
                if not row_ok:
                    break
                r_end += 1

            for rr in range(r, r_end):
                for cc in range(c, c_end):
                    visited[rr][cc] = True

            rectangles.append((r, c, r_end, c_end))

    # Sort by area descending and keep up to `max_walls` significant wall blocks
    rectangles.sort(key=lambda rect: (rect[2] - rect[0]) * (rect[3] - rect[1]), reverse=True)
    nodes: List[Dict[str, Any]] = []

    ox, oy = float(origin_xy[0]), float(origin_xy[1])
    res = max(float(resolution), 0.01)

    for idx, (r0, c0, r1, c1) in enumerate(rectangles[:max_walls]):
        cell_w = c1 - c0
        cell_h = r1 - r0
        if cell_w * cell_h < 2 and len(rectangles) > 10:
            continue

        # In ROS map_server, row 0 is top (+Y max), row (height - 1) is bottom (origin_y)
        dx = max(cell_w * res, 0.10)
        dy = max(cell_h * res, 0.10)
        cx = ox + (c0 + cell_w * 0.5) * res
        cy = oy + ((height - r1) + cell_h * 0.5) * res
        cz = wall_height * 0.5

        wall_node = create_primitive_node(
            name=f"pgm_wall_{idx + 1}",
            shape="box",
            pose=[clean_float(cx, 4), clean_float(cy, 4), clean_float(cz, 4), 0.0, 0.0, 0.0],
            size=[clean_float(dx, 4), clean_float(dy, 4), clean_float(wall_height, 4)],
            color=[0.76, 0.79, 0.83, 1.0],
            static=True,
            material_preset="concrete",
            folder_id="folder_arch",
        )
        nodes.append(wall_node)

    return nodes

