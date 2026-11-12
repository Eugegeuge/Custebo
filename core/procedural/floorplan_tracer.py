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


def _rdp_simplify(points: List[Tuple[float, float]], epsilon: float) -> List[Tuple[float, float]]:
    """Ramer-Douglas-Peucker polyline simplification in 2D meter coordinates."""
    if len(points) <= 2:
        return list(points)

    x0, y0 = points[0]
    x1, y1 = points[-1]
    dx = x1 - x0
    dy = y1 - y0
    line_len = math.hypot(dx, dy)

    max_dist = -1.0
    max_idx = 0
    for i in range(1, len(points) - 1):
        px, py = points[i]
        if line_len < 1e-9:
            d = math.hypot(px - x0, py - y0)
        else:
            d = abs(dy * px - dx * py + x1 * y0 - y1 * x0) / line_len
        if d > max_dist:
            max_dist = d
            max_idx = i

    if max_dist > epsilon:
        left = _rdp_simplify(points[: max_idx + 1], epsilon)
        right = _rdp_simplify(points[max_idx:], epsilon)
        return left[:-1] + right
    return [points[0], points[-1]]


def _chaikin_smooth_curve(
    points: List[Tuple[float, float]], closed: bool = False
) -> List[Tuple[float, float]]:
    """Apply 1 pass of Chaikin corner-cutting on curved polylines to eliminate pixel stair-stepping."""
    if len(points) < 3:
        return list(points)
    out: List[Tuple[float, float]] = []
    if not closed:
        out.append(points[0])
    n = len(points)
    limit = n if closed else n - 1
    for i in range(limit):
        p0 = points[i]
        p1 = points[(i + 1) % n]
        q = (0.75 * p0[0] + 0.25 * p1[0], 0.75 * p0[1] + 0.25 * p1[1])
        r = (0.25 * p0[0] + 0.75 * p1[0], 0.25 * p0[1] + 0.75 * p1[1])
        out.append(q)
        out.append(r)
    if not closed:
        out.append(points[-1])
    else:
        out.append(out[0])
    return out


def _skeletonize_binary_mask(mask: List[List[bool]]) -> List[List[bool]]:
    """
    Zhang-Suen morphological thinning to reduce thick occupied wall bands to a 1-pixel
    medial centerline while preserving topology, diagonals, and curves.
    """
    h = len(mask)
    w = len(mask[0]) if h > 0 else 0
    skel = [row[:] for row in mask]

    def neighbors(r: int, c: int) -> List[int]:
        # P2, P3, P4, P5, P6, P7, P8, P9 (clockwise starting top)
        return [
            1 if skel[r - 1][c] else 0,
            1 if skel[r - 1][c + 1] else 0,
            1 if skel[r][c + 1] else 0,
            1 if skel[r + 1][c + 1] else 0,
            1 if skel[r + 1][c] else 0,
            1 if skel[r + 1][c - 1] else 0,
            1 if skel[r][c - 1] else 0,
            1 if skel[r - 1][c - 1] else 0,
        ]

    for _ in range(20):
        changed = False
        for step in (1, 2):
            to_clear: List[Tuple[int, int]] = []
            for r in range(1, h - 1):
                for c in range(1, w - 1):
                    if not skel[r][c]:
                        continue
                    nb = neighbors(r, c)
                    b_sum = sum(nb)
                    if b_sum < 2 or b_sum > 6:
                        continue
                    # 0->1 transitions in ordered P2..P9..P2
                    a_trans = sum(
                        1 for i in range(8) if nb[i] == 0 and nb[(i + 1) % 8] == 1
                    )
                    if a_trans != 1:
                        continue
                    p2, p3, p4, p5, p6, p7, p8, p9 = nb
                    if step == 1:
                        if p2 * p4 * p6 == 0 and p4 * p6 * p8 == 0:
                            to_clear.append((r, c))
                    else:
                        if p2 * p4 * p8 == 0 and p2 * p6 * p8 == 0:
                            to_clear.append((r, c))
            if to_clear:
                changed = True
                for r, c in to_clear:
                    skel[r][c] = False
        if not changed:
            break
    return skel


def reconstruct_3d_walls_from_pgm(
    pgm_bytes: bytes,
    resolution: float = 0.05,
    origin_xy: Tuple[float, float] = (-5.0, -5.0),
    wall_height: float = 2.50,
    occupied_threshold: int = 80,
    max_walls: int = 120,
) -> List[Dict[str, Any]]:
    """
    Intelligent 2D ROS SLAM `.pgm` to 3D Wall & Obstacle Vectorizer:
      1. Finds 8-connected components of occupied pixels.
      2. Identifies compact circular pillars/obstacles and emits smooth 3D `<cylinder>` nodes
         instead of jagged pixel squares.
      3. Identifies circular ring walls and emits smooth tangent-continuous curved wall arcs.
      4. Thins arbitrary wall networks into 1-pixel skeletons, traces continuous chains,
         and classifies straight walls (horizontal, vertical, or diagonal at any angle)
         vs. curved walls (smoothed with Chaikin + RDP) so zero "mini-square" artifacts appear!
    """
    width, height, _, grid = parse_pgm_bytes(pgm_bytes)
    ox, oy = float(origin_xy[0]), float(origin_xy[1])
    res = max(float(resolution), 0.01)

    def pixel_to_world(r: float, c: float) -> Tuple[float, float]:
        wx = ox + (c + 0.5) * res
        wy = oy + (height - 1 - r + 0.5) * res
        return (wx, wy)

    occ_mask = [
        [grid[r][c] <= occupied_threshold for c in range(width)]
        for r in range(height)
    ]

    # 1. Extract 8-connected components
    comp_visited = [[False] * width for _ in range(height)]
    components: List[List[Tuple[int, int]]] = []
    dirs8 = [(-1, -1), (-1, 0), (-1, 1), (0, -1), (0, 1), (1, -1), (1, 0), (1, 1)]

    for r in range(height):
        for c in range(width):
            if not occ_mask[r][c] or comp_visited[r][c]:
                continue
            stack = [(r, c)]
            comp_visited[r][c] = True
            comp: List[Tuple[int, int]] = []
            while stack:
                cr, cc = stack.pop()
                comp.append((cr, cc))
                for dr, dc in dirs8:
                    nr, nc = cr + dr, cc + dc
                    if 0 <= nr < height and 0 <= nc < width and occ_mask[nr][nc] and not comp_visited[nr][nc]:
                        comp_visited[nr][nc] = True
                        stack.append((nr, nc))
            if len(comp) >= 2:
                components.append(comp)

    nodes: List[Dict[str, Any]] = []
    wall_mask = [[False] * width for _ in range(height)]
    cyl_idx = 0
    wall_idx = 0

    for comp in components:
        w_pts = [pixel_to_world(r, c) for r, c in comp]
        n_pix = len(w_pts)
        cx = sum(p[0] for p in w_pts) / n_pix
        cy = sum(p[1] for p in w_pts) / n_pix

        xs = [p[0] for p in w_pts]
        ys = [p[1] for p in w_pts]
        span_x = max(xs) - min(xs) + res
        span_y = max(ys) - min(ys) + res
        aspect = max(span_x, span_y) / max(min(span_x, span_y), res)

        radii = [math.hypot(p[0] - cx, p[1] - cy) for p in w_pts]
        mean_r = sum(radii) / n_pix
        max_r = max(radii) if radii else res
        std_r = math.sqrt(sum((rv - mean_r) ** 2 for rv in radii) / n_pix) if n_pix > 1 else 0.0
        bbox_area_cells = max((span_x / res) * (span_y / res), 1.0)
        fill_ratio = n_pix / bbox_area_cells

        # Case A: Solid Circular Pillar / Round Obstacle -> Single Smooth Cylinder!
        if (
            n_pix >= 4
            and aspect < 1.38
            and fill_ratio >= 0.52
            and max_r <= 1.8
            and abs(span_x - span_y) <= 2.5 * res
        ):
            # Verify it is closer to a disc than a sharp square by checking corner occupancy or radial ratio
            expected_disc_r = math.sqrt((n_pix * res * res) / math.pi)
            if fill_ratio < 0.88 or n_pix <= 16:
                cyl_idx += 1
                radius = max(expected_disc_r, res * 0.8)
                nodes.append(
                    create_primitive_node(
                        name=f"pgm_columna_{cyl_idx}",
                        shape="cylinder",
                        pose=[clean_float(cx, 4), clean_float(cy, 4), clean_float(wall_height * 0.5, 4), 0.0, 0.0, 0.0],
                        radius=clean_float(radius, 4),
                        length=clean_float(wall_height, 4),
                        color=[0.70, 0.75, 0.82, 1.0],
                        static=True,
                        material_preset="concrete",
                        folder_id="folder_arch",
                    )
                )
                continue

        # Case B: Hollow Circular Ring Wall -> Smooth Tangent Curved Wall Segments!
        if (
            n_pix >= 16
            and aspect < 1.28
            and fill_ratio < 0.48
            and mean_r > 3.0 * res
            and (std_r / max(mean_r, 1e-6)) < 0.22
        ):
            n_arc_segs = max(min(int((2.0 * math.pi * mean_r) / 0.65), 24), 10)
            est_thick = max(min(2.2 * std_r + res, 0.35), 0.14)
            arc_pts = [
                (
                    cx + mean_r * math.cos(2.0 * math.pi * i / n_arc_segs),
                    cy + mean_r * math.sin(2.0 * math.pi * i / n_arc_segs),
                )
                for i in range(n_arc_segs)
            ]
            ring_walls = extrude_polyline_to_walls(
                points_xy=arc_pts,
                thickness=est_thick,
                height=wall_height,
                closed_loop=True,
                wall_type="wall",
                base_name="pgm_muro_curvo",
            )
            # Slightly lengthen each segment for seamless mitered curve joints
            for rw in ring_walls:
                rw["archetype_params"]["length"] = clean_float(
                    float(rw["archetype_params"]["length"]) + est_thick * 0.35, 4
                )
                rw["parts"][0]["size"][0] = rw["archetype_params"]["length"]
                rw["geometry"]["size"][0] = rw["archetype_params"]["length"]
            nodes.extend(ring_walls)
            continue

        # Otherwise mark component pixels into `wall_mask` for skeleton centerline vectorization
        for r, c in comp:
            wall_mask[r][c] = True

    # 2. Pad & Skeletonize `wall_mask` to extract 1-pixel centerlines
    pad_h = height + 2
    pad_w = width + 2
    padded = [[False] * pad_w for _ in range(pad_h)]
    for r in range(height):
        for c in range(width):
            padded[r + 1][c + 1] = wall_mask[r][c]

    skel_pad = _skeletonize_binary_mask(padded)
    skel_pts = set()
    for r in range(height):
        for c in range(width):
            if skel_pad[r + 1][c + 1]:
                skel_pts.add((r, c))

    if not skel_pts:
        return nodes[:max_walls]

    # Estimate wall thickness from ratio of total wall pixels to skeleton pixels
    total_wall_pix = sum(1 for r in range(height) for c in range(width) if wall_mask[r][c])
    avg_thick_px = max(1.0, min(6.0, total_wall_pix / float(max(len(skel_pts), 1))))
    wall_thickness_m = clean_float(max(avg_thick_px * res, 0.15), 3)

    def skel_neighbors(pt: Tuple[int, int]) -> List[Tuple[int, int]]:
        r, c = pt
        out = []
        for dr, dc in dirs8:
            nb = (r + dr, c + dc)
            if nb in skel_pts:
                out.append(nb)
        return out

    deg_map = {pt: len(skel_neighbors(pt)) for pt in skel_pts}
    visited_edges: set[Tuple[Tuple[int, int], Tuple[int, int]]] = set()
    chains: List[List[Tuple[int, int]]] = []

    def add_edge(a: Tuple[int, int], b: Tuple[int, int]) -> None:
        visited_edges.add((a, b))
        visited_edges.add((b, a))

    # Trace chains starting from endpoints (deg == 1) and junctions (deg >= 3)
    start_nodes = [pt for pt, d in deg_map.items() if d != 2]
    for s_pt in start_nodes:
        for nb in skel_neighbors(s_pt):
            if (s_pt, nb) in visited_edges:
                continue
            chain = [s_pt, nb]
            add_edge(s_pt, nb)
            curr = nb
            prev = s_pt
            while deg_map.get(curr, 0) == 2:
                nbs = [x for x in skel_neighbors(curr) if x != prev and (curr, x) not in visited_edges]
                if not nbs:
                    break
                nxt = nbs[0]
                add_edge(curr, nxt)
                chain.append(nxt)
                prev, curr = curr, nxt
            chains.append(chain)

    # Also trace any remaining closed loops (all deg == 2)
    for s_pt in skel_pts:
        for nb in skel_neighbors(s_pt):
            if (s_pt, nb) in visited_edges:
                continue
            chain = [s_pt, nb]
            add_edge(s_pt, nb)
            curr = nb
            prev = s_pt
            while True:
                nbs = [x for x in skel_neighbors(curr) if x != prev and (curr, x) not in visited_edges]
                if not nbs:
                    # Close loop back to s_pt if adjacent
                    if s_pt in skel_neighbors(curr) and (curr, s_pt) not in visited_edges and len(chain) >= 3:
                        add_edge(curr, s_pt)
                        chain.append(s_pt)
                    break
                nxt = nbs[0]
                add_edge(curr, nxt)
                chain.append(nxt)
                prev, curr = curr, nxt
            chains.append(chain)

    # 3. Classify each chain into straight walls vs smooth curved walls
    for chain in chains:
        if len(chain) < 2:
            continue
        world_chain = [pixel_to_world(r, c) for r, c in chain]
        is_closed = len(chain) >= 4 and chain[0] == chain[-1]

        # First pass RDP with tight tolerance (1.35 * res) to remove pixel staircase
        simplified = _rdp_simplify(world_chain, epsilon=res * 1.35)
        if len(simplified) < 2:
            continue

        # Check if the chain has curved sections (consecutive segments turning smoothly)
        is_curved = False
        if len(simplified) >= 4:
            turn_angles = []
            for i in range(len(simplified) - 2):
                a1 = math.atan2(simplified[i + 1][1] - simplified[i][1], simplified[i + 1][0] - simplified[i][0])
                a2 = math.atan2(simplified[i + 2][1] - simplified[i + 1][1], simplified[i + 2][0] - simplified[i + 1][0])
                diff = abs((a2 - a1 + math.pi) % (2.0 * math.pi) - math.pi)
                turn_angles.append(math.degrees(diff))
            # If multiple consecutive turns are between 6° and 55° (smooth arc rather than 90° room corner)
            smooth_turns = [ang for ang in turn_angles if 6.0 <= ang <= 55.0]
            if len(smooth_turns) >= 2:
                is_curved = True

        if is_curved:
            # Smooth curve with Chaikin corner cutting and simplify gently
            final_pts = _rdp_simplify(_chaikin_smooth_curve(simplified, closed=is_closed), epsilon=res * 0.75)
        else:
            # Straight walls (including diagonal walls at any angle!)
            final_pts = _rdp_simplify(simplified, epsilon=res * 1.85)

        for i in range(len(final_pts) - 1):
            x0, y0 = final_pts[i]
            x1, y1 = final_pts[i + 1]
            dx = x1 - x0
            dy = y1 - y0
            seg_len = math.hypot(dx, dy)
            if seg_len < max(res * 1.5, 0.08):
                continue

            wall_idx += 1
            mx = (x0 + x1) * 0.5
            my = (y0 + y1) * 0.5
            yaw = math.atan2(dy, dx)
            # Add slight corner overlap so angled and curved joints meet cleanly with zero gaps
            eff_len = seg_len + wall_thickness_m * (0.40 if is_curved else 0.25)

            w_node = create_archetype_node(
                archetype="wall",
                name=f"pgm_muro_{'curvo' if is_curved else 'recto'}_{wall_idx}",
                pose=[
                    clean_float(mx, 4),
                    clean_float(my, 4),
                    clean_float(wall_height * 0.5, 4),
                    0.0,
                    0.0,
                    clean_float(yaw, 6),
                ],
                custom_params={
                    "length": clean_float(eff_len, 4),
                    "thickness": wall_thickness_m,
                    "height": clean_float(wall_height, 4),
                },
            )
            nodes.append(w_node)
            if len(nodes) >= max_walls:
                return nodes

    return nodes


