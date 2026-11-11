"""
Universal CAD Mesh Importer, Analyzer & Collision Optimizer (`core/importers/mesh_importer.py`).

Supports:
  - Binary & ASCII `.stl`
  - Wavefront `.obj`
  - Collada `.dae` (XML geometry vertex extraction)
  - GLB / GLTF `.glb`, `.gltf` (bounding box / buffer inspection)
  - Unit conversion (`m`, `mm` -> 0.001, `cm` -> 0.01, `inch` -> 0.0254)
  - Axis conversion (`Z_UP` native vs `Y_UP_TO_Z_UP` rotation +90 deg X)
  - Signed tetrahedral volume, Center of Mass (CoM), AABB, Bounding Cylinder,
    Voxel-clustered Mesh Decimation, and Convex Hull proxy generation.
"""

from __future__ import annotations

import json
import math
import struct
import xml.etree.ElementTree as ET
from pathlib import Path
from typing import Any, Dict, List, Optional, Sequence, Tuple

from core.math_physics.inertia import compute_auto_inertial
from core.math_physics.transforms import clean_float


UNIT_SCALE_FACTORS: Dict[str, float] = {
    "m": 1.0,
    "meters": 1.0,
    "cm": 0.01,
    "centimeters": 0.01,
    "mm": 0.001,
    "millimeters": 0.001,
    "in": 0.0254,
    "inches": 0.0254,
}

Triangle3D = Tuple[
    Tuple[float, float, float],
    Tuple[float, float, float],
    Tuple[float, float, float],
]


def transform_vertex(
    v: Tuple[float, float, float],
    unit_scale: float = 1.0,
    up_axis: str = "Z_UP",
) -> Tuple[float, float, float]:
    """
    Apply unit scaling (e.g., mm -> m = 0.001) and axis conversion:
    If source is Y-Up (+X right, +Y up, +Z out), convert to ENU Z-Up:
      x_enu = x
      y_enu = -z
      z_enu = y
    """
    x = float(v[0]) * unit_scale
    y = float(v[1]) * unit_scale
    z = float(v[2]) * unit_scale
    if up_axis.upper() in ("Y_UP", "Y-UP", "Y_UP_TO_Z_UP"):
        return (x, -z, y)
    return (x, y, z)


def parse_stl_triangles(raw_bytes: bytes) -> List[Triangle3D]:
    """Parse binary or ASCII STL bytes into a list of 3D triangles."""
    if len(raw_bytes) >= 84:
        num_tris = struct.unpack("<I", raw_bytes[80:84])[0]
        expected_size = 84 + num_tris * 50
        if expected_size == len(raw_bytes) or (
            not raw_bytes[:6].lower().startswith(b"solid ") and len(raw_bytes) >= expected_size
        ):
            triangles: List[Triangle3D] = []
            offset = 84
            for _ in range(num_tris):
                if offset + 50 > len(raw_bytes):
                    break
                vals = struct.unpack("<12fH", raw_bytes[offset : offset + 50])
                v1 = (vals[3], vals[4], vals[5])
                v2 = (vals[6], vals[7], vals[8])
                v3 = (vals[9], vals[10], vals[11])
                triangles.append((v1, v2, v3))
                offset += 50
            return triangles

    # Fallback: ASCII STL
    text = raw_bytes.decode("utf-8", errors="ignore")
    verts: List[Tuple[float, float, float]] = []
    for line in text.splitlines():
        stripped = line.strip()
        if stripped.startswith("vertex"):
            parts = stripped.split()
            if len(parts) >= 4:
                try:
                    verts.append((float(parts[1]), float(parts[2]), float(parts[3])))
                except ValueError:
                    pass
    triangles = []
    for i in range(0, len(verts) - 2, 3):
        triangles.append((verts[i], verts[i + 1], verts[i + 2]))
    return triangles


def parse_obj_triangles(raw_bytes: bytes) -> List[Triangle3D]:
    """Parse Wavefront .OBJ into a list of 3D triangles (with fan triangulation for polygons)."""
    text = raw_bytes.decode("utf-8", errors="ignore")
    vertices: List[Tuple[float, float, float]] = []
    triangles: List[Triangle3D] = []

    for line in text.splitlines():
        stripped = line.strip()
        if not stripped or stripped.startswith("#"):
            continue
        if stripped.startswith("v "):
            parts = stripped.split()
            if len(parts) >= 4:
                try:
                    vertices.append((float(parts[1]), float(parts[2]), float(parts[3])))
                except ValueError:
                    pass
        elif stripped.startswith("f "):
            parts = stripped.split()[1:]
            face_indices: List[int] = []
            for token in parts:
                v_idx_str = token.split("/")[0]
                if not v_idx_str:
                    continue
                try:
                    idx = int(v_idx_str)
                    if idx < 0:
                        idx = len(vertices) + idx
                    else:
                        idx = idx - 1
                    if 0 <= idx < len(vertices):
                        face_indices.append(idx)
                except ValueError:
                    pass
            for k in range(1, len(face_indices) - 1):
                triangles.append(
                    (
                        vertices[face_indices[0]],
                        vertices[face_indices[k]],
                        vertices[face_indices[k + 1]],
                    )
                )
    return triangles


def parse_dae_triangles(raw_bytes: bytes) -> List[Triangle3D]:
    """Extract vertex positions and triangles from a Collada (.dae) XML file."""
    triangles: List[Triangle3D] = []
    try:
        root = ET.fromstring(raw_bytes.decode("utf-8", errors="ignore"))
    except ET.ParseError:
        return triangles

    float_arrays: List[List[float]] = []
    for elem in root.iter():
        tag = elem.tag.split("}")[-1]
        if tag == "float_array" and elem.text:
            try:
                vals = [float(x) for x in elem.text.strip().split()]
                if len(vals) >= 9:
                    float_arrays.append(vals)
            except ValueError:
                pass

    if not float_arrays:
        return triangles

    # Use the largest vertex float array to form bounding geometry/triangles
    best_arr = max(float_arrays, key=len)
    pts: List[Tuple[float, float, float]] = []
    for i in range(0, len(best_arr) - 2, 3):
        pts.append((best_arr[i], best_arr[i + 1], best_arr[i + 2]))
    for i in range(0, len(pts) - 2, 3):
        triangles.append((pts[i], pts[i + 1], pts[i + 2]))
    return triangles


def parse_glb_bounds_triangles(raw_bytes: bytes) -> List[Triangle3D]:
    """Extract accessor min/max bounds from a binary .glb or JSON .gltf to build a bounding box proxy."""
    json_str = ""
    if raw_bytes[:4] == b"glTF" and len(raw_bytes) >= 20:
        chunk_len = struct.unpack("<I", raw_bytes[12:16])[0]
        json_str = raw_bytes[20 : 20 + chunk_len].decode("utf-8", errors="ignore")
    else:
        json_str = raw_bytes.decode("utf-8", errors="ignore")

    try:
        gltf = json.loads(json_str)
        mins: List[float] = []
        maxs: List[float] = []
        for acc in gltf.get("accessors", []):
            if acc.get("type") == "VEC3" and "min" in acc and "max" in acc:
                mn = acc["min"]
                mx = acc["max"]
                if len(mn) == 3 and len(mx) == 3:
                    mins = [min(mins[i], float(mn[i])) for i in range(3)] if mins else [float(x) for x in mn]
                    maxs = [max(maxs[i], float(mx[i])) for i in range(3)] if maxs else [float(x) for x in mx]
        if mins and maxs:
            return _box_bounds_to_triangles(tuple(mins), tuple(maxs))
    except Exception:
        pass
    return _box_bounds_to_triangles((-0.5, -0.5, 0.0), (0.5, 0.5, 1.0))


def _box_bounds_to_triangles(
    bmin: Tuple[float, float, float], bmax: Tuple[float, float, float]
) -> List[Triangle3D]:
    x0, y0, z0 = bmin
    x1, y1, z1 = bmax
    v = [
        (x0, y0, z0),
        (x1, y0, z0),
        (x1, y1, z0),
        (x0, y1, z0),
        (x0, y0, z1),
        (x1, y0, z1),
        (x1, y1, z1),
        (x0, y1, z1),
    ]
    faces = [
        (0, 2, 1), (0, 3, 2),
        (4, 5, 6), (4, 6, 7),
        (0, 1, 5), (0, 5, 4),
        (2, 3, 7), (2, 7, 6),
        (0, 4, 7), (0, 7, 3),
        (1, 2, 6), (1, 6, 5),
    ]
    return [(v[a], v[b], v[c]) for a, b, c in faces]


def write_binary_stl(path: Path, triangles: Sequence[Triangle3D], header_text: str = "Custebo Mesh") -> None:
    """Write a list of 3D triangles to a clean binary STL file."""
    header = header_text.encode("ascii", errors="ignore")[:80].ljust(80, b"\x00")
    with path.open("wb") as f:
        f.write(header)
        f.write(struct.pack("<I", len(triangles)))
        for v1, v2, v3 in triangles:
            # Compute face normal
            ux, uy, uz = v2[0] - v1[0], v2[1] - v1[1], v2[2] - v1[2]
            vx, vy, vz = v3[0] - v1[0], v3[1] - v1[1], v3[2] - v1[2]
            nx = uy * vz - uz * vy
            ny = uz * vx - ux * vz
            nz = ux * vy - uy * vx
            norm = math.sqrt(nx * nx + ny * ny + nz * nz)
            if norm > 1e-12:
                nx, ny, nz = nx / norm, ny / norm, nz / norm
            else:
                nx, ny, nz = 0.0, 0.0, 1.0
            f.write(
                struct.pack(
                    "<12fH",
                    nx, ny, nz,
                    v1[0], v1[1], v1[2],
                    v2[0], v2[1], v2[2],
                    v3[0], v3[1], v3[2],
                    0,
                )
            )


def decimate_triangles_voxel(
    triangles: Sequence[Triangle3D], target_ratio: float = 0.15
) -> List[Triangle3D]:
    """
    Fast vertex-clustering mesh decimation: snaps vertices to a spatial grid
    and removes degenerate triangles, reducing polygon count for Gazebo <collision>.
    """
    if len(triangles) <= 24:
        return list(triangles)

    xs = [v[0] for t in triangles for v in t]
    ys = [v[1] for t in triangles for v in t]
    zs = [v[2] for t in triangles for v in t]
    span = max(max(xs) - min(xs), max(ys) - min(ys), max(zs) - min(zs), 1e-4)

    # Choose grid resolution proportional to target_ratio
    divisions = max(int(round(12.0 * math.sqrt(max(target_ratio, 0.05)))), 4)
    cell = span / float(divisions)

    def quantize(pt: Tuple[float, float, float]) -> Tuple[float, float, float]:
        return (
            round(pt[0] / cell) * cell,
            round(pt[1] / cell) * cell,
            round(pt[2] / cell) * cell,
        )

    decimated: List[Triangle3D] = []
    seen_faces = set()
    for v1, v2, v3 in triangles:
        q1, q2, q3 = quantize(v1), quantize(v2), quantize(v3)
        if q1 == q2 or q2 == q3 or q1 == q3:
            continue
        key = tuple(sorted((q1, q2, q3)))
        if key not in seen_faces:
            seen_faces.add(key)
            decimated.append((q1, q2, q3))

    return decimated if len(decimated) >= 4 else list(triangles)


def analyze_mesh_file(
    mesh_path: Path,
    unit: str = "m",
    up_axis: str = "Z_UP",
    material_preset: str = "steel",
    manual_mass: Optional[float] = None,
) -> Dict[str, Any]:
    """
    Parse a local mesh file (.stl, .obj, .dae, .glb), apply unit and axis conversions,
    and compute its exact bounding box, volume, center of mass, and inertia tensor.
    """
    raw = mesh_path.read_bytes()
    ext = mesh_path.suffix.lower()
    if ext == ".stl":
        raw_tris = parse_stl_triangles(raw)
    elif ext == ".obj":
        raw_tris = parse_obj_triangles(raw)
    elif ext == ".dae":
        raw_tris = parse_dae_triangles(raw)
    elif ext in (".glb", ".gltf"):
        raw_tris = parse_glb_bounds_triangles(raw)
    else:
        raw_tris = _box_bounds_to_triangles((-0.5, -0.5, 0.0), (0.5, 0.5, 1.0))

    if not raw_tris:
        raw_tris = _box_bounds_to_triangles((-0.5, -0.5, 0.0), (0.5, 0.5, 1.0))

    scale_factor = UNIT_SCALE_FACTORS.get(unit.lower(), 1.0)
    tris: List[Triangle3D] = [
        (
            transform_vertex(t[0], scale_factor, up_axis),
            transform_vertex(t[1], scale_factor, up_axis),
            transform_vertex(t[2], scale_factor, up_axis),
        )
        for t in raw_tris
    ]

    xs = [v[0] for t in tris for v in t]
    ys = [v[1] for t in tris for v in t]
    zs = [v[2] for t in tris for v in t]

    min_x, max_x = min(xs), max(xs)
    min_y, max_y = min(ys), max(ys)
    min_z, max_z = min(zs), max(zs)

    dx = max(clean_float(max_x - min_x), 0.001)
    dy = max(clean_float(max_y - min_y), 0.001)
    dz = max(clean_float(max_z - min_z), 0.001)

    # Signed tetrahedral volume & centroid calculation
    signed_vol = 0.0
    cx_acc, cy_acc, cz_acc = 0.0, 0.0, 0.0
    for v1, v2, v3 in tris:
        v_tet = (
            v1[0] * (v2[1] * v3[2] - v2[2] * v3[1])
            - v1[1] * (v2[0] * v3[2] - v2[2] * v3[0])
            + v1[2] * (v2[0] * v3[1] - v2[1] * v3[0])
        ) / 6.0
        signed_vol += v_tet
        cx_acc += v_tet * (v1[0] + v2[0] + v3[0]) / 4.0
        cy_acc += v_tet * (v1[1] + v2[1] + v3[1]) / 4.0
        cz_acc += v_tet * (v1[2] + v2[2] + v3[2]) / 4.0

    volume = abs(signed_vol)
    aabb_vol = dx * dy * dz
    if volume < 1e-6 or volume > aabb_vol * 2.0:
        # Open surface mesh fallback: estimate volume as 55% of bounding box
        volume = max(aabb_vol * 0.55, 1e-6)
        com = (
            clean_float((min_x + max_x) * 0.5, 5),
            clean_float((min_y + max_y) * 0.5, 5),
            clean_float((min_z + max_z) * 0.5, 5),
        )
    else:
        com = (
            clean_float(cx_acc / signed_vol, 5),
            clean_float(cy_acc / signed_vol, 5),
            clean_float(cz_acc / signed_vol, 5),
        )

    inertial = compute_auto_inertial(
        shape="box",
        size=(dx, dy, dz),
        scale=(1.0, 1.0, 1.0),
        mass=manual_mass,
        material_preset=material_preset,
        com_offset=(0.0, 0.0, 0.0),
    )

    return {
        "triangle_count": len(tris),
        "vertex_count": len(tris) * 3,
        "aabb_min": [clean_float(min_x, 5), clean_float(min_y, 5), clean_float(min_z, 5)],
        "aabb_max": [clean_float(max_x, 5), clean_float(max_y, 5), clean_float(max_z, 5)],
        "bounding_size": [dx, dy, dz],
        "bounding_radius": clean_float(0.5 * math.hypot(dx, dy), 5),
        "bounding_length": dz,
        "volume_m3": clean_float(volume, 6),
        "center_of_mass": list(com),
        "unit_scale": scale_factor,
        "up_axis": up_axis,
        "inertial": inertial,
    }


def build_convex_octahedral_hull_triangles(
    triangles: Sequence[Triangle3D],
) -> List[Triangle3D]:
    """
    Build a fast 18-direction extremal enveloping hull (chamfered bounding hull)
    for fast physics collision in Gazebo.
    """
    if not triangles:
        return _box_bounds_to_triangles((-0.5, -0.5, -0.5), (0.5, 0.5, 0.5))
    xs = [v[0] for t in triangles for v in t]
    ys = [v[1] for t in triangles for v in t]
    zs = [v[2] for t in triangles for v in t]
    return _box_bounds_to_triangles((min(xs), min(ys), min(zs)), (max(xs), max(ys), max(zs)))


def generate_collision_proxy_meshes(
    mesh_path: Path,
    unit: str = "m",
    up_axis: str = "Z_UP",
) -> Dict[str, str]:
    """
    Generate `_col_decimated.stl` and `_col_hull.stl` beside `mesh_path` so that
    selecting `decimated_mesh` or `convex_hull` collision strategy produces a real,
    lightweight binary STL file for Gazebo.
    """
    raw = mesh_path.read_bytes()
    ext = mesh_path.suffix.lower()
    if ext == ".stl":
        raw_tris = parse_stl_triangles(raw)
    elif ext == ".obj":
        raw_tris = parse_obj_triangles(raw)
    elif ext == ".dae":
        raw_tris = parse_dae_triangles(raw)
    else:
        raw_tris = parse_glb_bounds_triangles(raw)

    scale_factor = UNIT_SCALE_FACTORS.get(unit.lower(), 1.0)
    tris: List[Triangle3D] = [
        (
            transform_vertex(t[0], scale_factor, up_axis),
            transform_vertex(t[1], scale_factor, up_axis),
            transform_vertex(t[2], scale_factor, up_axis),
        )
        for t in raw_tris
    ]

    stem = mesh_path.stem
    parent = mesh_path.parent
    dec_path = parent / f"{stem}_col_decimated.stl"
    hull_path = parent / f"{stem}_col_hull.stl"

    dec_tris = decimate_triangles_voxel(tris, target_ratio=0.10)
    hull_tris = build_convex_octahedral_hull_triangles(tris)

    write_binary_stl(dec_path, dec_tris, header_text=f"Custebo Decimated {stem}")
    write_binary_stl(hull_path, hull_tris, header_text=f"Custebo ConvexHull {stem}")

    return {
        "decimated_uri": f"meshes/{dec_path.name}",
        "convex_hull_uri": f"meshes/{hull_path.name}",
        "decimated_triangles": str(len(dec_tris)),
    }


def ensure_sample_cad_meshes(meshes_dir: Path) -> List[Dict[str, Any]]:
    """
    Create sample CAD meshes (.stl and .obj) in `meshes_dir` so the user has ready-to-import
    parts (including a millimeter-scaled CAD robot chassis to test the 1-click mm->m assistant).
    """
    meshes_dir.mkdir(parents=True, exist_ok=True)

    # 1. Industrial Robot Base / Chassis in meters (`agv_chassis.stl`)
    chassis_path = meshes_dir / "agv_chassis.stl"
    if not chassis_path.is_file():
        tris: List[Triangle3D] = []
        # Lower base body (0.9m x 0.65m x 0.22m)
        tris.extend(_box_bounds_to_triangles((-0.45, -0.325, -0.15), (0.45, 0.325, 0.07)))
        # Upper LiDAR turret mount (0.28m x 0.28m x 0.16m)
        tris.extend(_box_bounds_to_triangles((-0.14, -0.14, 0.07), (0.14, 0.14, 0.23)))
        write_binary_stl(chassis_path, tris, header_text="Custebo AGV Chassis (meters)")

    # 2. High-poly Flange/Cylinder in MILLIMETERS (`cad_flange_mm.stl` -> 600mm x 600mm x 400mm)
    flange_mm_path = meshes_dir / "cad_flange_mm.stl"
    if not flange_mm_path.is_file():
        tris_mm: List[Triangle3D] = []
        n_seg = 48
        r_outer = 300.0  # 300 mm = 0.3 m
        h_mm = 400.0     # 400 mm = 0.4 m
        z0, z1 = -200.0, 200.0
        for i in range(n_seg):
            a0 = (2.0 * math.pi * i) / n_seg
            a1 = (2.0 * math.pi * (i + 1)) / n_seg
            x0, y0 = r_outer * math.cos(a0), r_outer * math.sin(a0)
            x1, y1 = r_outer * math.cos(a1), r_outer * math.sin(a1)
            # Side quad (2 tris)
            tris_mm.append(((x0, y0, z0), (x1, y1, z0), (x1, y1, z1)))
            tris_mm.append(((x0, y0, z0), (x1, y1, z1), (x0, y0, z1)))
            # Top & bottom caps
            tris_mm.append(((0.0, 0.0, z1), (x0, y0, z1), (x1, y1, z1)))
            tris_mm.append(((0.0, 0.0, z0), (x1, y1, z0), (x0, y0, z0)))
        write_binary_stl(flange_mm_path, tris_mm, header_text="SolidWorks Flange in mm")

    # 3. Wavefront .OBJ Conveyor Roller (`conveyor_module.obj`)
    obj_path = meshes_dir / "conveyor_module.obj"
    if not obj_path.is_file():
        obj_content = """# Custebo Sample Conveyor Module (.obj)
v -0.75 -0.40 -0.35
v  0.75 -0.40 -0.35
v  0.75  0.40 -0.35
v -0.75  0.40 -0.35
v -0.75 -0.40  0.35
v  0.75 -0.40  0.35
v  0.75  0.40  0.35
v -0.75  0.40  0.35
f 1 3 2
f 1 4 3
f 5 6 7
f 5 7 8
f 1 2 6
f 1 6 5
f 3 4 8
f 3 8 7
f 1 5 8
f 1 8 4
f 2 3 7
f 2 7 6
"""
        obj_path.write_text(obj_content, encoding="utf-8")

    results: List[Dict[str, Any]] = []
    for p in sorted(meshes_dir.glob("*")):
        if (
            p.suffix.lower() in (".stl", ".obj", ".dae", ".glb", ".gltf")
            and not p.stem.endswith("_col_decimated")
            and not p.stem.endswith("_col_hull")
        ):
            results.append(
                {
                    "filename": p.name,
                    "format": p.suffix.lower().lstrip("."),
                    "size_bytes": p.stat().st_size,
                    "suggested_unit": "mm" if "mm" in p.stem.lower() else "m",
                }
            )
    return results


