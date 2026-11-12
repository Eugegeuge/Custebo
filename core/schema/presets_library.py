"""
Catalog of Smart Building Blocks & Parametric Multi-Part Composites (`core/schema/presets_library.py`).

Every preset here is built as either:
  1. A pre-configured parametric single primitive (e.g., `wall`, `column_square`, `column_round`, `curb`, `cardboard_box`)
  2. A multi-part parametric composite (`wall_doorway`, `wall_window`, `wall_corner_l`, `stairs`, `ramp`,
     `table`, `warehouse_rack`, `euro_pallet`, `safety_fence`, `traffic_cone`, `bollard`)
     whose sub-parts (`node["parts"]`) are deterministically generated from `node["archetype_params"]`.

All coordinates in ENU Z-Up, meters, and radians.
When a composite node's origin `pose` has Z = 0 (or bottom at Z = 0), its parts are positioned so
the base of the structure rests at local z = -height/2 (matching centered bounding-box convention
so `DropToGroundCommand` works identically on both simple primitives and complex composites!).
"""

from __future__ import annotations

import copy
import math
from typing import Any, Dict, List, Optional

from core.math_physics.transforms import clean_float
from core.schema.scene_graph import generate_node_id, normalize_node_in_place


ARCHETYPE_CATALOG: Dict[str, Dict[str, Any]] = {
    # --- ARCHITECTURE & WALLS ---
    "wall": {
        "label": "🧱 Pared / Muro",
        "category": "architecture",
        "folder_id": "folder_arch",
        "static": True,
        "material_preset": "concrete",
        "default_color": [0.78, 0.80, 0.83, 1.0],
        "default_params": {
            "length": 4.0,
            "thickness": 0.20,
            "height": 2.50,
        },
    },
    "wall_doorway": {
        "label": "🚪 Muro con Puerta",
        "category": "architecture",
        "folder_id": "folder_arch",
        "static": True,
        "material_preset": "concrete",
        "default_color": [0.78, 0.80, 0.83, 1.0],
        "default_params": {
            "length": 4.0,
            "thickness": 0.20,
            "height": 2.50,
            "door_width": 1.10,
            "door_height": 2.10,
        },
    },
    "wall_window": {
        "label": "🪟 Muro con Ventana",
        "category": "architecture",
        "folder_id": "folder_arch",
        "static": True,
        "material_preset": "concrete",
        "default_color": [0.78, 0.80, 0.83, 1.0],
        "default_params": {
            "length": 4.0,
            "thickness": 0.20,
            "height": 2.50,
            "window_width": 1.50,
            "window_height": 1.10,
            "sill_height": 0.90,
        },
    },
    "wall_corner_l": {
        "label": "📐 Esquina en L",
        "category": "architecture",
        "folder_id": "folder_arch",
        "static": True,
        "material_preset": "concrete",
        "default_color": [0.78, 0.80, 0.83, 1.0],
        "default_params": {
            "length_x": 3.0,
            "length_y": 3.0,
            "thickness": 0.20,
            "height": 2.50,
        },
    },
    "column_square": {
        "label": "🏛 Columna Cuadrada",
        "category": "architecture",
        "folder_id": "folder_arch",
        "static": True,
        "material_preset": "concrete",
        "default_color": [0.64, 0.68, 0.73, 1.0],
        "default_params": {
            "width": 0.45,
            "height": 2.50,
        },
    },
    " stairs": {
        "label": "🪜 Escalera",
        "category": "architecture",
        "folder_id": "folder_arch",
        "static": True,
        "material_preset": "concrete",
        "default_color": [0.60, 0.64, 0.68, 1.0],
        "default_params": {
            "steps": 6,
            "width": 1.40,
            "step_depth": 0.30,
            "step_height": 0.16,
        },
    },
    "ramp": {
        "label": "📐 Rampa Robot",
        "category": "architecture",
        "folder_id": "folder_arch",
        "static": True,
        "material_preset": "concrete",
        "default_color": [0.52, 0.56, 0.62, 1.0],
        "default_params": {
            "length": 3.0,
            "width": 1.50,
            "height": 0.50,
            "segments": 10,
        },
    },
    # --- WAREHOUSE, FURNITURE & PROPS ---
    "warehouse_rack": {
        "label": "🗄 Estantería Almacén",
        "category": "warehouse",
        "folder_id": "folder_props",
        "static": True,
        "material_preset": "steel",
        "default_color": [0.15, 0.39, 0.92, 1.0],
        "default_params": {
            "width": 2.0,
            "depth": 0.80,
            "height": 2.20,
            "shelves": 4,
            "post_width": 0.06,
        },
    },
    "table": {
        "label": "🪑 Mesa / Escritorio",
        "category": "warehouse",
        "folder_id": "folder_props",
        "static": True,
        "material_preset": "wood",
        "default_color": [0.76, 0.54, 0.32, 1.0],
        "default_params": {
            "width": 1.40,
            "depth": 0.80,
            "height": 0.75,
            "top_thickness": 0.05,
            "leg_width": 0.06,
        },
    },
    "euro_pallet": {
        "label": "🪵 Palé Europeo",
        "category": "warehouse",
        "folder_id": "folder_props",
        "static": False,
        "material_preset": "wood",
        "default_color": [0.72, 0.52, 0.28, 1.0],
        "default_params": {
            "length": 1.20,
            "width": 0.80,
            "height": 0.144,
        },
    },
    "safety_fence": {
        "label": "🚧 Valla Industrial",
        "category": "warehouse",
        "folder_id": "folder_props",
        "static": True,
        "material_preset": "steel",
        "default_color": [0.95, 0.75, 0.05, 1.0],
        "default_params": {
            "length": 2.0,
            "height": 1.10,
            "rail_radius": 0.03,
        },
    },
    "jersey_barrier": {
        "label": "🚧 Barrera Vial",
        "category": "warehouse",
        "folder_id": "folder_props",
        "static": True,
        "material_preset": "concrete",
        "default_color": [0.75, 0.77, 0.79, 1.0],
        "default_params": {
            "length": 2.0,
            "bottom_width": 0.60,
            "top_width": 0.24,
            "height": 0.85,
        },
    },
    "traffic_cone": {
        "label": "🔺 Cono Tráfico",
        "category": "warehouse",
        "folder_id": "folder_props",
        "static": False,
        "material_preset": "plastic",
        "default_color": [0.98, 0.38, 0.08, 1.0],
        "default_params": {
            "base_width": 0.36,
            "height": 0.55,
        },
    },
    "bollard": {
        "label": "🛑 Bolardo / Poste",
        "category": "warehouse",
        "folder_id": "folder_props",
        "static": True,
        "material_preset": "steel",
        "default_color": [0.95, 0.72, 0.06, 1.0],
        "default_params": {
            "radius": 0.09,
            "height": 0.95,
        },
    },
    "cardboard_box": {
        "label": "📦 Caja Cartón (Dinámica)",
        "category": "warehouse",
        "folder_id": "folder_props",
        "static": False,
        "material_preset": "wood",
        "default_color": [0.80, 0.62, 0.38, 1.0],
        "default_params": {
            "dx": 0.45,
            "dy": 0.35,
            "dz": 0.32,
        },
    },
    # --- ROADS, STREETS & TRAFFIC INFRASTRUCTURE ---
    "road_straight": {
        "label": "🛣 Tramo Carretera",
        "category": "roads",
        "folder_id": "folder_arch",
        "static": True,
        "material_preset": "concrete",
        "default_color": [0.18, 0.20, 0.23, 1.0],
        "default_params": {
            "length": 8.0,
            "width": 4.5,
            "thickness": 0.06,
            "curb_width": 0.25,
        },
    },
    "road_curved": {
        "label": "↪ Curva Carretera",
        "category": "roads",
        "folder_id": "folder_arch",
        "static": True,
        "material_preset": "concrete",
        "default_color": [0.18, 0.20, 0.23, 1.0],
        "default_params": {
            "radius": 6.0,
            "width": 4.5,
            "angle_deg": 90.0,
            "thickness": 0.06,
            "segments": 10,
        },
    },
    "road_intersection": {
        "label": "➕ Cruce 4 Vías",
        "category": "roads",
        "folder_id": "folder_arch",
        "static": True,
        "material_preset": "concrete",
        "default_color": [0.18, 0.20, 0.23, 1.0],
        "default_params": {
            "arm_length": 10.0,
            "width": 4.5,
            "thickness": 0.06,
        },
    },
    "road_crosswalk": {
        "label": "🦓 Paso de Cebra",
        "category": "roads",
        "folder_id": "folder_arch",
        "static": True,
        "material_preset": "concrete",
        "default_color": [0.18, 0.20, 0.23, 1.0],
        "default_params": {
            "length": 4.0,
            "width": 4.5,
            "thickness": 0.06,
            "stripes": 6,
        },
    },
    "roundabout": {
        "label": "🔄 Rotonda / Glorieta",
        "category": "roads",
        "folder_id": "folder_arch",
        "static": True,
        "material_preset": "concrete",
        "default_color": [0.18, 0.20, 0.23, 1.0],
        "default_params": {
            "outer_radius": 6.5,
            "inner_radius": 2.2,
            "thickness": 0.06,
        },
    },
    "speed_bump": {
        "label": "〰 Badén Reductor",
        "category": "roads",
        "folder_id": "folder_props",
        "static": True,
        "material_preset": "plastic",
        "default_color": [0.95, 0.76, 0.08, 1.0],
        "default_params": {
            "width": 4.0,
            "depth": 0.48,
            "height": 0.07,
            "stripes": 8,
        },
    },
}

# Fix accidental leading space in key if any
if " stairs" in ARCHETYPE_CATALOG:
    ARCHETYPE_CATALOG["stairs"] = ARCHETYPE_CATALOG.pop(" stairs")


def _make_part(
    name: str,
    shape: str,
    local_pose: List[float],
    size: Optional[List[float]] = None,
    radius: float = 0.1,
    length: float = 1.0,
    color: Optional[List[float]] = None,
) -> Dict[str, Any]:
    return {
        "name": name,
        "shape": shape,
        "local_pose": [clean_float(float(v), 6) for v in local_pose[:6]],
        "size": [clean_float(float(v), 6) for v in (size or [1.0, 1.0, 1.0])[:3]],
        "radius": clean_float(float(radius), 6),
        "length": clean_float(float(length), 6),
        "color": [clean_float(float(c), 4) for c in (color or [0.75, 0.78, 0.82, 1.0])[:4]],
    }


def rebuild_archetype_node(node: Dict[str, Any]) -> Dict[str, Any]:
    """
    Regenerate `node["parts"]` and bounding `node["geometry"]` based on `node["archetype"]`
    and `node["archetype_params"]`.
    All parts are centered relative to the overall bounding box center (local z in [-H/2, +H/2]).
    """
    arch = node.get("archetype")
    if not arch or arch not in ARCHETYPE_CATALOG:
        return node

    spec = ARCHETYPE_CATALOG[arch]
    params = copy.deepcopy(spec["default_params"])
    if isinstance(node.get("archetype_params"), dict):
        params.update(node["archetype_params"])
    node["archetype_params"] = params

    base_color = node.get("material", {}).get("color", spec["default_color"])
    parts: List[Dict[str, Any]] = []
    bbox = [1.0, 1.0, 1.0]

    if arch == "wall":
        L = max(float(params.get("length", 4.0)), 0.1)
        T = max(float(params.get("thickness", 0.20)), 0.02)
        H = max(float(params.get("height", 2.50)), 0.1)
        bbox = [L, T, H]
        parts = [_make_part("wall_body", "box", [0, 0, 0, 0, 0, 0], size=[L, T, H], color=base_color)]

    elif arch == "wall_doorway":
        L = max(float(params.get("length", 4.0)), 0.8)
        T = max(float(params.get("thickness", 0.20)), 0.02)
        H = max(float(params.get("height", 2.50)), 1.0)
        dw = min(max(float(params.get("door_width", 1.10)), 0.4), L - 0.2)
        dh = min(max(float(params.get("door_height", 2.10)), 0.5), H - 0.1)
        side_w = (L - dw) * 0.5
        lintel_h = H - dh
        bbox = [L, T, H]

        parts = [
            _make_part(
                "wall_left",
                "box",
                [-(dw * 0.5 + side_w * 0.5), 0.0, 0.0, 0.0, 0.0, 0.0],
                size=[side_w, T, H],
                color=base_color,
            ),
            _make_part(
                "wall_right",
                "box",
                [+(dw * 0.5 + side_w * 0.5), 0.0, 0.0, 0.0, 0.0, 0.0],
                size=[side_w, T, H],
                color=base_color,
            ),
            _make_part(
                "door_lintel",
                "box",
                [0.0, 0.0, (H * 0.5) - (lintel_h * 0.5), 0.0, 0.0, 0.0],
                size=[dw, T, lintel_h],
                color=base_color,
            ),
        ]

    elif arch == "wall_window":
        L = max(float(params.get("length", 4.0)), 1.0)
        T = max(float(params.get("thickness", 0.20)), 0.02)
        H = max(float(params.get("height", 2.50)), 1.2)
        ww = min(max(float(params.get("window_width", 1.50)), 0.3), L - 0.3)
        wh = min(max(float(params.get("window_height", 1.10)), 0.3), H - 0.4)
        sill_h = min(max(float(params.get("sill_height", 0.90)), 0.2), H - wh - 0.1)
        top_h = max(H - sill_h - wh, 0.1)
        side_w = (L - ww) * 0.5
        bbox = [L, T, H]

        z_bottom = -H * 0.5
        parts = [
            _make_part(
                "wall_left",
                "box",
                [-(ww * 0.5 + side_w * 0.5), 0.0, 0.0, 0.0, 0.0, 0.0],
                size=[side_w, T, H],
                color=base_color,
            ),
            _make_part(
                "wall_right",
                "box",
                [+(ww * 0.5 + side_w * 0.5), 0.0, 0.0, 0.0, 0.0, 0.0],
                size=[side_w, T, H],
                color=base_color,
            ),
            _make_part(
                "window_sill",
                "box",
                [0.0, 0.0, z_bottom + sill_h * 0.5, 0.0, 0.0, 0.0],
                size=[ww, T, sill_h],
                color=base_color,
            ),
            _make_part(
                "window_lintel",
                "box",
                [0.0, 0.0, (H * 0.5) - top_h * 0.5, 0.0, 0.0, 0.0],
                size=[ww, T, top_h],
                color=base_color,
            ),
        ]

    elif arch == "wall_corner_l":
        lx = max(float(params.get("length_x", 3.0)), 0.5)
        ly = max(float(params.get("length_y", 3.0)), 0.5)
        T = max(float(params.get("thickness", 0.20)), 0.05)
        H = max(float(params.get("height", 2.50)), 0.3)
        bbox = [lx, ly, H]
        # Corner meets at (-lx/2, -ly/2)
        parts = [
            _make_part(
                "wall_arm_x",
                "box",
                [0.0, -ly * 0.5 + T * 0.5, 0.0, 0.0, 0.0, 0.0],
                size=[lx, T, H],
                color=base_color,
            ),
            _make_part(
                "wall_arm_y",
                "box",
                [-lx * 0.5 + T * 0.5, T * 0.5, 0.0, 0.0, 0.0, 0.0],
                size=[T, max(ly - T, 0.1), H],
                color=base_color,
            ),
        ]

    elif arch == "column_square":
        w = max(float(params.get("width", 0.45)), 0.1)
        H = max(float(params.get("height", 2.50)), 0.2)
        bbox = [w, w, H]
        parts = [_make_part("column_body", "box", [0, 0, 0, 0, 0, 0], size=[w, w, H], color=base_color)]

    elif arch == "stairs":
        n_steps = max(min(int(params.get("steps", 6)), 30), 2)
        w = max(float(params.get("width", 1.40)), 0.3)
        sd = max(float(params.get("step_depth", 0.30)), 0.1)
        sh = max(float(params.get("step_height", 0.16)), 0.05)
        total_len = n_steps * sd
        total_h = n_steps * sh
        bbox = [total_len, w, total_h]

        x_start = -total_len * 0.5
        z_bottom = -total_h * 0.5
        for i in range(n_steps):
            step_h = (i + 1) * sh
            px = x_start + (i + 0.5) * sd
            pz = z_bottom + step_h * 0.5
            parts.append(
                _make_part(
                    f"step_{i + 1}",
                    "box",
                    [px, 0.0, pz, 0.0, 0.0, 0.0],
                    size=[sd, w, step_h],
                    color=base_color,
                )
            )

    elif arch == "ramp":
        L = max(float(params.get("length", 3.0)), 0.5)
        W = max(float(params.get("width", 1.50)), 0.3)
        H = max(float(params.get("height", 0.50)), 0.1)
        pitch_angle = -math.atan2(H, L)
        ramp_len = math.hypot(L, H)
        slab_t = 0.08
        bbox = [L, W, H]
        parts = [
            _make_part(
                "ramp_slope",
                "box",
                [0.0, 0.0, 0.0, 0.0, pitch_angle, 0.0],
                size=[ramp_len, W, slab_t],
                color=base_color,
            ),
            _make_part(
                "ramp_support_end",
                "box",
                [L * 0.5 - 0.1, 0.0, 0.0, 0.0, 0.0, 0.0],
                size=[0.20, W, H],
                color=base_color,
            ),
        ]

    elif arch == "warehouse_rack":
        W = max(float(params.get("width", 2.0)), 0.5)
        D = max(float(params.get("depth", 0.80)), 0.3)
        H = max(float(params.get("height", 2.20)), 0.6)
        n_shelves = max(min(int(params.get("shelves", 4)), 10), 1)
        pw = max(float(params.get("post_width", 0.06)), 0.03)
        bbox = [W, D, H]

        post_color = base_color
        beam_color = [0.95, 0.45, 0.08, 1.0]
        deck_color = [0.76, 0.60, 0.38, 1.0]

        ox = W * 0.5 - pw * 0.5
        oy = D * 0.5 - pw * 0.5
        for idx, (sx, sy) in enumerate([(-ox, -oy), (ox, -oy), (-ox, oy), (ox, oy)]):
            parts.append(
                _make_part(
                    f"upright_post_{idx + 1}",
                    "box",
                    [sx, sy, 0.0, 0.0, 0.0, 0.0],
                    size=[pw, pw, H],
                    color=post_color,
                )
            )

        z_bottom = -H * 0.5
        spacing = (H - 0.20) / float(max(n_shelves, 1))
        for s_idx in range(n_shelves):
            sz = z_bottom + 0.18 + s_idx * spacing
            parts.append(
                _make_part(
                    f"shelf_beam_{s_idx + 1}",
                    "box",
                    [0.0, 0.0, sz, 0.0, 0.0, 0.0],
                    size=[W, D, 0.045],
                    color=beam_color if s_idx == 0 else deck_color,
                )
            )

    elif arch == "table":
        W = max(float(params.get("width", 1.40)), 0.4)
        D = max(float(params.get("depth", 0.80)), 0.4)
        H = max(float(params.get("height", 0.75)), 0.3)
        tt = max(float(params.get("top_thickness", 0.05)), 0.02)
        lw = max(float(params.get("leg_width", 0.06)), 0.02)
        leg_h = max(H - tt, 0.1)
        bbox = [W, D, H]

        leg_color = [0.22, 0.25, 0.29, 1.0]
        parts.append(
            _make_part(
                "table_top",
                "box",
                [0.0, 0.0, H * 0.5 - tt * 0.5, 0.0, 0.0, 0.0],
                size=[W, D, tt],
                color=base_color,
            )
        )
        lx = W * 0.5 - lw * 0.5 - 0.04
        ly = D * 0.5 - lw * 0.5 - 0.04
        lz = -H * 0.5 + leg_h * 0.5
        for idx, (px, py) in enumerate([(-lx, -ly), (lx, -ly), (-lx, ly), (lx, ly)]):
            parts.append(
                _make_part(
                    f"table_leg_{idx + 1}",
                    "box",
                    [px, py, lz, 0.0, 0.0, 0.0],
                    size=[lw, lw, leg_h],
                    color=leg_color,
                )
            )

    elif arch == "euro_pallet":
        L = max(float(params.get("length", 1.20)), 0.4)
        W = max(float(params.get("width", 0.80)), 0.4)
        H = max(float(params.get("height", 0.144)), 0.08)
        bbox = [L, W, H]
        # 3 bottom runners along X + 5 top slats along X
        runner_h = H * 0.65
        slat_h = H * 0.35
        z_bot = -H * 0.5 + runner_h * 0.5
        z_top = H * 0.5 - slat_h * 0.5
        for idx, ry in enumerate([-W * 0.42, 0.0, W * 0.42]):
            parts.append(
                _make_part(
                    f"pallet_runner_{idx + 1}",
                    "box",
                    [0.0, ry, z_bot, 0.0, 0.0, 0.0],
                    size=[L, W * 0.14, runner_h],
                    color=base_color,
                )
            )
        for idx in range(5):
            sy = -W * 0.42 + idx * (W * 0.84 / 4.0)
            parts.append(
                _make_part(
                    f"pallet_slat_{idx + 1}",
                    "box",
                    [0.0, sy, z_top, 0.0, 0.0, 0.0],
                    size=[L, W * 0.13, slat_h],
                    color=base_color,
                )
            )

    elif arch == "safety_fence":
        L = max(float(params.get("length", 2.0)), 0.4)
        H = max(float(params.get("height", 1.10)), 0.4)
        r = max(float(params.get("rail_radius", 0.03)), 0.015)
        bbox = [L, r * 3.0, H]
        parts = [
            _make_part(
                "fence_post_left",
                "cylinder",
                [-L * 0.5 + r, 0.0, 0.0, 0.0, 0.0, 0.0],
                radius=r * 1.2,
                length=H,
                color=base_color,
            ),
            _make_part(
                "fence_post_right",
                "cylinder",
                [+L * 0.5 - r, 0.0, 0.0, 0.0, 0.0, 0.0],
                radius=r * 1.2,
                length=H,
                color=base_color,
            ),
            _make_part(
                "fence_rail_top",
                "box",
                [0.0, 0.0, H * 0.45, 0.0, 0.0, 0.0],
                size=[L, r * 1.6, r * 1.8],
                color=base_color,
            ),
            _make_part(
                "fence_rail_mid",
                "box",
                [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                size=[L, r * 1.6, r * 1.6],
                color=[0.18, 0.20, 0.22, 1.0],
            ),
        ]

    elif arch == "jersey_barrier":
        L = max(float(params.get("length", 2.0)), 0.4)
        bw = max(float(params.get("bottom_width", 0.60)), 0.2)
        tw = max(float(params.get("top_width", 0.24)), 0.1)
        H = max(float(params.get("height", 0.85)), 0.3)
        bbox = [L, bw, H]
        lower_h = H * 0.38
        upper_h = H * 0.62
        parts = [
            _make_part(
                "barrier_base",
                "box",
                [0.0, 0.0, -H * 0.5 + lower_h * 0.5, 0.0, 0.0, 0.0],
                size=[L, bw, lower_h],
                color=base_color,
            ),
            _make_part(
                "barrier_upper",
                "box",
                [0.0, 0.0, H * 0.5 - upper_h * 0.5, 0.0, 0.0, 0.0],
                size=[L, tw, upper_h],
                color=base_color,
            ),
        ]

    elif arch == "traffic_cone":
        bw = max(float(params.get("base_width", 0.36)), 0.15)
        H = max(float(params.get("height", 0.55)), 0.2)
        bbox = [bw, bw, H]
        base_h = 0.04
        body_h = H - base_h
        parts = [
            _make_part(
                "cone_base",
                "box",
                [0.0, 0.0, -H * 0.5 + base_h * 0.5, 0.0, 0.0, 0.0],
                size=[bw, bw, base_h],
                color=[0.15, 0.16, 0.18, 1.0],
            ),
            _make_part(
                "cone_lower",
                "cylinder",
                [0.0, 0.0, -H * 0.5 + base_h + body_h * 0.3, 0.0, 0.0, 0.0],
                radius=bw * 0.32,
                length=body_h * 0.6,
                color=base_color,
            ),
            _make_part(
                "cone_stripe",
                "cylinder",
                [0.0, 0.0, -H * 0.5 + base_h + body_h * 0.68, 0.0, 0.0, 0.0],
                radius=bw * 0.22,
                length=body_h * 0.22,
                color=[0.95, 0.96, 0.98, 1.0],
            ),
            _make_part(
                "cone_tip",
                "cylinder",
                [0.0, 0.0, H * 0.5 - body_h * 0.1, 0.0, 0.0, 0.0],
                radius=bw * 0.14,
                length=body_h * 0.2,
                color=base_color,
            ),
        ]

    elif arch == "bollard":
        r = max(float(params.get("radius", 0.09)), 0.03)
        H = max(float(params.get("height", 0.95)), 0.2)
        bbox = [r * 2.6, r * 2.6, H]
        parts = [
            _make_part(
                "bollard_flange",
                "cylinder",
                [0.0, 0.0, -H * 0.5 + 0.015, 0.0, 0.0, 0.0],
                radius=r * 1.4,
                length=0.03,
                color=[0.2, 0.22, 0.25, 1.0],
            ),
            _make_part(
                "bollard_post",
                "cylinder",
                [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                radius=r,
                length=H,
                color=base_color,
            ),
        ]

    elif arch == "cardboard_box":
        dx = max(float(params.get("dx", 0.45)), 0.05)
        dy = max(float(params.get("dy", 0.35)), 0.05)
        dz = max(float(params.get("dz", 0.32)), 0.05)
        bbox = [dx, dy, dz]
        parts = [_make_part("box_body", "box", [0, 0, 0, 0, 0, 0], size=[dx, dy, dz], color=base_color)]

    elif arch == "road_straight":
        L = max(float(params.get("length", 8.0)), 1.0)
        W = max(float(params.get("width", 4.5)), 1.2)
        T = max(float(params.get("thickness", 0.06)), 0.02)
        cw = max(float(params.get("curb_width", 0.25)), 0.0)
        curb_h = T + 0.08
        total_w = W + 2.0 * cw
        bbox = [L, total_w, curb_h]
        z_road = -curb_h * 0.5 + T * 0.5
        z_line = z_road + T * 0.5 + 0.004
        white = [0.95, 0.96, 0.98, 1.0]
        yellow = [0.98, 0.80, 0.12, 1.0]
        curb_col = [0.62, 0.65, 0.69, 1.0]

        parts = [
            _make_part("asphalt_base", "box", [0.0, 0.0, z_road, 0, 0, 0], size=[L, W, T], color=base_color),
            _make_part("edge_line_left", "box", [0.0, W * 0.46, z_line, 0, 0, 0], size=[L, 0.10, 0.008], color=white),
            _make_part("edge_line_right", "box", [0.0, -W * 0.46, z_line, 0, 0, 0], size=[L, 0.10, 0.008], color=white),
        ]
        if cw > 0.02:
            parts.append(
                _make_part("curb_left", "box", [0.0, (W + cw) * 0.5, 0.0, 0, 0, 0], size=[L, cw, curb_h], color=curb_col)
            )
            parts.append(
                _make_part("curb_right", "box", [0.0, -(W + cw) * 0.5, 0.0, 0, 0, 0], size=[L, cw, curb_h], color=curb_col)
            )
        n_dashes = max(int(L / 1.6), 2)
        step_x = L / float(n_dashes)
        dash_len = step_x * 0.52
        for i in range(n_dashes):
            dx_pos = -L * 0.5 + (i + 0.5) * step_x
            parts.append(
                _make_part(f"center_dash_{i + 1}", "box", [dx_pos, 0.0, z_line, 0, 0, 0], size=[dash_len, 0.11, 0.008], color=yellow)
            )

    elif arch == "road_curved":
        R = max(float(params.get("radius", 6.0)), 2.0)
        W = max(float(params.get("width", 4.5)), 1.2)
        ang_deg = max(min(float(params.get("angle_deg", 90.0)), 180.0), 15.0)
        T = max(float(params.get("thickness", 0.06)), 0.02)
        n_seg = max(min(int(params.get("segments", 10)), 24), 4)
        ang_rad = math.radians(ang_deg)
        d_theta = ang_rad / float(n_seg)
        seg_len = 2.0 * R * math.sin(d_theta * 0.5) + 0.18
        yellow = [0.98, 0.80, 0.12, 1.0]
        white = [0.95, 0.96, 0.98, 1.0]

        # Center curve around its chord midpoint so the gizmo sits right on the road curve
        mid_ang = ang_rad * 0.5
        cx_off = R * math.cos(mid_ang)
        cy_off = R * math.sin(mid_ang)
        bbox = [R + W, R + W, T + 0.02]

        for i in range(n_seg):
            theta = (i + 0.5) * d_theta
            px = R * math.cos(theta) - cx_off
            py = R * math.sin(theta) - cy_off
            tangent_yaw = theta + math.pi * 0.5
            parts.append(
                _make_part(
                    f"curve_asphalt_{i + 1}",
                    "box",
                    [px, py, 0.0, 0.0, 0.0, tangent_yaw],
                    size=[seg_len, W, T],
                    color=base_color,
                )
            )
            if i % 2 == 0:
                parts.append(
                    _make_part(
                        f"curve_dash_{i + 1}",
                        "box",
                        [px, py, T * 0.5 + 0.005, 0.0, 0.0, tangent_yaw],
                        size=[seg_len * 0.55, 0.11, 0.008],
                        color=yellow,
                    )
                )
            # Outer & inner white edge lines
            r_in = max(R - W * 0.46, 0.3)
            r_out = R + W * 0.46
            len_in = 2.0 * r_in * math.sin(d_theta * 0.5) + 0.05
            len_out = 2.0 * r_out * math.sin(d_theta * 0.5) + 0.05
            parts.append(
                _make_part(
                    f"edge_in_{i + 1}",
                    "box",
                    [r_in * math.cos(theta) - cx_off, r_in * math.sin(theta) - cy_off, T * 0.5 + 0.005, 0.0, 0.0, tangent_yaw],
                    size=[len_in, 0.09, 0.008],
                    color=white,
                )
            )
            parts.append(
                _make_part(
                    f"edge_out_{i + 1}",
                    "box",
                    [r_out * math.cos(theta) - cx_off, r_out * math.sin(theta) - cy_off, T * 0.5 + 0.005, 0.0, 0.0, tangent_yaw],
                    size=[len_out, 0.09, 0.008],
                    color=white,
                )
            )

    elif arch == "road_intersection":
        L = max(float(params.get("arm_length", 10.0)), 4.0)
        W = max(float(params.get("width", 4.5)), 1.5)
        T = max(float(params.get("thickness", 0.06)), 0.02)
        bbox = [L, L, T + 0.02]
        white = [0.95, 0.96, 0.98, 1.0]
        yellow = [0.98, 0.80, 0.12, 1.0]
        z_line = T * 0.5 + 0.005
        arm_span = max((L - W) * 0.5, 0.6)
        arm_offset = W * 0.5 + arm_span * 0.5

        parts = [
            _make_part("road_arm_x", "box", [0, 0, 0, 0, 0, 0], size=[L, W, T], color=base_color),
            _make_part("road_arm_y", "box", [0, 0, 0.001, 0, 0, 0], size=[W, L, T], color=base_color),
            # 4 Stop lines
            _make_part("stop_x_pos", "box", [W * 0.54, -W * 0.23, z_line, 0, 0, 0], size=[0.22, W * 0.44, 0.008], color=white),
            _make_part("stop_x_neg", "box", [-W * 0.54, W * 0.23, z_line, 0, 0, 0], size=[0.22, W * 0.44, 0.008], color=white),
            _make_part("stop_y_pos", "box", [W * 0.23, W * 0.54, z_line, 0, 0, 0], size=[W * 0.44, 0.22, 0.008], color=white),
            _make_part("stop_y_neg", "box", [-W * 0.23, -W * 0.54, z_line, 0, 0, 0], size=[W * 0.44, 0.22, 0.008], color=white),
            # 4 Center divider lines on the arms
            _make_part("div_x_pos", "box", [arm_offset, 0, z_line, 0, 0, 0], size=[arm_span * 0.8, 0.10, 0.008], color=yellow),
            _make_part("div_x_neg", "box", [-arm_offset, 0, z_line, 0, 0, 0], size=[arm_span * 0.8, 0.10, 0.008], color=yellow),
            _make_part("div_y_pos", "box", [0, arm_offset, z_line, 0, 0, 0], size=[0.10, arm_span * 0.8, 0.008], color=yellow),
            _make_part("div_y_neg", "box", [0, -arm_offset, z_line, 0, 0, 0], size=[0.10, arm_span * 0.8, 0.008], color=yellow),
        ]

    elif arch == "road_crosswalk":
        L = max(float(params.get("length", 4.0)), 1.5)
        W = max(float(params.get("width", 4.5)), 1.5)
        T = max(float(params.get("thickness", 0.06)), 0.02)
        n_stripes = max(min(int(params.get("stripes", 6)), 16), 2)
        bbox = [L, W, T + 0.02]
        white = [0.96, 0.97, 0.99, 1.0]
        z_line = T * 0.5 + 0.005

        parts = [
            _make_part("crosswalk_asphalt", "box", [0, 0, 0, 0, 0, 0], size=[L, W, T], color=base_color),
        ]
        usable_w = W * 0.84
        step_y = usable_w / float(n_stripes)
        stripe_w = step_y * 0.55
        stripe_l = min(L * 0.68, 2.6)
        for i in range(n_stripes):
            sy = -usable_w * 0.5 + (i + 0.5) * step_y
            parts.append(
                _make_part(
                    f"zebra_stripe_{i + 1}",
                    "box",
                    [0.0, sy, z_line, 0.0, 0.0, 0.0],
                    size=[stripe_l, stripe_w, 0.009],
                    color=white,
                )
            )

    elif arch == "roundabout":
        r_out = max(float(params.get("outer_radius", 6.5)), 2.5)
        r_in = min(max(float(params.get("inner_radius", 2.2)), 0.6), r_out - 1.2)
        T = max(float(params.get("thickness", 0.06)), 0.02)
        island_h = T + 0.14
        bbox = [r_out * 2.0, r_out * 2.0, island_h]
        z_road = -island_h * 0.5 + T * 0.5

        parts = [
            _make_part(
                "roundabout_asphalt",
                "cylinder",
                [0.0, 0.0, z_road, 0.0, 0.0, 0.0],
                radius=r_out,
                length=T,
                color=base_color,
            ),
            _make_part(
                "island_curb",
                "cylinder",
                [0.0, 0.0, -island_h * 0.5 + (T + 0.08) * 0.5, 0.0, 0.0, 0.0],
                radius=r_in + 0.22,
                length=T + 0.08,
                color=[0.85, 0.87, 0.90, 1.0],
            ),
            _make_part(
                "island_grass",
                "cylinder",
                [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                radius=r_in,
                length=island_h,
                color=[0.20, 0.58, 0.26, 1.0],
            ),
        ]
        r_mid = (r_out + r_in + 0.22) * 0.5
        n_dashes = 16
        for i in range(n_dashes):
            ang = (2.0 * math.pi * i) / float(n_dashes)
            parts.append(
                _make_part(
                    f"lane_dash_{i + 1}",
                    "box",
                    [
                        r_mid * math.cos(ang),
                        r_mid * math.sin(ang),
                        z_road + T * 0.5 + 0.005,
                        0.0,
                        0.0,
                        ang + math.pi * 0.5,
                    ],
                    size=[0.75, 0.11, 0.008],
                    color=[0.96, 0.97, 0.99, 1.0],
                )
            )

    elif arch == "speed_bump":
        W = max(float(params.get("width", 4.0)), 0.8)
        D = max(float(params.get("depth", 0.48)), 0.2)
        H = max(float(params.get("height", 0.07)), 0.02)
        n_st = max(min(int(params.get("stripes", 8)), 20), 2)
        bbox = [D, W, H]
        seg_w = W / float(n_st)
        yellow = [0.96, 0.78, 0.06, 1.0]
        black = [0.14, 0.15, 0.17, 1.0]
        for i in range(n_st):
            sy = -W * 0.5 + (i + 0.5) * seg_w
            parts.append(
                _make_part(
                    f"bump_seg_{i + 1}",
                    "box",
                    [0.0, sy, 0.0, 0.0, 0.0, 0.0],
                    size=[D, seg_w * 0.98, H],
                    color=yellow if i % 2 == 0 else black,
                )
            )

    node["parts"] = parts
    node.setdefault("geometry", {})
    node["geometry"]["shape"] = "box"
    node["geometry"]["size"] = [clean_float(bbox[0]), clean_float(bbox[1]), clean_float(bbox[2])]
    return normalize_node_in_place(node)


def create_archetype_node(
    archetype: str,
    name: Optional[str] = None,
    pose: Optional[List[float]] = None,
    custom_params: Optional[Dict[str, Any]] = None,
    node_id: Optional[str] = None,
) -> Dict[str, Any]:
    """Create a normalized node from `ARCHETYPE_CATALOG` with its parts and physics ready."""
    if archetype not in ARCHETYPE_CATALOG:
        raise ValueError(f"Unknown archetype '{archetype}'.")

    spec = ARCHETYPE_CATALOG[archetype]
    params = copy.deepcopy(spec["default_params"])
    if custom_params:
        params.update(custom_params)

    node: Dict[str, Any] = {
        "id": node_id or generate_node_id(archetype),
        "name": name or archetype,
        "type": "primitive",
        "archetype": archetype,
        "archetype_params": params,
        "parent_id": None,
        "folder_id": spec["folder_id"],
        "visible": True,
        "locked": False,
        "static": bool(spec["static"]),
        "pose": [0.0, 0.0, 0.5, 0.0, 0.0, 0.0],
        "scale": [1.0, 1.0, 1.0],
        "geometry": {"shape": "box", "size": [1.0, 1.0, 1.0]},
        "material": {
            "color": list(spec["default_color"]),
            "roughness": 0.65 if spec["static"] else 0.45,
            "metalness": 0.15,
        },
        "collision": {
            "enabled": True,
            "strategy": "primitive",
            "surface_preset": "warehouse_floor",
            "mu1": 0.75,
            "mu2": 0.75,
        },
        "inertial": {
            "mode": "auto",
            "material_preset": spec["material_preset"],
            "center_of_mass": [0.0, 0.0, 0.0],
        },
        "parts": [],
    }

    rebuild_archetype_node(node)
    # Place resting on ground Z = 0 by default if pose wasn't explicitly specified
    if pose is None:
        h = float(node["geometry"]["size"][2])
        node["pose"] = [0.0, 0.0, clean_float(h * 0.5), 0.0, 0.0, 0.0]
    else:
        node["pose"] = [clean_float(float(v)) for v in pose[:6]]

    return normalize_node_in_place(node)

