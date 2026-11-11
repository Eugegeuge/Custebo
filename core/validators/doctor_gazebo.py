"""
Doctor Gazebo Real-Time Linter (`core/validators/doctor_gazebo.py`).
Checks the Scene Graph for physical anomalies, overlapping dynamic bodies, and invalid inertias.
"""

from __future__ import annotations

from typing import Any, Dict, List
from core.math_physics.transforms import compute_local_half_extents, compute_world_aabb
from core.schema.scene_graph import SceneGraphDocument
from tools.validate_sdf import is_positive_definite_inertia


def diagnose_scene_graph(doc: SceneGraphDocument) -> List[Dict[str, Any]]:
    """
    Return a list of diagnostic alerts (ERROR / WARNING) for the current SceneGraphDocument.
    """
    alerts: List[Dict[str, Any]] = []
    dynamic_aabbs: List[tuple[str, str, tuple[tuple[float, float, float], tuple[float, float, float]]]] = []

    for node in doc.nodes:
        nid = node.get("id", "")
        name = node.get("name", nid)
        ntype = node.get("type", "primitive")
        is_static = bool(node.get("static", True))

        if ntype in ("primitive", "mesh") and not is_static:
            inertial = node.get("inertial", {})
            mass = float(inertial.get("mass", 0.0))
            if mass <= 0.0:
                alerts.append(
                    {
                        "severity": "ERROR",
                        "code": "ZERO_MASS",
                        "node_id": nid,
                        "message": f"El modelo '{name}' es dinámico pero tiene masa {mass} kg (<= 0).",
                    }
                )
            else:
                ok, reason = is_positive_definite_inertia(
                    float(inertial.get("ixx", 0.0)),
                    float(inertial.get("iyy", 0.0)),
                    float(inertial.get("izz", 0.0)),
                    float(inertial.get("ixy", 0.0)),
                    float(inertial.get("ixz", 0.0)),
                    float(inertial.get("iyz", 0.0)),
                )
                if not ok:
                    alerts.append(
                        {
                            "severity": "ERROR",
                            "code": "INVALID_INERTIA",
                            "node_id": nid,
                            "message": f"Inercia físicamente inválida en '{name}': {reason}",
                        }
                    )

            geom = node.get("geometry", {})
            half_ext = compute_local_half_extents(
                geom.get("shape", "box"),
                size=geom.get("size", [1.0, 1.0, 1.0]),
                radius=float(geom.get("radius", 0.5)),
                length=float(geom.get("length", 1.0)),
            )
            aabb = compute_world_aabb(
                node.get("pose", [0, 0, 0, 0, 0, 0]),
                half_ext,
                node.get("scale", [1.0, 1.0, 1.0]),
            )
            # Check penetration below ground plane z = 0
            if aabb[0][2] < -1e-3:
                alerts.append(
                    {
                        "severity": "WARNING",
                        "code": "BELOW_GROUND",
                        "node_id": nid,
                        "message": f"El objeto dinámico '{name}' atraviesa el suelo Z=0 (z_min={aabb[0][2]:.3f}m). Usa 'Drop to Ground' (Espacio).",
                    }
                )
            dynamic_aabbs.append((nid, name, aabb))

    # Check overlapping dynamic objects at t = 0
    for i in range(len(dynamic_aabbs)):
        id_a, name_a, (min_a, max_a) = dynamic_aabbs[i]
        for j in range(i + 1, len(dynamic_aabbs)):
            id_b, name_b, (min_b, max_b) = dynamic_aabbs[j]
            overlap_x = (min_a[0] < max_b[0] - 1e-4) and (max_a[0] > min_b[0] + 1e-4)
            overlap_y = (min_a[1] < max_b[1] - 1e-4) and (max_a[1] > min_b[1] + 1e-4)
            overlap_z = (min_a[2] < max_b[2] - 1e-4) and (max_a[2] > min_b[2] + 1e-4)
            if overlap_x and overlap_y and overlap_z:
                alerts.append(
                    {
                        "severity": "WARNING",
                        "code": "DYNAMIC_OVERLAP",
                        "node_id": id_a,
                        "message": f"Los objetos dinámicos '{name_a}' y '{name_b}' se solapan en t=0.",
                    }
                )

    return alerts

