"""
Custebo Local Backend Server (`server/app.py`).

Provides a zero-dependency Python 3.12 HTTP & JSON RPC server connecting the
Three.js 3D Viewport UI with:
  - The authoritative `CommandManager` & `SceneGraphDocument` (Undo/Redo, Commands)
  - The deterministic `SDFCompiler` (Gazebo Harmonic SDF 1.10 & Classic SDF 1.7)
  - Phase 2 Custom Objects Library ("Objetos Personalizados": CAD Meshes .stl/.obj/.dae/.glb,
    User Prefabs, Gazebo Fuel Cloud Models, ROS 2 Workspace Scanner, and Round-Trip SDF Parser)
  - Real-time `Doctor Gazebo` & native `gz sdf -k` validation
  - Local filesystem persistence (`projects/*.gproj.json`, `exports/*.sdf`, `assets/meshes/`)
  - Direct `gz sim` launcher subprocess ("Play in Gazebo")
"""

from __future__ import annotations

import argparse
import base64
import copy
import json
import mimetypes
import subprocess
import sys
import uuid
from http import HTTPStatus
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Any, Dict, List, Optional
from urllib.parse import parse_qs, unquote, urlparse

ROOT_DIR = Path(__file__).resolve().parent.parent
if str(ROOT_DIR) not in sys.path:
    sys.path.insert(0, str(ROOT_DIR))

from core.commands import (
    AddMultipleNodesCommand,
    AddNodeCommand,
    BatchUpdatePosesCommand,
    CommandManager,
    DeleteNodesCommand,
    DropToGroundCommand,
    DuplicateNodeCommand,
    ExplodeCompositeCommand,
    UpdateNodeCommand,
    UpdateWorldSettingsCommand,
)
from core.compilers import SDFCompiler
from core.compilers.xacro_compiler import compile_scene_to_xacro
from core.importers import (
    analyze_mesh_file,
    ensure_sample_cad_meshes,
    generate_collision_proxy_meshes,
    parse_sdf_to_scene_graph,
    scan_ros2_workspaces,
    search_gazebo_fuel,
)
from core.math_physics import MATERIAL_PRESETS, SURFACE_PRESETS
from core.procedural import (
    align_or_distribute_nodes,
    create_parametric_terrain_node,
    extrude_polyline_to_walls,
    generate_grid_array,
    generate_scatter_brush,
    generate_spline_clones,
    reconstruct_3d_walls_from_pgm,
    snap_node_to_nearest_neighbor,
)
from core.ros2.nav2_map_generator import generate_nav2_occupancy_map
from core.ros2.navigation_tools import (
    create_dynamic_actor_node,
    create_robot_spawn_node,
    create_static_sensor_node,
    create_waypoint_node,
    export_nav2_waypoints_yaml,
    generate_ros_gz_bridge_yaml,
)
from core.schema import (
    ARCHETYPE_CATALOG,
    SceneGraphDocument,
    create_archetype_node,
    create_ground_plane_node,
    create_light_node,
    create_primitive_node,
    generate_node_id,
    normalize_node_in_place,
)
from core.validators import diagnose_scene_graph
from tools.validate_sdf import find_gz_binary, validate_sdf_content


FRONTEND_DIR = ROOT_DIR / "frontend"
PROJECTS_DIR = ROOT_DIR / "projects"
EXPORTS_DIR = ROOT_DIR / "exports"
ASSETS_DIR = ROOT_DIR / "assets"
MESHES_DIR = ASSETS_DIR / "meshes"
CUSTOM_OBJECTS_FILE = ASSETS_DIR / "custom_objects.json"


class EditorSessionState:
    """Holds the active SceneGraphDocument, CommandManager, and Custom Objects Library."""

    def __init__(self) -> None:
        PROJECTS_DIR.mkdir(parents=True, exist_ok=True)
        EXPORTS_DIR.mkdir(parents=True, exist_ok=True)
        MESHES_DIR.mkdir(parents=True, exist_ok=True)

        ensure_sample_cad_meshes(MESHES_DIR)
        self.custom_objects: List[Dict[str, Any]] = self._load_or_init_custom_objects()

        self.manager = CommandManager(SceneGraphDocument.create_default("custebo_world"))
        self._seed_starter_objects()

    def _load_or_init_custom_objects(self) -> List[Dict[str, Any]]:
        if CUSTOM_OBJECTS_FILE.is_file():
            try:
                data = json.loads(CUSTOM_OBJECTS_FILE.read_text(encoding="utf-8"))
                if isinstance(data, list) and data:
                    return data
            except Exception:
                pass

        # Seed default custom objects from sample CAD meshes
        starter_custom: List[Dict[str, Any]] = []
        samples = [
            ("custom_agv_chassis", "🦾 Chasis Robot AGV (.STL)", "agv_chassis.stl", "m", "Z_UP", "aluminum", [0.22, 0.58, 0.92, 1.0]),
            ("custom_flange_mm", "⚙ Brida CAD 600mm (.STL mm→m)", "cad_flange_mm.stl", "mm", "Z_UP", "steel", [0.72, 0.76, 0.82, 1.0]),
            ("custom_conveyor", "🛤 Módulo Transportador (.OBJ)", "conveyor_module.obj", "m", "Z_UP", "steel", [0.18, 0.72, 0.54, 1.0]),
        ]
        for cid, label, fname, unit, up_axis, mat_preset, col in samples:
            mpath = MESHES_DIR / fname
            if mpath.is_file():
                meta = analyze_mesh_file(mpath, unit=unit, up_axis=up_axis, material_preset=mat_preset)
                proxies = generate_collision_proxy_meshes(mpath, unit=unit, up_axis=up_axis)
                starter_custom.append(
                    {
                        "id": cid,
                        "label": label,
                        "kind": "cad_mesh",
                        "filename": fname,
                        "unit": unit,
                        "up_axis": up_axis,
                        "material_preset": mat_preset,
                        "color": col,
                        "analysis": meta,
                        "proxies": proxies,
                    }
                )
        self._save_custom_objects_list(starter_custom)
        return starter_custom

    def _save_custom_objects_list(self, items: List[Dict[str, Any]]) -> None:
        CUSTOM_OBJECTS_FILE.write_text(
            json.dumps(items, indent=2, ensure_ascii=False), encoding="utf-8"
        )

    def _seed_starter_objects(self) -> None:
        """Add starter building blocks: doorway wall, warehouse rack, and dynamic crate."""
        wall_door = create_archetype_node(
            archetype="wall_doorway",
            name="muro_entrada",
            pose=[0.0, 3.0, 1.25, 0.0, 0.0, 0.0],
            node_id="node_starter_wall_door",
        )
        rack = create_archetype_node(
            archetype="warehouse_rack",
            name="estanteria_A1",
            pose=[-2.5, 0.0, 1.10, 0.0, 0.0, 1.570796],
            node_id="node_starter_rack",
        )
        box = create_primitive_node(
            name="wooden_crate",
            shape="box",
            pose=[0.0, 0.0, 0.4, 0.0, 0.0, 0.0],
            size=[0.8, 0.8, 0.8],
            color=[0.85, 0.55, 0.24, 1.0],
            static=False,
            material_preset="wood",
            folder_id="folder_props",
            node_id="node_starter_box",
        )
        self.manager.doc.nodes.extend([wall_door, rack, box])

    def build_node_from_custom_entry(self, entry: Dict[str, Any]) -> Dict[str, Any]:
        """Instantiate a Scene Graph node from a Custom Object entry (CAD mesh or saved prefab)."""
        if entry.get("kind") == "saved_prefab" and isinstance(entry.get("node_template"), dict):
            node = copy.deepcopy(entry["node_template"])
            node["id"] = generate_node_id("custom")
            node["name"] = self.manager.doc.ensure_unique_name(node.get("name", "objeto_custom"))
            node["locked"] = False
            return normalize_node_in_place(node)

        # CAD Mesh custom entry
        fname = entry.get("filename", "agv_chassis.stl")
        analysis = entry.get("analysis", {})
        proxies = entry.get("proxies", {})
        bsize = analysis.get("bounding_size", [1.0, 1.0, 1.0])
        brad = float(analysis.get("bounding_radius", 0.5))
        blen = float(analysis.get("bounding_length", 1.0))
        z0 = max(float(bsize[2]) * 0.5, 0.1)

        base_name = Path(fname).stem
        node: Dict[str, Any] = {
            "id": generate_node_id("mesh"),
            "name": self.manager.doc.ensure_unique_name(base_name),
            "type": "mesh",
            "custom_object_id": entry.get("id"),
            "folder_id": "folder_props",
            "visible": True,
            "locked": False,
            "static": False,
            "pose": [0.0, 0.0, round(z0, 4), 0.0, 0.0, 0.0],
            "scale": [1.0, 1.0, 1.0],
            "mesh": {
                "uri": f"meshes/{fname}",
                "filename": fname,
                "unit": entry.get("unit", "m"),
                "unit_scale": float(analysis.get("unit_scale", 1.0)),
                "up_axis": entry.get("up_axis", "Z_UP"),
                "triangle_count": int(analysis.get("triangle_count", 12)),
                "volume_m3": float(analysis.get("volume_m3", 0.1)),
                "decimated_uri": proxies.get("decimated_uri", f"meshes/{fname}"),
                "convex_hull_uri": proxies.get("convex_hull_uri", f"meshes/{fname}"),
            },
            "geometry": {
                "shape": "box",
                "size": list(bsize),
                "radius": brad,
                "length": blen,
            },
            "material": {
                "color": list(entry.get("color", [0.22, 0.58, 0.92, 1.0])),
                "roughness": 0.4,
                "metalness": 0.3,
            },
            "collision": {
                "enabled": True,
                "strategy": "aabb",
                "surface_preset": "warehouse_floor",
                "mu1": 0.7,
                "mu2": 0.7,
            },
            "inertial": copy.deepcopy(
                analysis.get(
                    "inertial",
                    {
                        "mode": "auto",
                        "material_preset": entry.get("material_preset", "steel"),
                        "center_of_mass": analysis.get("center_of_mass", [0.0, 0.0, 0.0]),
                    },
                )
            ),
        }
        node["inertial"]["mode"] = "auto"
        node["inertial"]["material_preset"] = entry.get("material_preset", "steel")
        node["inertial"]["center_of_mass"] = analysis.get("center_of_mass", [0.0, 0.0, 0.0])
        return normalize_node_in_place(node)

    def build_full_Payload(self) -> Dict[str, Any]:
        sdf_xml, line_map = SDFCompiler.compile(self.manager.doc)
        doctor_alerts = diagnose_scene_graph(self.manager.doc)
        return {
            "ok": True,
            "scene_graph": self.manager.doc.to_dict(),
            "history": self.manager.get_history_state(),
            "sdf_preview": sdf_xml,
            "sdf_line_map": line_map,
            "doctor_alerts": doctor_alerts,
            "custom_objects": self.custom_objects,
            "presets": {
                "materials": MATERIAL_PRESETS,
                "surfaces": SURFACE_PRESETS,
                "archetypes": ARCHETYPE_CATALOG,
            },
        }


SESSION = EditorSessionState()


class CusteboRequestHandler(SimpleHTTPRequestHandler):
    """Serves static files from `frontend/`, `/meshes/*` from `assets/meshes/`, and `/api/*`."""

    def __init__(self, *args: Any, **kwargs: Any) -> None:
        super().__init__(*args, directory=str(FRONTEND_DIR), **kwargs)

    def log_message(self, format: str, *args: Any) -> None:
        pass

    def _send_json(self, payload: Dict[str, Any], status: int = HTTPStatus.OK) -> None:
        raw = json.dumps(payload, ensure_ascii=False).encode("utf-8")
        self.send_response(status)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Content-Length", str(len(raw)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(raw)

    def _read_json_body(self) -> Dict[str, Any]:
        length = int(self.headers.get("Content-Length", "0"))
        if length <= 0:
            return {}
        raw = self.rfile.read(length).decode("utf-8")
        return json.loads(raw) if raw else {}

    def do_GET(self) -> None:
        parsed = urlparse(self.path)
        if parsed.path == "/api/state":
            self._send_json(SESSION.build_full_Payload())
            return
        if parsed.path == "/api/projects":
            files = sorted(p.name for p in PROJECTS_DIR.glob("*.json"))
            self._send_json({"ok": True, "projects": files})
            return
        if parsed.path == "/api/fuel_search":
            qs = parse_qs(parsed.query)
            q = qs.get("q", [""])[0]
            fuel_models = search_gazebo_fuel(q)
            ros_models = scan_ros2_workspaces()
            self._send_json({"ok": True, "fuel_models": fuel_models, "ros_models": ros_models})
            return
        if parsed.path.startswith("/meshes/"):
            rel_name = Path(unquote(parsed.path[len("/meshes/") :])).name
            mesh_file = MESHES_DIR / rel_name
            if mesh_file.is_file():
                data = mesh_file.read_bytes()
                self.send_response(HTTPStatus.OK)
                self.send_header("Content-Type", "application/octet-stream")
                self.send_header("Content-Length", str(len(data)))
                self.end_headers()
                self.wfile.write(data)
                return
            self.send_error(HTTPStatus.NOT_FOUND, f"Mesh not found: {rel_name}")
            return
        super().do_GET()

    def do_POST(self) -> None:
        parsed = urlparse(self.path)
        try:
            body = self._read_json_body()
        except Exception as exc:
            self._send_json({"ok": False, "error": f"Invalid JSON body: {exc}"}, HTTPStatus.BAD_REQUEST)
            return

        try:
            if parsed.path == "/api/command":
                self._handle_command(body)
            elif parsed.path == "/api/undo":
                SESSION.manager.undo()
                self._send_json(SESSION.build_full_Payload())
            elif parsed.path == "/api/redo":
                SESSION.manager.redo()
                self._send_json(SESSION.build_full_Payload())
            elif parsed.path == "/api/upload_custom_mesh":
                self._handle_upload_custom_mesh(body)
            elif parsed.path == "/api/save_custom_object":
                self._handle_save_custom_object(body)
            elif parsed.path == "/api/delete_custom_object":
                self._handle_delete_custom_object(body)
            elif parsed.path == "/api/import_sdf":
                self._handle_import_sdf(body)
            elif parsed.path in ("/api/import_pgm_walls", "/api/import_pgm_map"):
                self._handle_import_pgm_walls(body)
            elif parsed.path == "/api/generate_nav2_map":
                self._handle_generate_nav2_map(body)
            elif parsed.path == "/api/export_nav2_waypoints":
                self._handle_export_nav2_waypoints()
            elif parsed.path == "/api/export_ros_gz_bridge":
                self._handle_export_ros_gz_bridge()
            elif parsed.path == "/api/export_xacro":
                self._handle_export_xacro(body)
            elif parsed.path == "/api/new_project":
                world_name = str(body.get("world_name", "custebo_world")).strip() or "custebo_world"
                SESSION.manager = CommandManager(SceneGraphDocument.create_default(world_name))
                self._send_json(SESSION.build_full_Payload())
            elif parsed.path == "/api/load_project":
                self._handle_load_project(body)
            elif parsed.path == "/api/save_project":
                self._handle_save_project(body)
            elif parsed.path == "/api/export_sdf":
                self._handle_export_sdf(body)
            elif parsed.path == "/api/validate_sdf":
                self._handle_validate_sdf(body)
            elif parsed.path == "/api/play_gazebo":
                self._handle_play_gazebo()
            else:
                self._send_json({"ok": False, "error": f"Unknown endpoint {parsed.path}"}, HTTPStatus.NOT_FOUND)
        except Exception as exc:
            self._send_json({"ok": False, "error": str(exc)}, HTTPStatus.INTERNAL_SERVER_ERROR)

    def _handle_command(self, body: Dict[str, Any]) -> None:
        cmd_type = body.get("type", "")
        created_node_id: Optional[str] = None

        if cmd_type == "add_primitive":
            shape = str(body.get("shape", "box"))
            name = str(body.get("name", shape))
            pose = body.get("pose")
            size = body.get("size")
            radius = float(body.get("radius", 0.5))
            length = float(body.get("length", 1.0))
            static = bool(body.get("static", False))
            material_preset = str(body.get("material_preset", "wood"))
            folder_id = str(body.get("folder_id", "folder_props"))
            if shape == "ground_plane":
                node = create_ground_plane_node(name=name, folder_id="folder_env")
            else:
                node = create_primitive_node(
                    name=name,
                    shape=shape,
                    pose=pose,
                    size=size,
                    radius=radius,
                    length=length,
                    static=static,
                    material_preset=material_preset,
                    folder_id=folder_id,
                )
            cmd = AddNodeCommand(node)
            SESSION.manager.execute(cmd)
            created_node_id = cmd.node_data["id"]

        elif cmd_type == "add_archetype":
            archetype = str(body.get("archetype", "wall"))
            name = body.get("name")
            pose = body.get("pose")
            custom_params = body.get("params")
            node = create_archetype_node(
                archetype=archetype,
                name=name,
                pose=pose,
                custom_params=custom_params,
            )
            cmd = AddNodeCommand(node)
            SESSION.manager.execute(cmd)
            created_node_id = cmd.node_data["id"]

        elif cmd_type == "add_custom_object":
            cid = str(body.get("custom_id", ""))
            entry = next((c for c in SESSION.custom_objects if c.get("id") == cid), None)
            if entry is None:
                self._send_json({"ok": False, "error": f"Objeto personalizado '{cid}' no encontrado."}, HTTPStatus.NOT_FOUND)
                return
            node = SESSION.build_node_from_custom_entry(entry)
            cmd = AddNodeCommand(node)
            SESSION.manager.execute(cmd)
            created_node_id = cmd.node_data["id"]

        elif cmd_type == "add_fuel_include":
            uri = str(body.get("uri", "")).strip()
            name = str(body.get("name", "fuel_model")).strip()
            size = body.get("size", [1.0, 1.0, 1.0])
            color = body.get("color", [0.30, 0.62, 0.90, 1.0])
            is_static = bool(body.get("static", True))
            proxy_arch = body.get("proxy_archetype")

            parts = []
            if proxy_arch and proxy_arch in ARCHETYPE_CATALOG:
                temp_arch = create_archetype_node(proxy_arch)
                parts = temp_arch.get("parts", [])
                size = temp_arch.get("geometry", {}).get("size", size)

            z0 = round(float(size[2]) * 0.5, 4)
            inc_node: Dict[str, Any] = {
                "id": generate_node_id("include"),
                "name": SESSION.manager.doc.ensure_unique_name(name),
                "type": "include",
                "folder_id": "folder_props",
                "visible": True,
                "locked": False,
                "static": is_static,
                "pose": [0.0, 0.0, z0, 0.0, 0.0, 0.0],
                "scale": [1.0, 1.0, 1.0],
                "include": {"uri": uri},
                "geometry": {"shape": "box", "size": list(size)},
                "material": {"color": list(color), "roughness": 0.5, "metalness": 0.2},
                "parts": parts,
            }
            cmd = AddNodeCommand(inc_node)
            SESSION.manager.execute(cmd)
            created_node_id = cmd.node_data["id"]

        elif cmd_type == "explode_composite":
            node_id = str(body["node_id"])
            exp_cmd = ExplodeCompositeCommand(node_id)
            SESSION.manager.execute(exp_cmd)
            if exp_cmd.spawned_nodes:
                created_node_id = exp_cmd.spawned_nodes[0]["id"]

        elif cmd_type == "add_light":
            light_type = str(body.get("light_type", "point"))
            name = str(body.get("name", f"{light_type}_light"))
            pose = body.get("pose", [0.0, 0.0, 4.0, 0.0, 0.0, 0.0])
            node = create_light_node(name=name, light_type=light_type, pose=pose)
            cmd = AddNodeCommand(node)
            SESSION.manager.execute(cmd)
            created_node_id = cmd.node_data["id"]

        elif cmd_type == "update_node":
            node_id = str(body["node_id"])
            patch = dict(body.get("patch", {}))
            label = body.get("label")
            coalesce_key = body.get("coalesce_key")

            # If unit or up_axis was changed on a mesh node, re-analyze mesh file automatically
            target_node = SESSION.manager.doc.get_node(node_id)
            if target_node and target_node.get("type") == "mesh" and "mesh" in patch:
                m_info = dict(target_node.get("mesh", {}))
                m_info.update(patch["mesh"])
                m_path = MESHES_DIR / Path(m_info.get("filename", "")).name
                if m_path.is_file():
                    unit = m_info.get("unit", "m")
                    up_axis = m_info.get("up_axis", "Z_UP")
                    mat_preset = target_node.get("inertial", {}).get("material_preset", "steel")
                    analysis = analyze_mesh_file(m_path, unit=unit, up_axis=up_axis, material_preset=mat_preset)
                    proxies = generate_collision_proxy_meshes(m_path, unit=unit, up_axis=up_axis)
                    patch["mesh"]["unit_scale"] = analysis["unit_scale"]
                    patch["mesh"]["decimated_uri"] = proxies["decimated_uri"]
                    patch["mesh"]["convex_hull_uri"] = proxies["convex_hull_uri"]
                    patch.setdefault("geometry", {})
                    patch["geometry"]["size"] = analysis["bounding_size"]
                    patch["geometry"]["radius"] = analysis["bounding_radius"]
                    patch["geometry"]["length"] = analysis["bounding_length"]
                    patch["pose"] = list(target_node.get("pose", [0, 0, 0.5, 0, 0, 0]))
                    patch["pose"][2] = round(float(analysis["bounding_size"][2]) * 0.5, 4)

            SESSION.manager.execute(
                UpdateNodeCommand(
                    node_id=node_id,
                    patch=patch,
                    label=label,
                    coalesce_key=coalesce_key,
                )
            )

        elif cmd_type == "delete_nodes":
            node_ids = list(body.get("node_ids", []))
            if node_ids:
                SESSION.manager.execute(DeleteNodesCommand(node_ids))

        elif cmd_type == "duplicate_node":
            node_id = str(body["node_id"])
            dup_cmd = DuplicateNodeCommand(node_id)
            SESSION.manager.execute(dup_cmd)
            if dup_cmd.created_node:
                created_node_id = dup_cmd.created_node["id"]

        elif cmd_type == "drop_to_ground":
            node_id = str(body["node_id"])
            raw_gz = body.get("ground_z")
            ground_z = float(raw_gz) if isinstance(raw_gz, (int, float)) else None
            SESSION.manager.execute(DropToGroundCommand(node_id, ground_z=ground_z))

        elif cmd_type == "snap_to_neighbor":
            node_id = str(body["node_id"])
            new_pose = snap_node_to_nearest_neighbor(SESSION.manager.doc, node_id)
            SESSION.manager.execute(
                BatchUpdatePosesCommand({node_id: new_pose}, label=f"Snap '{node_id}' to Neighbor")
            )

        elif cmd_type == "align_distribute":
            node_ids = list(body.get("node_ids", []))
            if not node_ids:
                node_ids = [
                    n["id"]
                    for n in SESSION.manager.doc.nodes
                    if n.get("type") not in ("ground_plane", "light") and not n.get("locked")
                ]
            mode = str(body.get("mode", "distribute_x"))
            spacing = float(body.get("spacing", 1.5))
            poses_map = align_or_distribute_nodes(SESSION.manager.doc, node_ids, mode=mode, spacing=spacing)
            if poses_map:
                SESSION.manager.execute(
                    BatchUpdatePosesCommand(poses_map, label=f"CAD {mode} ({len(poses_map)} objects)")
                )

        elif cmd_type in ("extrude_floorplan_walls", "extrude_floorplan"):
            points_xy = list(body.get("points_xy", body.get("points", [])))
            thickness = float(body.get("thickness", 0.20))
            height = float(body.get("height", 2.50))
            closed_loop = bool(body.get("closed_loop", body.get("closed", False)))
            wall_type = str(body.get("wall_type", "wall"))
            walls = extrude_polyline_to_walls(
                points_xy=points_xy,
                thickness=thickness,
                height=height,
                closed_loop=closed_loop,
                wall_type=wall_type,
            )
            if walls:
                cmd = AddMultipleNodesCommand(walls, label=f"Trace {len(walls)} 3D Wall(s)")
                SESSION.manager.execute(cmd)
                created_node_id = cmd.nodes_list[-1]["id"]

        elif cmd_type in ("generate_grid_array", "grid_array_clone"):
            node_id = str(body.get("node_id", ""))
            src = SESSION.manager.doc.get_node(node_id)
            if src is None:
                self._send_json({"ok": False, "error": "Selecciona un objeto base primero."}, HTTPStatus.BAD_REQUEST)
                return
            clones = generate_grid_array(
                source_node=src,
                count_x=int(body.get("count_x", 3)),
                count_y=int(body.get("count_y", 2)),
                count_z=int(body.get("count_z", 1)),
                spacing_x=float(body.get("spacing_x", 2.5)),
                spacing_y=float(body.get("spacing_y", 2.5)),
                spacing_z=float(body.get("spacing_z", 2.0)),
            )
            if clones:
                cmd = AddMultipleNodesCommand(clones, label=f"Grid Array ({len(clones)} clones)")
                SESSION.manager.execute(cmd)
                created_node_id = cmd.nodes_list[-1]["id"]

        elif cmd_type in ("generate_spline_clones", "spline_clone"):
            node_id = str(body.get("node_id", ""))
            src = SESSION.manager.doc.get_node(node_id)
            if src is None:
                src = create_archetype_node("traffic_cone", name="cone")
            pts = body.get(
                "control_points_xy",
                body.get("control_points", [[-4.0, -3.0], [-1.5, 2.0], [1.5, -2.0], [4.0, 3.0]]),
            )
            count = int(body.get("count", 8))
            align_tan = bool(body.get("align_to_tangent", body.get("align_tangent", True)))
            clones = generate_spline_clones(src, pts, count=count, align_to_tangent=align_tan)
            if clones:
                cmd = AddMultipleNodesCommand(clones, label=f"Spline Cloner ({len(clones)} clones)")
                SESSION.manager.execute(cmd)
                created_node_id = cmd.nodes_list[-1]["id"]

        elif cmd_type in ("generate_scatter_brush", "scatter_brush"):
            node_id = str(body.get("node_id", ""))
            src = SESSION.manager.doc.get_node(node_id) if node_id else None
            if src is None:
                arch = str(body.get("archetype", "cardboard_box"))
                src = create_archetype_node(arch, name=f"scatter_{arch}")
            if "center_xy" in body:
                center_xy = tuple(body.get("center_xy", [0.0, 0.0])[:2])
            else:
                center_xy = (float(body.get("center_x", 0.0)), float(body.get("center_y", 0.0)))
            radius = float(body.get("radius", 5.0))
            count = int(body.get("count", 10))
            min_dist = float(body.get("min_distance", body.get("min_clearance", 0.9)))
            scale_min = float(body.get("scale_min", 0.8))
            scale_max = float(body.get("scale_max", 1.25))
            seed = body.get("seed")
            clones = generate_scatter_brush(
                src,
                center_xy=(float(center_xy[0]), float(center_xy[1])),
                radius=radius,
                count=count,
                min_distance=min_dist,
                scale_min=scale_min,
                scale_max=scale_max,
                random_yaw=bool(body.get("random_yaw", True)),
                seed=int(seed) if seed is not None else None,
            )
            if clones:
                cmd = AddMultipleNodesCommand(clones, label=f"Scatter Brush ({len(clones)} items)")
                SESSION.manager.execute(cmd)
                created_node_id = cmd.nodes_list[-1]["id"]

        elif cmd_type == "generate_terrain":
            profile = str(body.get("profile", body.get("preset", "hills")))
            sx = float(body.get("size_x", body.get("width", 12.0)))
            sy = float(body.get("size_y", body.get("depth", 12.0)))
            mh = float(body.get("max_height", body.get("amplitude", 0.80)))
            res = int(body.get("grid_res", 7))
            tnode = create_parametric_terrain_node(
                name=f"terreno_{profile}",
                size_x=sx,
                size_y=sy,
                max_height=mh,
                grid_res=res,
                profile=profile,
                heightmaps_dir=ASSETS_DIR / "heightmaps",
            )
            cmd = AddNodeCommand(tnode)
            SESSION.manager.execute(cmd)
            created_node_id = cmd.node_data["id"]

        elif cmd_type == "add_waypoint":
            existing_wps = [n for n in SESSION.manager.doc.nodes if n.get("type") == "waypoint"]
            idx = len(existing_wps) + 1
            pose = body.get("pose", [round((idx - 1) * 1.5, 2), 0.0, 0.02, 0.0, 0.0, 0.0])
            wp_node = create_waypoint_node(
                name=str(body.get("name", f"wp_{idx}")),
                pose=pose,
                dwell_time_s=float(body.get("dwell_time_s", 0.0)),
            )
            cmd = AddNodeCommand(wp_node)
            SESSION.manager.execute(cmd)
            created_node_id = cmd.node_data["id"]

        elif cmd_type == "add_robot_spawn":
            existing_spawns = [n for n in SESSION.manager.doc.nodes if n.get("type") == "robot_spawn"]
            idx = len(existing_spawns) + 1
            robot_model = str(body.get("robot_model", "turtlebot4"))
            pose = body.get("pose", [-1.5, -1.5 + (idx - 1) * 1.2, 0.05, 0.0, 0.0, 0.0])
            sp_node = create_robot_spawn_node(
                name=str(body.get("name", f"spawn_{robot_model}_{idx}")),
                robot_model=robot_model,
                robot_namespace=str(body.get("robot_namespace", f"robot{idx}")),
                pose=pose,
            )
            cmd = AddNodeCommand(sp_node)
            SESSION.manager.execute(cmd)
            created_node_id = cmd.node_data["id"]

        elif cmd_type == "add_sensor":
            sensor_type = str(body.get("sensor_type", "camera_rgbd"))
            existing_sensors = [n for n in SESSION.manager.doc.nodes if n.get("type") == "sensor"]
            idx = len(existing_sensors) + 1
            s_node = create_static_sensor_node(
                name=str(body.get("name", f"{sensor_type}_{idx}")),
                sensor_type=sensor_type,
                pose=body.get("pose"),
                fov_deg=float(body.get("fov_deg", 75.0)),
                range_max=float(body.get("range_max", 8.0)),
            )
            cmd = AddNodeCommand(s_node)
            SESSION.manager.execute(cmd)
            created_node_id = cmd.node_data["id"]

        elif cmd_type == "add_actor":
            existing_actors = [n for n in SESSION.manager.doc.nodes if n.get("type") == "actor"]
            idx = len(existing_actors) + 1
            a_node = create_dynamic_actor_node(
                name=str(body.get("name", f"actor_peaton_{idx}")),
                pose=body.get("pose"),
                waypoints=body.get("waypoints"),
                speed_mps=float(body.get("speed_mps", 1.1)),
                loop=bool(body.get("loop", True)),
            )
            cmd = AddNodeCommand(a_node)
            SESSION.manager.execute(cmd)
            created_node_id = cmd.node_data["id"]

        elif cmd_type == "update_world":
            patch = dict(body.get("patch", {}))
            SESSION.manager.execute(UpdateWorldSettingsCommand(patch))

        elif cmd_type == "add_folder":
            folder_name = str(body.get("name", "New Folder")).strip() or "New Folder"
            folder_id = f"folder_{len(SESSION.manager.doc.folders) + 1}"
            SESSION.manager.doc.folders.append(
                {"id": folder_id, "name": folder_name, "visible": True, "locked": False}
            )
        else:
            self._send_json({"ok": False, "error": f"Unknown command type '{cmd_type}'"}, HTTPStatus.BAD_REQUEST)
            return

        payload = SESSION.build_full_Payload()
        if created_node_id:
            payload["selected_node_id"] = created_node_id
        self._send_json(payload)

    def _handle_upload_custom_mesh(self, body: Dict[str, Any]) -> None:
        raw_filename = Path(str(body.get("filename", "custom_mesh.stl"))).name
        safe_filename = "".join(c if (c.isalnum() or c in "._-") else "_" for c in raw_filename)
        if not safe_filename.lower().endswith((".stl", ".obj", ".dae", ".glb", ".gltf")):
            safe_filename += ".stl"

        b64_data = str(body.get("content_base64", ""))
        raw_bytes = base64.b64decode(b64_data)
        mesh_path = MESHES_DIR / safe_filename
        mesh_path.write_bytes(raw_bytes)

        unit = str(body.get("unit", "m"))
        up_axis = str(body.get("up_axis", "Z_UP"))
        material_preset = str(body.get("material_preset", "steel"))
        label = str(body.get("label", f"🧩 {Path(safe_filename).stem} ({Path(safe_filename).suffix.upper()})"))

        analysis = analyze_mesh_file(mesh_path, unit=unit, up_axis=up_axis, material_preset=material_preset)
        proxies = generate_collision_proxy_meshes(mesh_path, unit=unit, up_axis=up_axis)

        entry_id = f"custom_{uuid.uuid4().hex[:6]}"
        entry = {
            "id": entry_id,
            "label": label,
            "kind": "cad_mesh",
            "filename": safe_filename,
            "unit": unit,
            "up_axis": up_axis,
            "material_preset": material_preset,
            "color": [0.25, 0.62, 0.95, 1.0],
            "analysis": analysis,
            "proxies": proxies,
        }
        SESSION.custom_objects.append(entry)
        SESSION._save_custom_objects_list(SESSION.custom_objects)

        node = SESSION.build_node_from_custom_entry(entry)
        cmd = AddNodeCommand(node)
        SESSION.manager.execute(cmd)

        payload = SESSION.build_full_Payload()
        payload["selected_node_id"] = cmd.node_data["id"]
        self._send_json(payload)

    def _handle_save_custom_object(self, body: Dict[str, Any]) -> None:
        node_id = str(body.get("node_id", ""))
        node = SESSION.manager.doc.get_node(node_id)
        if node is None:
            self._send_json({"ok": False, "error": "Selecciona un objeto primero."}, HTTPStatus.BAD_REQUEST)
            return

        custom_label = str(body.get("label", "")).strip() or f"⭐ {node.get('name', 'Prefab')}"
        template = copy.deepcopy(node)
        entry_id = f"prefab_{uuid.uuid4().hex[:6]}"
        entry = {
            "id": entry_id,
            "label": custom_label,
            "kind": "saved_prefab",
            "node_template": template,
        }
        SESSION.custom_objects.append(entry)
        SESSION._save_custom_objects_list(SESSION.custom_objects)
        self._send_json(SESSION.build_full_Payload())

    def _handle_delete_custom_object(self, body: Dict[str, Any]) -> None:
        cid = str(body.get("custom_id", ""))
        SESSION.custom_objects = [c for c in SESSION.custom_objects if c.get("id") != cid]
        SESSION._save_custom_objects_list(SESSION.custom_objects)
        self._send_json(SESSION.build_full_Payload())

    def _handle_import_sdf(self, body: Dict[str, Any]) -> None:
        xml_content = str(body.get("xml_content", ""))
        if not xml_content.strip():
            self._send_json({"ok": False, "error": "Contenido SDF vacío."}, HTTPStatus.BAD_REQUEST)
            return
        doc = parse_sdf_to_scene_graph(xml_content)
        SESSION.manager = CommandManager(doc)
        self._send_json(SESSION.build_full_Payload())

    def _handle_import_pgm_walls(self, body: Dict[str, Any]) -> None:
        b64_data = str(body.get("content_base64", body.get("pgm_base64", "")))
        if not b64_data:
            self._send_json({"ok": False, "error": "Archivo PGM vacío."}, HTTPStatus.BAD_REQUEST)
            return
        raw_bytes = base64.b64decode(b64_data)
        resolution = float(body.get("resolution", 0.05))
        wall_height = float(body.get("wall_height", 2.50))
        walls = reconstruct_3d_walls_from_pgm(
            raw_bytes, resolution=resolution, wall_height=wall_height
        )
        if walls:
            SESSION.manager.execute(
                AddMultipleNodesCommand(walls, label=f"Import PGM Map ({len(walls)} 3D Walls)")
            )
        payload = SESSION.build_full_Payload()
        payload["walls_created"] = len(walls)
        self._send_json(payload)

    def _handle_generate_nav2_map(self, body: Dict[str, Any]) -> None:
        nav_cfg = SESSION.manager.doc.robotics.get("nav2_map_config", {})
        z_min = float(body.get("z_min", nav_cfg.get("z_min", 0.15)))
        z_max = float(body.get("z_max", nav_cfg.get("z_max", 0.60)))
        resolution = float(body.get("resolution", nav_cfg.get("resolution", 0.05)))
        padding_m = float(body.get("padding_m", nav_cfg.get("padding_m", 2.0)))

        SESSION.manager.doc.robotics["nav2_map_config"] = {
            "z_min": z_min,
            "z_max": z_max,
            "resolution": resolution,
            "padding_m": padding_m,
        }

        world_name = SESSION.manager.doc.world.get("name", "custebo_world")
        pgm_filename = f"{world_name}_map.pgm"
        yaml_filename = f"{world_name}_map.yaml"

        result = generate_nav2_occupancy_map(
            nodes=SESSION.manager.doc.nodes,
            z_min=z_min,
            z_max=z_max,
            resolution=resolution,
            padding_m=padding_m,
            map_filename=pgm_filename,
        )

        pgm_path = EXPORTS_DIR / pgm_filename
        yaml_path = EXPORTS_DIR / yaml_filename
        pgm_path.write_bytes(result["pgm_bytes"])
        yaml_path.write_text(result["yaml_content"], encoding="utf-8")

        self._send_json(
            {
                "ok": True,
                "width": result["width"],
                "height": result["height"],
                "resolution": result["resolution"],
                "origin": result["origin"],
                "z_min": result["z_min"],
                "z_max": result["z_max"],
                "obstacle_count": result["obstacle_count"],
                "pgm_filename": pgm_filename,
                "yaml_filename": yaml_filename,
                "pgm_path": str(pgm_path),
                "yaml_path": str(yaml_path),
                "pgm_base64": result["pgm_base64"],
                "yaml_content": result["yaml_content"],
            }
        )

    def _handle_export_nav2_waypoints(self) -> None:
        yaml_str = export_nav2_waypoints_yaml(SESSION.manager.doc)
        world_name = SESSION.manager.doc.world.get("name", "custebo_world")
        out_path = EXPORTS_DIR / f"{world_name}_nav2_waypoints.yaml"
        out_path.write_text(yaml_str, encoding="utf-8")
        self._send_json(
            {
                "ok": True,
                "exported_path": str(out_path),
                "filename": out_path.name,
                "yaml_content": yaml_str,
            }
        )

    def _handle_export_ros_gz_bridge(self) -> None:
        yaml_str = generate_ros_gz_bridge_yaml(SESSION.manager.doc)
        world_name = SESSION.manager.doc.world.get("name", "custebo_world")
        out_path = EXPORTS_DIR / f"{world_name}_ros_gz_bridge.yaml"
        out_path.write_text(yaml_str, encoding="utf-8")
        self._send_json(
            {
                "ok": True,
                "exported_path": str(out_path),
                "filename": out_path.name,
                "yaml_content": yaml_str,
            }
        )

    def _handle_export_xacro(self, body: Dict[str, Any]) -> None:
        target = str(body.get("target", SESSION.manager.doc.world.get("sdf_target", "harmonic")))
        xacro_xml = compile_scene_to_xacro(SESSION.manager.doc, target=target)
        world_name = SESSION.manager.doc.world.get("name", "custebo_world")
        out_path = EXPORTS_DIR / f"{world_name}.sdf.xacro"
        out_path.write_text(xacro_xml, encoding="utf-8")
        self._send_json(
            {
                "ok": True,
                "exported_path": str(out_path),
                "filename": out_path.name,
                "xacro_xml": xacro_xml,
            }
        )

    def _handle_save_project(self, body: Dict[str, Any]) -> None:
        filename = str(body.get("filename", "")).strip()
        if not filename:
            world_name = SESSION.manager.doc.world.get("name", "custebo_world")
            filename = f"{world_name}.gproj.json"
        if not filename.endswith(".json"):
            filename += ".gproj.json"
        safe_name = Path(filename).name
        target_path = PROJECTS_DIR / safe_name
        target_path.write_text(SESSION.manager.doc.to_json(indent=2), encoding="utf-8")
        payload = SESSION.build_full_Payload()
        payload["saved_path"] = str(target_path)
        self._send_json(payload)

    def _handle_load_project(self, body: Dict[str, Any]) -> None:
        if "scene_graph" in body and isinstance(body["scene_graph"], dict):
            doc = SceneGraphDocument.from_dict(body["scene_graph"])
            SESSION.manager = CommandManager(doc)
            self._send_json(SESSION.build_full_Payload())
            return

        filename = str(body.get("filename", "")).strip()
        target_path = PROJECTS_DIR / Path(filename).name
        if not target_path.is_file():
            self._send_json({"ok": False, "error": f"Project file not found: {target_path}"}, HTTPStatus.NOT_FOUND)
            return
        doc = SceneGraphDocument.from_json(target_path.read_text(encoding="utf-8"))
        SESSION.manager = CommandManager(doc)
        self._send_json(SESSION.build_full_Payload())

    def _handle_export_sdf(self, body: Dict[str, Any]) -> None:
        target = body.get("target")
        sdf_xml, line_map = SDFCompiler.compile(SESSION.manager.doc, target_override=target)
        world_name = SESSION.manager.doc.world.get("name", "custebo_world")
        filename = str(body.get("filename", f"{world_name}.sdf")).strip()
        if not filename.endswith(".sdf"):
            filename += ".sdf"
        out_path = EXPORTS_DIR / Path(filename).name
        out_path.write_text(sdf_xml, encoding="utf-8")

        report = validate_sdf_content(sdf_xml, str(out_path))
        self._send_json(
            {
                "ok": True,
                "exported_path": str(out_path),
                "sdf_xml": sdf_xml,
                "sdf_line_map": line_map,
                "validation": {
                    "is_valid": report.is_valid,
                    "gz_cli_checked": report.gz_cli_checked,
                    "issues": [
                        {
                            "severity": i.severity,
                            "code": i.code,
                            "message": i.message,
                            "location": i.location,
                        }
                        for i in report.issues
                    ],
                },
            }
        )

    def _handle_validate_sdf(self, body: Dict[str, Any]) -> None:
        sdf_xml, _ = SDFCompiler.compile(SESSION.manager.doc)
        report = validate_sdf_content(sdf_xml, "live_world.sdf")
        self._send_json(
            {
                "ok": True,
                "is_valid": report.is_valid,
                "gz_cli_checked": report.gz_cli_checked,
                "issues": [
                    {
                        "severity": i.severity,
                        "code": i.code,
                        "message": i.message,
                        "location": i.location,
                    }
                    for i in report.issues
                ],
            }
        )

    def _handle_play_gazebo(self) -> None:
        gz_bin = find_gz_binary()
        if not gz_bin:
            self._send_json({"ok": False, "error": "Binario `gz` no encontrado en el sistema."}, HTTPStatus.NOT_FOUND)
            return
        sdf_xml, _ = SDFCompiler.compile(SESSION.manager.doc, target_override="harmonic")
        temp_sdf = EXPORTS_DIR / "custebo_live_preview.sdf"
        temp_sdf.write_text(sdf_xml, encoding="utf-8")
        subprocess.Popen(
            [gz_bin, "sim", str(temp_sdf)],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            start_new_session=True,
        )
        self._send_json({"ok": True, "launched_file": str(temp_sdf)})


def run_server(host: str = "127.0.0.1", port: int = 8080) -> None:
    mimetypes.add_type("application/javascript", ".js")
    mimetypes.add_type("text/css", ".css")
    server = ThreadingHTTPServer((host, port), CusteboRequestHandler)
    print(f"======================================================================")
    print(f"  Custebo 3D World Editor (Phase 1 & 2) running at:")
    print(f"  -> http://{host}:{port}")
    print(f"======================================================================")
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("\nShutting down Custebo server.")
    finally:
        server.server_close()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Start the Custebo 3D World Editor server.")
    parser.add_argument("--host", default="127.0.0.1", help="Bind host (default: 127.0.0.1)")
    parser.add_argument("--port", type=int, default=8080, help="Bind port (default: 8080)")
    args = parser.parse_args()
    run_server(host=args.host, port=args.port)
