"""
Scene Graph Data Schema (`core/schema/scene_graph.py`).

Pure JSON-serializable data structures for Custebo.
Designed in Phase 1 to natively support Phases 1 through 5 without breaking migrations:
  - Phase 1: Primitives, Lights, Ground Plane, Outliner Hierarchy, Folders/Layers, Clean SDF
  - Phase 2: Local CAD Meshes, Auto-Inertia, Collision Strategies, Fuel/ROS 2 URIs
  - Phase 3: Procedural Generators (Floorplan 2D->3D, Linear/Grid Arrays, Splines, Scatter)
  - Phase 4: Nav2 2D Map Slice Config, Waypoint Graphs, Robot Spawn Poses, Sensors, Actors, XACRO
  - Phase 5: ROS 2 Package Export Settings & User Prefabs
"""

from __future__ import annotations

import copy
import json
import uuid
from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional
from core.math_physics.inertia import compute_auto_inertial
from core.math_physics.transforms import clean_float


SCHEMA_VERSION = "1.0.0"


def generate_node_id(prefix: str = "node") -> str:
    return f"{prefix}_{uuid.uuid4().hex[:8]}"


@dataclass
class SceneGraphDocument:
    """
    Root Scene Graph container holding pure state (zero 3D rendering references).
    Can be serialized to/from `.gproj` / `.json` and compiled directly to `.sdf`.
    """

    schema_version: str = SCHEMA_VERSION
    world: Dict[str, Any] = field(default_factory=dict)
    folders: List[Dict[str, Any]] = field(default_factory=list)
    nodes: List[Dict[str, Any]] = field(default_factory=list)
    reference_planes: List[Dict[str, Any]] = field(default_factory=list)
    robotics: Dict[str, Any] = field(default_factory=dict)
    package_export: Dict[str, Any] = field(default_factory=dict)
    metadata: Dict[str, Any] = field(default_factory=dict)

    @classmethod
    def create_default(cls, world_name: str = "custebo_world") -> "SceneGraphDocument":
        """Create a default clean world with sun light, ground plane, and sample objects."""
        doc = cls(
            schema_version=SCHEMA_VERSION,
            world={
                "name": world_name,
                "sdf_target": "harmonic",  # "harmonic" (SDF 1.10) or "classic" (SDF 1.7)
                "sdf_version": "1.10",
                "gravity": [0.0, 0.0, -9.81],
                "physics": {
                    "name": "1ms",
                    "engine": "dart",
                    "max_step_size": 0.001,
                    "real_time_factor": 1.0,
                },
                "ambient": [0.4, 0.4, 0.4, 1.0],
                "background": [0.7, 0.75, 0.82, 1.0],
                "plugins": {
                    "physics": True,
                    "user_commands": True,
                    "scene_broadcaster": True,
                    "sensors": True,
                    "imu": False,
                    "magnetometer": False,
                    "air_pressure": False,
                    "navsat": False,
                },
            },
            folders=[
                {"id": "folder_env", "name": "Environment & Lights", "visible": True, "locked": False},
                {"id": "folder_arch", "name": "Static Architecture", "visible": True, "locked": False},
                {"id": "folder_props", "name": "Models & Props", "visible": True, "locked": False},
            ],
            nodes=[
                create_light_node(
                    name="sun",
                    light_type="directional",
                    pose=[0.0, 0.0, 10.0, 0.0, 0.0, 0.0],
                    direction=[-0.5, 0.1, -0.9],
                    diffuse=[0.85, 0.85, 0.82, 1.0],
                    specular=[0.25, 0.25, 0.25, 1.0],
                    cast_shadows=True,
                    folder_id="folder_env",
                    node_id="node_sun",
                ),
                create_ground_plane_node(
                    name="ground_plane",
                    size=[100.0, 100.0],
                    color=[0.42, 0.44, 0.47, 1.0],
                    folder_id="folder_env",
                    node_id="node_ground_plane",
                    locked=True,
                ),
            ],
            reference_planes=[],
            robotics={
                "nav2_map_config": {
                    "z_min": 0.15,
                    "z_max": 0.60,
                    "resolution": 0.05,
                    "padding_m": 2.0,
                },
                "spawn_poses": [],
                "waypoint_graphs": [],
            },
            package_export={
                "package_name": "custebo_robot_worlds",
                "maintainer": "Eugegeuge",
                "maintainer_email": "hugosema.19@gmail.com",
                "license": "Apache-2.0",
            },
            metadata={
                "generator": "Custebo 3D World Editor",
                "units_distance": "meters",
                "units_angle": "radians",
                "coordinate_frame": "ENU_Z_UP",
            },
        )
        return doc

    def to_dict(self) -> Dict[str, Any]:
        return copy.deepcopy(
            {
                "schema_version": self.schema_version,
                "world": self.world,
                "folders": self.folders,
                "nodes": self.nodes,
                "reference_planes": self.reference_planes,
                "robotics": self.robotics,
                "package_export": self.package_export,
                "metadata": self.metadata,
            }
        )

    def to_json(self, indent: int = 2) -> str:
        return json.dumps(self.to_dict(), indent=indent, ensure_ascii=False)

    @classmethod
    def from_dict(cls, data: Dict[str, Any]) -> "SceneGraphDocument":
        default = cls.create_default()
        world = copy.deepcopy(default.world)
        if isinstance(data.get("world"), dict):
            world.update(copy.deepcopy(data["world"]))

        doc = cls(
            schema_version=str(data.get("schema_version", SCHEMA_VERSION)),
            world=world,
            folders=copy.deepcopy(data.get("folders", default.folders)),
            nodes=copy.deepcopy(data.get("nodes", [])),
            reference_planes=copy.deepcopy(data.get("reference_planes", [])),
            robotics=copy.deepcopy(data.get("robotics", default.robotics)),
            package_export=copy.deepcopy(data.get("package_export", default.package_export)),
            metadata=copy.deepcopy(data.get("metadata", default.metadata)),
        )
        for node in doc.nodes:
            normalize_node_in_place(node)
        return doc

    @classmethod
    def from_json(cls, json_str: str) -> "SceneGraphDocument":
        return cls.from_dict(json.loads(json_str))

    def get_node(self, node_id: str) -> Optional[Dict[str, Any]]:
        for node in self.nodes:
            if node.get("id") == node_id:
                return node
        return None

    def ensure_unique_name(self, base_name: str, exclude_id: Optional[str] = None) -> str:
        sanitized = "".join(c if (c.isalnum() or c in "_-") else "_" for c in base_name.strip())
        if not sanitized:
            sanitized = "object"
        existing = {
            n.get("name") for n in self.nodes if n.get("id") != exclude_id
        }
        if sanitized not in existing:
            return sanitized
        counter = 1
        while f"{sanitized}_{counter}" in existing:
            counter += 1
        return f"{sanitized}_{counter}"


def normalize_node_in_place(node: Dict[str, Any]) -> Dict[str, Any]:
    """Ensure a node dictionary conforms to ENU Z-Up, finite floats, and valid inertial values."""
    if not node.get("id"):
        node["id"] = generate_node_id(node.get("type", "node"))
    if not node.get("name"):
        node["name"] = node["id"]
    node.setdefault("type", "primitive")
    node.setdefault("parent_id", None)
    node.setdefault("folder_id", "folder_props")
    node.setdefault("visible", True)
    node.setdefault("locked", False)
    node.setdefault("static", node["type"] in ("ground_plane", "light"))

    pose = node.get("pose", [0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    if not isinstance(pose, list) or len(pose) < 6:
        pose = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    node["pose"] = [clean_float(float(v)) for v in pose[:6]]

    scale = node.get("scale", [1.0, 1.0, 1.0])
    if not isinstance(scale, list) or len(scale) < 3:
        scale = [1.0, 1.0, 1.0]
    node["scale"] = [max(clean_float(float(v)), 0.001) for v in scale[:3]]

    if node["type"] in ("primitive", "ground_plane", "mesh", "include"):
        geom = node.setdefault("geometry", {})
        if node["type"] == "ground_plane":
            geom.setdefault("shape", "plane")
            geom.setdefault("size", [100.0, 100.0])
            geom.setdefault("normal", [0.0, 0.0, 1.0])
        elif node["type"] in ("mesh", "include"):
            geom.setdefault("shape", "box")
            geom.setdefault("size", [1.0, 1.0, 1.0])
            geom.setdefault("radius", 0.5)
            geom.setdefault("length", 1.0)
        else:
            shape = geom.setdefault("shape", "box")
            if shape == "box":
                geom.setdefault("size", [1.0, 1.0, 1.0])
            elif shape == "cylinder":
                geom.setdefault("radius", 0.5)
                geom.setdefault("length", 1.0)
                geom.setdefault("size", [1.0, 1.0, 1.0])
            elif shape == "sphere":
                geom.setdefault("radius", 0.5)
                geom.setdefault("size", [1.0, 1.0, 1.0])
            elif shape == "plane":
                geom.setdefault("size", [10.0, 10.0])

        mat = node.setdefault("material", {})
        mat.setdefault("color", [0.25, 0.52, 0.85, 1.0])
        mat.setdefault("roughness", 0.5)
        mat.setdefault("metalness", 0.1)

        col = node.setdefault("collision", {})
        col.setdefault("enabled", True)
        col.setdefault("strategy", "aabb" if node["type"] == "mesh" else "primitive")
        col.setdefault("surface_preset", "warehouse_floor")
        col.setdefault("mu1", 0.7)
        col.setdefault("mu2", 0.7)

        inertial = node.setdefault("inertial", {})
        inertial.setdefault("mode", "auto")
        inertial.setdefault("material_preset", "steel" if node["type"] == "mesh" else "wood")
        inertial.setdefault("center_of_mass", [0.0, 0.0, 0.0])

        if inertial.get("mode", "auto") == "auto" or float(inertial.get("mass", 0.0)) <= 0.0:
            recalculate_node_inertial(node)

    return node


def recalculate_node_inertial(node: Dict[str, Any]) -> None:
    """Recalculate mass and inertia tensor for a primitive or mesh node."""
    geom = node.get("geometry", {})
    shape = geom.get("shape", "box")
    size = geom.get("size", [1.0, 1.0, 1.0])
    if len(size) == 2:
        size = [size[0], size[1], 0.01]
    radius = float(geom.get("radius", 0.5))
    length = float(geom.get("length", 1.0))
    scale = node.get("scale", [1.0, 1.0, 1.0])

    inertial = node.setdefault("inertial", {})
    mode = inertial.get("mode", "auto")
    preset = inertial.get("material_preset", "wood")
    com = tuple(inertial.get("center_of_mass", [0.0, 0.0, 0.0])[:3])

    manual_mass = float(inertial.get("mass", 0.0)) if mode == "manual_mass" else None
    computed = compute_auto_inertial(
        shape=shape,
        size=size,
        radius=radius,
        length=length,
        scale=scale,
        mass=manual_mass,
        material_preset=preset,
        com_offset=(float(com[0]), float(com[1]), float(com[2])),
    )
    inertial.update(computed)


def create_primitive_node(
    name: str = "box",
    shape: str = "box",
    pose: Optional[List[float]] = None,
    size: Optional[List[float]] = None,
    radius: float = 0.5,
    length: float = 1.0,
    color: Optional[List[float]] = None,
    static: bool = False,
    material_preset: str = "wood",
    folder_id: str = "folder_props",
    node_id: Optional[str] = None,
) -> Dict[str, Any]:
    """Factory for creating a normalized 3D primitive node (box, cylinder, sphere)."""
    if size is None:
        size = [1.0, 1.0, 1.0]
    if pose is None:
        if shape == "box":
            z0 = float(size[2]) * 0.5
        elif shape == "cylinder":
            z0 = float(length) * 0.5
        elif shape == "sphere":
            z0 = float(radius)
        else:
            z0 = 0.5
        pose = [0.0, 0.0, z0, 0.0, 0.0, 0.0]

    if color is None:
        palette = {
            "box": [0.23, 0.51, 0.96, 1.0],
            "cylinder": [0.06, 0.72, 0.50, 1.0],
            "sphere": [0.96, 0.62, 0.04, 1.0],
        }
        color = palette.get(shape, [0.6, 0.65, 0.7, 1.0])

    node: Dict[str, Any] = {
        "id": node_id or generate_node_id(shape),
        "name": name,
        "type": "primitive",
        "parent_id": None,
        "folder_id": folder_id,
        "visible": True,
        "locked": False,
        "static": bool(static),
        "pose": [clean_float(float(v)) for v in pose[:6]],
        "scale": [1.0, 1.0, 1.0],
        "geometry": {
            "shape": shape,
            "size": [clean_float(float(v)) for v in size[:3]],
            "radius": clean_float(float(radius)),
            "length": clean_float(float(length)),
        },
        "material": {
            "color": [clean_float(float(c), 4) for c in color[:4]],
            "roughness": 0.45,
            "metalness": 0.1,
        },
        "collision": {
            "enabled": True,
            "strategy": "primitive",
            "surface_preset": "warehouse_floor",
            "mu1": 0.7,
            "mu2": 0.7,
        },
        "inertial": {
            "mode": "auto",
            "material_preset": material_preset,
            "center_of_mass": [0.0, 0.0, 0.0],
        },
    }
    return normalize_node_in_place(node)


def create_ground_plane_node(
    name: str = "ground_plane",
    size: Optional[List[float]] = None,
    color: Optional[List[float]] = None,
    folder_id: str = "folder_env",
    node_id: Optional[str] = None,
    locked: bool = True,
) -> Dict[str, Any]:
    if size is None:
        size = [100.0, 100.0]
    if color is None:
        color = [0.42, 0.44, 0.47, 1.0]
    node: Dict[str, Any] = {
        "id": node_id or generate_node_id("ground"),
        "name": name,
        "type": "ground_plane",
        "parent_id": None,
        "folder_id": folder_id,
        "visible": True,
        "locked": locked,
        "static": True,
        "pose": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        "scale": [1.0, 1.0, 1.0],
        "geometry": {
            "shape": "plane",
            "size": [clean_float(float(size[0])), clean_float(float(size[1]))],
            "normal": [0.0, 0.0, 1.0],
        },
        "material": {
            "color": [clean_float(float(c), 4) for c in color[:4]],
            "roughness": 0.85,
            "metalness": 0.05,
        },
        "collision": {
            "enabled": True,
            "strategy": "primitive",
            "surface_preset": "warehouse_floor",
            "mu1": 0.8,
            "mu2": 0.8,
        },
    }
    return normalize_node_in_place(node)


def create_light_node(
    name: str = "light",
    light_type: str = "point",
    pose: Optional[List[float]] = None,
    direction: Optional[List[float]] = None,
    diffuse: Optional[List[float]] = None,
    specular: Optional[List[float]] = None,
    intensity: float = 1.0,
    range_m: float = 20.0,
    cast_shadows: bool = True,
    spot_inner_angle: float = 0.6,
    spot_outer_angle: float = 1.0,
    folder_id: str = "folder_env",
    node_id: Optional[str] = None,
) -> Dict[str, Any]:
    if pose is None:
        pose = [0.0, 0.0, 4.0, 0.0, 0.0, 0.0]
    if direction is None:
        direction = [-0.5, 0.1, -0.9] if light_type == "directional" else [0.0, 0.0, -1.0]
    if diffuse is None:
        diffuse = [0.95, 0.95, 0.90, 1.0]
    if specular is None:
        specular = [0.3, 0.3, 0.3, 1.0]

    node: Dict[str, Any] = {
        "id": node_id or generate_node_id("light"),
        "name": name,
        "type": "light",
        "parent_id": None,
        "folder_id": folder_id,
        "visible": True,
        "locked": False,
        "static": True,
        "pose": [clean_float(float(v)) for v in pose[:6]],
        "scale": [1.0, 1.0, 1.0],
        "light": {
            "light_type": light_type,  # "directional" | "point" | "spot"
            "direction": [clean_float(float(v)) for v in direction[:3]],
            "diffuse": [clean_float(float(c), 4) for c in diffuse[:4]],
            "specular": [clean_float(float(c), 4) for c in specular[:4]],
            "intensity": clean_float(float(intensity)),
            "range": clean_float(float(range_m)),
            "cast_shadows": bool(cast_shadows),
            "attenuation": {
                "constant": 0.9,
                "linear": 0.01,
                "quadratic": 0.001,
            },
            "spot": {
                "inner_angle": clean_float(float(spot_inner_angle)),
                "outer_angle": clean_float(float(spot_outer_angle)),
                "falloff": 1.0,
            },
        },
    }
    return normalize_node_in_place(node)

