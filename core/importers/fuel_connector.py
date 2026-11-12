"""
Gazebo Fuel Cloud Connector & ROS 2 Workspace Scanner (`core/importers/fuel_connector.py`).

Provides:
  1. Curated & Live Search against the Open Robotics Gazebo Fuel API (`https://fuel.gazebosim.org/1.0/models`).
  2. Automatic scanner for local ROS 2 workspaces (`AMENT_PREFIX_PATH`, `GZ_SIM_RESOURCE_PATH`, `/opt/ros/jazzy/share`)
     discovering `model.config`, `model.sdf`, and mesh assets with `model://` and `package://` URIs.
  3. Rich Multi-Part 3D Geometry Resolver (`resolve_fuel_or_ros_model_visuals`):
     - Parses `<visual>` elements from local or remote `model.sdf` files when available.
     - Synthesizes detailed, multi-part 3D compound representations (trees, humans, vehicles,
       forklifts, mobile robots, dumpsters, chairs, streetlights, fire hydrants, barrels, stop signs,
       shelves, tables, pallets, cones, barriers) so imported Fuel / ROS 2 models NEVER appear as plain cubes!
"""

from __future__ import annotations

import json
import math
import os
import shutil
import urllib.parse
import urllib.request
import xml.etree.ElementTree as ET
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

from core.schema.presets_library import ARCHETYPE_CATALOG, create_archetype_node


CURATED_FUEL_MODELS: List[Dict[str, Any]] = [
    {
        "name": "🗄 Estantería Almacén (Warehouse Rack)",
        "model_name": "aws_robomaker_warehouse_ShelfE_01",
        "owner": "OpenRobotics",
        "uri": "https://fuel.gazebosim.org/1.0/OpenRobotics/models/aws_robomaker_warehouse_ShelfE_01",
        "category": "Warehouse & Logistics",
        "size": [2.4, 1.0, 2.2],
        "color": [0.20, 0.45, 0.85, 1.0],
        "static": True,
        "proxy_archetype": "warehouse_rack",
    },
    {
        "name": "🚜 Carretilla Elevadora (Forklift)",
        "model_name": "Forklift",
        "owner": "OpenRobotics",
        "uri": "https://fuel.gazebosim.org/1.0/OpenRobotics/models/Forklift",
        "category": "Warehouse & Logistics",
        "size": [2.2, 1.1, 2.0],
        "color": [0.95, 0.68, 0.08, 1.0],
        "static": True,
        "proxy_archetype": "fuel_forklift",
    },
    {
        "name": "🪵 Palé Europeo (Euro Pallet)",
        "model_name": "Euro pallet",
        "owner": "OpenRobotics",
        "uri": "https://fuel.gazebosim.org/1.0/OpenRobotics/models/Euro%20pallet",
        "category": "Warehouse & Logistics",
        "size": [1.2, 0.8, 0.144],
        "color": [0.76, 0.56, 0.32, 1.0],
        "static": False,
        "proxy_archetype": "euro_pallet",
    },
    {
        "name": "🛢 Bidón Industrial (Oil Drum / Barrel)",
        "model_name": "Construction Barrel",
        "owner": "OpenRobotics",
        "uri": "https://fuel.gazebosim.org/1.0/OpenRobotics/models/Construction%20Barrel",
        "category": "Warehouse & Industrial",
        "size": [0.6, 0.6, 0.9],
        "color": [0.92, 0.36, 0.10, 1.0],
        "static": False,
        "proxy_archetype": "fuel_barrel",
    },
    {
        "name": "🔺 Cono de Obra (Construction Cone)",
        "model_name": "Construction Cone",
        "owner": "OpenRobotics",
        "uri": "https://fuel.gazebosim.org/1.0/OpenRobotics/models/Construction%20Cone",
        "category": "Traffic & Safety",
        "size": [0.4, 0.4, 0.7],
        "color": [0.96, 0.38, 0.08, 1.0],
        "static": False,
        "proxy_archetype": "traffic_cone",
    },
    {
        "name": "🚧 Barrera Vial (Jersey Barrier)",
        "model_name": "Jersey Barrier",
        "owner": "OpenRobotics",
        "uri": "https://fuel.gazebosim.org/1.0/OpenRobotics/models/Jersey%20Barrier",
        "category": "Traffic & Safety",
        "size": [2.0, 0.6, 0.85],
        "color": [0.74, 0.76, 0.78, 1.0],
        "static": True,
        "proxy_archetype": "jersey_barrier",
    },
    {
        "name": "🛑 Señal de STOP (Stop Sign)",
        "model_name": "Stop Sign",
        "owner": "OpenRobotics",
        "uri": "https://fuel.gazebosim.org/1.0/OpenRobotics/models/Stop%20Sign",
        "category": "Traffic & Safety",
        "size": [0.5, 0.5, 2.1],
        "color": [0.88, 0.14, 0.14, 1.0],
        "static": True,
        "proxy_archetype": "fuel_stop_sign",
    },
    {
        "name": "🚒 Boca de Incendios (Fire Hydrant)",
        "model_name": "Fire Hydrant",
        "owner": "OpenRobotics",
        "uri": "https://fuel.gazebosim.org/1.0/OpenRobotics/models/Fire%20Hydrant",
        "category": "Outdoor & Safety",
        "size": [0.45, 0.45, 0.85],
        "color": [0.85, 0.16, 0.16, 1.0],
        "static": True,
        "proxy_archetype": "fuel_hydrant",
    },
    {
        "name": "🪑 Mesa de Cafetería / Lab (Cafe Table)",
        "model_name": "Cafe table",
        "owner": "OpenRobotics",
        "uri": "https://fuel.gazebosim.org/1.0/OpenRobotics/models/Cafe%20table",
        "category": "Furniture & Indoor",
        "size": [0.9, 0.9, 0.75],
        "color": [0.68, 0.48, 0.28, 1.0],
        "static": True,
        "proxy_archetype": "table",
    },
    {
        "name": "💺 Silla de Oficina / Lab (Office Chair)",
        "model_name": "OfficeChairBlue",
        "owner": "OpenRobotics",
        "uri": "https://fuel.gazebosim.org/1.0/OpenRobotics/models/OfficeChairBlue",
        "category": "Furniture & Indoor",
        "size": [0.6, 0.6, 0.95],
        "color": [0.18, 0.45, 0.82, 1.0],
        "static": False,
        "proxy_archetype": "fuel_chair",
    },
    {
        "name": "🚶 Persona de Pie (Standing Person)",
        "model_name": "Standing person",
        "owner": "OpenRobotics",
        "uri": "https://fuel.gazebosim.org/1.0/OpenRobotics/models/Standing%20person",
        "category": "Humans & Actors",
        "size": [0.55, 0.38, 1.75],
        "color": [0.25, 0.65, 0.85, 1.0],
        "static": True,
        "proxy_archetype": "fuel_person",
    },
    {
        "name": "🌳 Roble / Árbol Exterior (Oak Tree)",
        "model_name": "Oak tree",
        "owner": "OpenRobotics",
        "uri": "https://fuel.gazebosim.org/1.0/OpenRobotics/models/Oak%20tree",
        "category": "Outdoor & Nature",
        "size": [2.6, 2.6, 4.2],
        "color": [0.18, 0.62, 0.28, 1.0],
        "static": True,
        "proxy_archetype": "fuel_tree",
    },
    {
        "name": "🌲 Pino Exterior (Pine Tree)",
        "model_name": "Pine Tree",
        "owner": "OpenRobotics",
        "uri": "https://fuel.gazebosim.org/1.0/OpenRobotics/models/Pine%20Tree",
        "category": "Outdoor & Nature",
        "size": [2.0, 2.0, 4.5],
        "color": [0.12, 0.50, 0.24, 1.0],
        "static": True,
        "proxy_archetype": "fuel_pine_tree",
    },
    {
        "name": "🗑 Contenedor Industrial (Dumpster)",
        "model_name": "Dumpster",
        "owner": "OpenRobotics",
        "uri": "https://fuel.gazebosim.org/1.0/OpenRobotics/models/Dumpster",
        "category": "Outdoor & Industrial",
        "size": [1.8, 1.1, 1.3],
        "color": [0.16, 0.52, 0.32, 1.0],
        "static": True,
        "proxy_archetype": "fuel_dumpster",
    },
    {
        "name": "🚗 Vehículo SUV / Coche (SUV)",
        "model_name": "SUV",
        "owner": "OpenRobotics",
        "uri": "https://fuel.gazebosim.org/1.0/OpenRobotics/models/SUV",
        "category": "Vehicles",
        "size": [4.3, 1.85, 1.65],
        "color": [0.22, 0.52, 0.88, 1.0],
        "static": True,
        "proxy_archetype": "fuel_car",
    },
    {
        "name": "🏮 Farola Exterior (Lamp Post)",
        "model_name": "Lamp Post",
        "owner": "OpenRobotics",
        "uri": "https://fuel.gazebosim.org/1.0/OpenRobotics/models/Lamp%20Post",
        "category": "Outdoor & Street",
        "size": [0.8, 0.8, 4.2],
        "color": [0.35, 0.38, 0.42, 1.0],
        "static": True,
        "proxy_archetype": "fuel_lamp_post",
    },
]


def build_procedural_fuel_parts(
    archetype_hint: str,
    model_name: str = "",
) -> Tuple[List[Dict[str, Any]], List[float], List[float]]:
    """
    Build a detailed multi-part 3D representation (`(parts, bounding_size, main_color)`)
    for Gazebo Fuel and ROS 2 Workspace models so they are immediately recognizable in the 3D Viewport.
    """
    key = (archetype_hint or "").lower().strip()
    name_low = (model_name or "").lower()

    # 1. If it directly matches a built-in Custebo parametric archetype, reuse its exact parts!
    if key in ARCHETYPE_CATALOG:
        temp = create_archetype_node(key)
        return (
            temp.get("parts", []),
            temp.get("geometry", {}).get("size", [1.0, 1.0, 1.0]),
            temp.get("material", {}).get("color", [0.3, 0.6, 0.9, 1.0]),
        )

    # Infer category from model_name if key wasn't explicit
    if not key:
        if any(w in name_low for w in ("shelf", "rack", "bookcase", "storage")):
            return build_procedural_fuel_parts("warehouse_rack", model_name)
        if "pallet" in name_low:
            return build_procedural_fuel_parts("euro_pallet", model_name)
        if "cone" in name_low:
            return build_procedural_fuel_parts("traffic_cone", model_name)
        if "barrier" in name_low or "jersey" in name_low:
            return build_procedural_fuel_parts("jersey_barrier", model_name)
        if "fence" in name_low or "rail" in name_low:
            return build_procedural_fuel_parts("safety_fence", model_name)
        if "table" in name_low or "desk" in name_low or "bench" in name_low:
            return build_procedural_fuel_parts("table", model_name)
        if "stair" in name_low:
            return build_procedural_fuel_parts("stairs", model_name)
        if "pine" in name_low:
            key = "fuel_pine_tree"
        elif any(w in name_low for w in ("tree", "oak", "bush", "plant", "foliage")):
            key = "fuel_tree"
        elif any(w in name_low for w in ("person", "human", "actor", "pedestrian", "casual", "doctor", "nurse", "worker")):
            key = "fuel_person"
        elif any(w in name_low for w in ("dumpster", "trash", "bin", "garbage", "container")):
            key = "fuel_dumpster"
        elif any(w in name_low for w in ("forklift", "lifter", "trolley", "jack")):
            key = "fuel_forklift"
        elif any(w in name_low for w in ("car", "suv", "prius", "sedan", "hatchback", "truck", "van", "ambulance", "bus")):
            key = "fuel_car"
        elif any(w in name_low for w in ("chair", "seat", "stool", "sofa", "couch")):
            key = "fuel_chair"
        elif any(w in name_low for w in ("lamp", "light_post", "streetlight", "pole")):
            key = "fuel_lamp_post"
        elif any(w in name_low for w in ("hydrant", "extinguisher")):
            key = "fuel_hydrant"
        elif any(w in name_low for w in ("stop", "sign")):
            key = "fuel_stop_sign"
        elif any(w in name_low for w in ("barrel", "drum", "cylinder", "can")):
            key = "fuel_barrel"
        elif any(w in name_low for w in ("robot", "turtlebot", "husky", "agv", "amr", "jackal", "scout", "rover", "diff_drive", "drone", "uav")):
            key = "fuel_robot"
        else:
            key = "fuel_industrial_module"

    # 2. Synthesize rich multi-part 3D geometry by category
    if key == "fuel_tree":
        # Trunk + 3 lush foliage spheres
        parts = [
            {
                "name": "trunk",
                "shape": "cylinder",
                "local_pose": [0.0, 0.0, -1.1, 0.0, 0.0, 0.0],
                "radius": 0.22,
                "length": 2.0,
                "size": [0.44, 0.44, 2.0],
                "color": [0.45, 0.28, 0.14, 1.0],
            },
            {
                "name": "canopy_main",
                "shape": "sphere",
                "local_pose": [0.0, 0.0, 0.65, 0.0, 0.0, 0.0],
                "radius": 1.25,
                "length": 2.5,
                "size": [2.5, 2.5, 2.5],
                "color": [0.18, 0.60, 0.26, 1.0],
            },
            {
                "name": "canopy_left",
                "shape": "sphere",
                "local_pose": [-0.55, 0.45, 0.35, 0.0, 0.0, 0.0],
                "radius": 0.90,
                "length": 1.8,
                "size": [1.8, 1.8, 1.8],
                "color": [0.22, 0.66, 0.30, 1.0],
            },
            {
                "name": "canopy_right",
                "shape": "sphere",
                "local_pose": [0.55, -0.40, 0.40, 0.0, 0.0, 0.0],
                "radius": 0.85,
                "length": 1.7,
                "size": [1.7, 1.7, 1.7],
                "color": [0.15, 0.54, 0.23, 1.0],
            },
        ]
        return parts, [2.6, 2.6, 4.2], [0.18, 0.60, 0.26, 1.0]

    if key == "fuel_pine_tree":
        parts = [
            {
                "name": "trunk",
                "shape": "cylinder",
                "local_pose": [0.0, 0.0, -1.35, 0.0, 0.0, 0.0],
                "radius": 0.18,
                "length": 1.8,
                "size": [0.36, 0.36, 1.8],
                "color": [0.42, 0.26, 0.12, 1.0],
            },
            {
                "name": "tier_bottom",
                "shape": "cylinder",
                "local_pose": [0.0, 0.0, -0.20, 0.0, 0.0, 0.0],
                "radius": 0.95,
                "length": 1.2,
                "size": [1.9, 1.9, 1.2],
                "color": [0.12, 0.46, 0.22, 1.0],
            },
            {
                "name": "tier_mid",
                "shape": "cylinder",
                "local_pose": [0.0, 0.0, 0.75, 0.0, 0.0, 0.0],
                "radius": 0.70,
                "length": 1.1,
                "size": [1.4, 1.4, 1.1],
                "color": [0.15, 0.52, 0.25, 1.0],
            },
            {
                "name": "tier_top",
                "shape": "cylinder",
                "local_pose": [0.0, 0.0, 1.55, 0.0, 0.0, 0.0],
                "radius": 0.40,
                "length": 0.9,
                "size": [0.8, 0.8, 0.9],
                "color": [0.18, 0.58, 0.28, 1.0],
            },
        ]
        return parts, [2.0, 2.0, 4.5], [0.14, 0.50, 0.24, 1.0]

    if key == "fuel_person":
        parts = [
            {
                "name": "leg_left",
                "shape": "cylinder",
                "local_pose": [0.0, 0.10, -0.45, 0.0, 0.0, 0.0],
                "radius": 0.07,
                "length": 0.85,
                "size": [0.14, 0.14, 0.85],
                "color": [0.18, 0.24, 0.35, 1.0],
            },
            {
                "name": "leg_right",
                "shape": "cylinder",
                "local_pose": [0.0, -0.10, -0.45, 0.0, 0.0, 0.0],
                "radius": 0.07,
                "length": 0.85,
                "size": [0.14, 0.14, 0.85],
                "color": [0.18, 0.24, 0.35, 1.0],
            },
            {
                "name": "torso",
                "shape": "box",
                "local_pose": [0.0, 0.0, 0.22, 0.0, 0.0, 0.0],
                "size": [0.26, 0.42, 0.58],
                "radius": 0.2,
                "length": 0.58,
                "color": [0.22, 0.58, 0.86, 1.0],
            },
            {
                "name": "head",
                "shape": "sphere",
                "local_pose": [0.0, 0.0, 0.68, 0.0, 0.0, 0.0],
                "radius": 0.13,
                "length": 0.26,
                "size": [0.26, 0.26, 0.26],
                "color": [0.92, 0.74, 0.60, 1.0],
            },
        ]
        return parts, [0.50, 0.50, 1.75], [0.22, 0.58, 0.86, 1.0]

    if key == "fuel_dumpster":
        parts = [
            {
                "name": "bin_body",
                "shape": "box",
                "local_pose": [0.0, 0.0, -0.10, 0.0, 0.0, 0.0],
                "size": [1.75, 1.05, 1.05],
                "radius": 0.5,
                "length": 1.0,
                "color": [0.16, 0.50, 0.30, 1.0],
            },
            {
                "name": "lid_left",
                "shape": "box",
                "local_pose": [-0.42, 0.0, 0.50, 0.0, 0.08, 0.0],
                "size": [0.84, 1.10, 0.10],
                "radius": 0.2,
                "length": 1.0,
                "color": [0.18, 0.20, 0.22, 1.0],
            },
            {
                "name": "lid_right",
                "shape": "box",
                "local_pose": [0.42, 0.0, 0.50, 0.0, -0.08, 0.0],
                "size": [0.84, 1.10, 0.10],
                "radius": 0.2,
                "length": 1.0,
                "color": [0.18, 0.20, 0.22, 1.0],
            },
        ]
        return parts, [1.8, 1.1, 1.3], [0.16, 0.50, 0.30, 1.0]

    if key == "fuel_forklift":
        parts = [
            {
                "name": "chassis",
                "shape": "box",
                "local_pose": [-0.25, 0.0, -0.45, 0.0, 0.0, 0.0],
                "size": [1.45, 1.05, 0.75],
                "radius": 0.4,
                "length": 1.0,
                "color": [0.95, 0.68, 0.08, 1.0],
            },
            {
                "name": "cab_roof",
                "shape": "box",
                "local_pose": [-0.20, 0.0, 0.42, 0.0, 0.0, 0.0],
                "size": [1.05, 0.98, 0.95],
                "radius": 0.4,
                "length": 1.0,
                "color": [0.20, 0.24, 0.28, 1.0],
            },
            {
                "name": "mast_left",
                "shape": "box",
                "local_pose": [0.52, 0.32, 0.05, 0.0, 0.0, 0.0],
                "size": [0.10, 0.10, 1.85],
                "radius": 0.1,
                "length": 1.85,
                "color": [0.15, 0.17, 0.20, 1.0],
            },
            {
                "name": "mast_right",
                "shape": "box",
                "local_pose": [0.52, -0.32, 0.05, 0.0, 0.0, 0.0],
                "size": [0.10, 0.10, 1.85],
                "radius": 0.1,
                "length": 1.85,
                "color": [0.15, 0.17, 0.20, 1.0],
            },
            {
                "name": "fork_left",
                "shape": "box",
                "local_pose": [0.92, 0.26, -0.88, 0.0, 0.0, 0.0],
                "size": [0.85, 0.12, 0.06],
                "radius": 0.1,
                "length": 0.85,
                "color": [0.35, 0.38, 0.42, 1.0],
            },
            {
                "name": "fork_right",
                "shape": "box",
                "local_pose": [0.92, -0.26, -0.88, 0.0, 0.0, 0.0],
                "size": [0.85, 0.12, 0.06],
                "radius": 0.1,
                "length": 0.85,
                "color": [0.35, 0.38, 0.42, 1.0],
            },
        ]
        return parts, [2.2, 1.1, 2.0], [0.95, 0.68, 0.08, 1.0]

    if key == "fuel_car":
        parts = [
            {
                "name": "lower_body",
                "shape": "box",
                "local_pose": [0.0, 0.0, -0.28, 0.0, 0.0, 0.0],
                "size": [4.20, 1.80, 0.68],
                "radius": 0.5,
                "length": 1.0,
                "color": [0.22, 0.52, 0.88, 1.0],
            },
            {
                "name": "cabin",
                "shape": "box",
                "local_pose": [-0.25, 0.0, 0.34, 0.0, 0.0, 0.0],
                "size": [2.35, 1.58, 0.62],
                "radius": 0.5,
                "length": 1.0,
                "color": [0.18, 0.42, 0.72, 1.0],
            },
            {
                "name": "wheel_fl",
                "shape": "cylinder",
                "local_pose": [1.30, 0.88, -0.50, 1.570796, 0.0, 0.0],
                "radius": 0.32,
                "length": 0.24,
                "size": [0.64, 0.24, 0.64],
                "color": [0.14, 0.15, 0.17, 1.0],
            },
            {
                "name": "wheel_fr",
                "shape": "cylinder",
                "local_pose": [1.30, -0.88, -0.50, 1.570796, 0.0, 0.0],
                "radius": 0.32,
                "length": 0.24,
                "size": [0.64, 0.24, 0.64],
                "color": [0.14, 0.15, 0.17, 1.0],
            },
            {
                "name": "wheel_rl",
                "shape": "cylinder",
                "local_pose": [-1.30, 0.88, -0.50, 1.570796, 0.0, 0.0],
                "radius": 0.32,
                "length": 0.24,
                "size": [0.64, 0.24, 0.64],
                "color": [0.14, 0.15, 0.17, 1.0],
            },
            {
                "name": "wheel_rr",
                "shape": "cylinder",
                "local_pose": [-1.30, -0.88, -0.50, 1.570796, 0.0, 0.0],
                "radius": 0.32,
                "length": 0.24,
                "size": [0.64, 0.24, 0.64],
                "color": [0.14, 0.15, 0.17, 1.0],
            },
        ]
        return parts, [4.3, 1.85, 1.65], [0.22, 0.52, 0.88, 1.0]

    if key == "fuel_chair":
        parts = [
            {
                "name": "base_pedestal",
                "shape": "cylinder",
                "local_pose": [0.0, 0.0, -0.25, 0.0, 0.0, 0.0],
                "radius": 0.26,
                "length": 0.42,
                "size": [0.52, 0.52, 0.42],
                "color": [0.20, 0.22, 0.25, 1.0],
            },
            {
                "name": "seat",
                "shape": "box",
                "local_pose": [0.0, 0.0, -0.02, 0.0, 0.0, 0.0],
                "size": [0.52, 0.52, 0.08],
                "radius": 0.25,
                "length": 0.08,
                "color": [0.18, 0.45, 0.82, 1.0],
            },
            {
                "name": "backrest",
                "shape": "box",
                "local_pose": [-0.22, 0.0, 0.26, 0.0, 0.0, 0.0],
                "size": [0.08, 0.48, 0.48],
                "radius": 0.2,
                "length": 0.48,
                "color": [0.18, 0.45, 0.82, 1.0],
            },
        ]
        return parts, [0.6, 0.6, 0.95], [0.18, 0.45, 0.82, 1.0]

    if key == "fuel_lamp_post":
        parts = [
            {
                "name": "pole",
                "shape": "cylinder",
                "local_pose": [0.0, 0.0, -0.10, 0.0, 0.0, 0.0],
                "radius": 0.08,
                "length": 4.0,
                "size": [0.16, 0.16, 4.0],
                "color": [0.30, 0.33, 0.38, 1.0],
            },
            {
                "name": "arm",
                "shape": "box",
                "local_pose": [0.35, 0.0, 1.90, 0.0, 0.0, 0.0],
                "size": [0.75, 0.10, 0.10],
                "radius": 0.1,
                "length": 0.75,
                "color": [0.30, 0.33, 0.38, 1.0],
            },
            {
                "name": "lamp_head",
                "shape": "sphere",
                "local_pose": [0.68, 0.0, 1.82, 0.0, 0.0, 0.0],
                "radius": 0.16,
                "length": 0.32,
                "size": [0.32, 0.32, 0.32],
                "color": [0.98, 0.92, 0.55, 1.0],
            },
        ]
        return parts, [0.9, 0.4, 4.2], [0.35, 0.38, 0.42, 1.0]

    if key == "fuel_hydrant":
        parts = [
            {
                "name": "body",
                "shape": "cylinder",
                "local_pose": [0.0, 0.0, -0.05, 0.0, 0.0, 0.0],
                "radius": 0.14,
                "length": 0.65,
                "size": [0.28, 0.28, 0.65],
                "color": [0.85, 0.16, 0.16, 1.0],
            },
            {
                "name": "cap",
                "shape": "sphere",
                "local_pose": [0.0, 0.0, 0.28, 0.0, 0.0, 0.0],
                "radius": 0.15,
                "length": 0.30,
                "size": [0.30, 0.30, 0.30],
                "color": [0.85, 0.16, 0.16, 1.0],
            },
            {
                "name": "nozzle_bar",
                "shape": "cylinder",
                "local_pose": [0.0, 0.0, 0.08, 1.570796, 0.0, 0.0],
                "radius": 0.06,
                "length": 0.42,
                "size": [0.12, 0.42, 0.12],
                "color": [0.78, 0.80, 0.82, 1.0],
            },
        ]
        return parts, [0.45, 0.45, 0.85], [0.85, 0.16, 0.16, 1.0]

    if key == "fuel_stop_sign":
        parts = [
            {
                "name": "post",
                "shape": "cylinder",
                "local_pose": [0.0, 0.0, -0.20, 0.0, 0.0, 0.0],
                "radius": 0.035,
                "length": 1.70,
                "size": [0.07, 0.07, 1.70],
                "color": [0.65, 0.68, 0.72, 1.0],
            },
            {
                "name": "sign_plate",
                "shape": "cylinder",
                "local_pose": [0.04, 0.0, 0.68, 0.0, 1.570796, 0.0],
                "radius": 0.32,
                "length": 0.03,
                "size": [0.03, 0.64, 0.64],
                "color": [0.88, 0.14, 0.14, 1.0],
            },
        ]
        return parts, [0.5, 0.65, 2.1], [0.88, 0.14, 0.14, 1.0]

    if key == "fuel_barrel":
        parts = [
            {
                "name": "drum_core",
                "shape": "cylinder",
                "local_pose": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                "radius": 0.29,
                "length": 0.88,
                "size": [0.58, 0.58, 0.88],
                "color": [0.92, 0.38, 0.10, 1.0],
            },
            {
                "name": "stripe_mid",
                "shape": "cylinder",
                "local_pose": [0.0, 0.0, 0.05, 0.0, 0.0, 0.0],
                "radius": 0.30,
                "length": 0.22,
                "size": [0.60, 0.60, 0.22],
                "color": [0.95, 0.96, 0.98, 1.0],
            },
        ]
        return parts, [0.60, 0.60, 0.90], [0.92, 0.38, 0.10, 1.0]

    if key == "fuel_robot":
        parts = [
            {
                "name": "base_chassis",
                "shape": "box",
                "local_pose": [0.0, 0.0, -0.04, 0.0, 0.0, 0.0],
                "size": [0.65, 0.50, 0.22],
                "radius": 0.3,
                "length": 0.22,
                "color": [0.20, 0.65, 0.52, 1.0],
            },
            {
                "name": "lidar_tower",
                "shape": "cylinder",
                "local_pose": [0.12, 0.0, 0.14, 0.0, 0.0, 0.0],
                "radius": 0.07,
                "length": 0.14,
                "size": [0.14, 0.14, 0.14],
                "color": [0.15, 0.18, 0.22, 1.0],
            },
            {
                "name": "wheel_left",
                "shape": "cylinder",
                "local_pose": [0.0, 0.27, -0.08, 1.570796, 0.0, 0.0],
                "radius": 0.12,
                "length": 0.06,
                "size": [0.24, 0.06, 0.24],
                "color": [0.12, 0.13, 0.15, 1.0],
            },
            {
                "name": "wheel_right",
                "shape": "cylinder",
                "local_pose": [0.0, -0.27, -0.08, 1.570796, 0.0, 0.0],
                "radius": 0.12,
                "length": 0.06,
                "size": [0.24, 0.06, 0.24],
                "color": [0.12, 0.13, 0.15, 1.0],
            },
        ]
        return parts, [0.70, 0.60, 0.42], [0.20, 0.65, 0.52, 1.0]

    # Default multi-part industrial module (base frame + main unit + top beacon)
    parts = [
        {
            "name": "plinth_base",
            "shape": "box",
            "local_pose": [0.0, 0.0, -0.42, 0.0, 0.0, 0.0],
            "size": [1.05, 0.85, 0.16],
            "radius": 0.4,
            "length": 0.16,
            "color": [0.24, 0.27, 0.32, 1.0],
        },
        {
            "name": "cabinet_body",
            "shape": "box",
            "local_pose": [0.0, 0.0, 0.05, 0.0, 0.0, 0.0],
            "size": [0.92, 0.74, 0.78],
            "radius": 0.4,
            "length": 0.78,
            "color": [0.28, 0.58, 0.86, 1.0],
        },
        {
            "name": "top_unit",
            "shape": "cylinder",
            "local_pose": [0.0, 0.0, 0.52, 0.0, 0.0, 0.0],
            "radius": 0.16,
            "length": 0.16,
            "size": [0.32, 0.32, 0.16],
            "color": [0.92, 0.65, 0.15, 1.0],
        },
    ]
    return parts, [1.05, 0.85, 1.15], [0.28, 0.58, 0.86, 1.0]


def parse_local_model_sdf_visuals(
    source_path: str,
    meshes_dest_dir: Optional[Path] = None,
) -> Optional[Dict[str, Any]]:
    """
    If `source_path` is a local directory containing `model.sdf`, extract its `<visual>`
    primitives or copy its local `.stl`/`.obj` mesh into `meshes_dest_dir`.
    """
    if not source_path or source_path == "local_workspace":
        return None
    model_dir = Path(source_path)
    if not model_dir.is_dir():
        return None

    sdf_candidates = [model_dir / "model.sdf", *sorted(model_dir.glob("*.sdf"))]
    sdf_file = next((p for p in sdf_candidates if p.is_file()), None)
    if sdf_file is None:
        return None

    try:
        root = ET.parse(sdf_file).getroot()
        parts: List[Dict[str, Any]] = []
        for idx, vis in enumerate(root.findall(".//visual")):
            geom = vis.find("geometry")
            if geom is None:
                continue
            pose_el = vis.find("pose")
            lpose = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
            if pose_el is not None and pose_el.text:
                vals = [float(x) for x in pose_el.text.strip().split()[:6]]
                lpose[: len(vals)] = vals

            box_el = geom.find("box/size")
            cyl_el = geom.find("cylinder")
            sph_el = geom.find("sphere/radius")
            mesh_el = geom.find("mesh/uri")

            if mesh_el is not None and mesh_el.text and meshes_dest_dir is not None:
                uri_txt = mesh_el.text.strip()
                rel_part = uri_txt.split("://")[-1]
                # Try resolving inside model_dir
                for cand in [model_dir / rel_part, model_dir / "meshes" / Path(rel_part).name]:
                    if cand.is_file() and cand.suffix.lower() in (".stl", ".obj"):
                        dest_name = f"ros2_{model_dir.name}_{cand.name}"
                        shutil.copy2(cand, meshes_dest_dir / dest_name)
                        return {"mesh_filename": dest_name}

            if box_el is not None and box_el.text:
                sz = [float(x) for x in box_el.text.strip().split()[:3]]
                parts.append(
                    {
                        "name": f"vis_{idx}",
                        "shape": "box",
                        "local_pose": lpose,
                        "size": sz,
                        "radius": 0.2,
                        "length": 1.0,
                        "color": [0.28, 0.65, 0.56, 1.0],
                    }
                )
            elif cyl_el is not None:
                r = float(cyl_el.findtext("radius", "0.25"))
                l = float(cyl_el.findtext("length", "0.5"))
                parts.append(
                    {
                        "name": f"vis_{idx}",
                        "shape": "cylinder",
                        "local_pose": lpose,
                        "radius": r,
                        "length": l,
                        "size": [r * 2, r * 2, l],
                        "color": [0.28, 0.65, 0.56, 1.0],
                    }
                )
            elif sph_el is not None and sph_el.text:
                r = float(sph_el.text.strip())
                parts.append(
                    {
                        "name": f"vis_{idx}",
                        "shape": "sphere",
                        "local_pose": lpose,
                        "radius": r,
                        "length": r * 2,
                        "size": [r * 2, r * 2, r * 2],
                        "color": [0.28, 0.65, 0.56, 1.0],
                    }
                )
        if parts:
            return {"parts": parts}
    except Exception:
        pass
    return None


def search_gazebo_fuel(query: str = "", timeout_sec: float = 2.5) -> List[Dict[str, Any]]:
    """
    Search Gazebo Fuel models matching `query`. Combines curated instant models
    with live results from `https://fuel.gazebosim.org/1.0/models` when reachable.
    """
    q = query.strip().lower()
    results: List[Dict[str, Any]] = []

    for item in CURATED_FUEL_MODELS:
        if (
            not q
            or q in item["name"].lower()
            or q in item["model_name"].lower()
            or q in item["category"].lower()
        ):
            results.append(dict(item))

    if q and len(q) >= 2:
        try:
            url = f"https://fuel.gazebosim.org/1.0/models?q={urllib.parse.quote(q)}&per_page=8"
            req = urllib.request.Request(url, headers={"Accept": "application/json", "User-Agent": "Custebo/1.0"})
            with urllib.request.urlopen(req, timeout=timeout_sec) as resp:
                if resp.status == 200:
                    data = json.loads(resp.read().decode("utf-8"))
                    seen_uris = {r["uri"] for r in results}
                    if isinstance(data, list):
                        for entry in data[:8]:
                            owner = entry.get("owner", "OpenRobotics")
                            mname = entry.get("name", "model")
                            encoded_name = urllib.parse.quote(mname)
                            uri = f"https://fuel.gazebosim.org/1.0/{owner}/models/{encoded_name}"
                            if uri not in seen_uris:
                                seen_uris.add(uri)
                                _, bsize, mcolor = build_procedural_fuel_parts("", mname)
                                results.append(
                                    {
                                        "name": f"{mname} ({owner})",
                                        "model_name": mname,
                                        "owner": owner,
                                        "uri": uri,
                                        "category": "Gazebo Fuel Online",
                                        "size": bsize,
                                        "color": mcolor,
                                        "static": True,
                                        "proxy_archetype": None,
                                    }
                                )
        except Exception:
            pass

    return results


def scan_ros2_workspaces(extra_paths: Optional[List[str]] = None) -> List[Dict[str, Any]]:
    """
    Scan `AMENT_PREFIX_PATH`, `GZ_SIM_RESOURCE_PATH`, `/opt/ros/jazzy/share`, and any
    user-provided workspace paths for Gazebo models (`model.config`) and ROS 2 package meshes.
    """
    search_roots: List[Path] = []
    for env_var in ("AMENT_PREFIX_PATH", "GZ_SIM_RESOURCE_PATH", "GAZEBO_MODEL_PATH"):
        val = os.environ.get(env_var, "")
        for token in val.split(os.pathsep):
            if token.strip():
                p = Path(token.strip())
                if p.is_dir():
                    search_roots.append(p)

    if extra_paths:
        for ep in extra_paths:
            if ep and Path(ep).is_dir():
                search_roots.append(Path(ep))

    jazzy_share = Path("/opt/ros/jazzy/share")
    if jazzy_share.is_dir() and jazzy_share not in search_roots:
        search_roots.append(jazzy_share)

    discovered: List[Dict[str, Any]] = []
    seen_uris = set()

    for root in search_roots:
        try:
            for dirpath, dirnames, filenames in os.walk(root):
                rel_depth = len(Path(dirpath).relative_to(root).parts)
                if rel_depth > 4:
                    dirnames[:] = []
                    continue

                if "model.config" in filenames:
                    model_dir = Path(dirpath)
                    model_name = model_dir.name
                    try:
                        cfg_root = ET.parse(model_dir / "model.config").getroot()
                        name_el = cfg_root.find("name")
                        if name_el is not None and name_el.text:
                            model_name = name_el.text.strip()
                    except Exception:
                        pass

                    uri = f"model://{model_dir.name}"
                    if uri not in seen_uris:
                        seen_uris.add(uri)
                        discovered.append(
                            {
                                "name": model_name,
                                "uri": uri,
                                "source_path": str(model_dir),
                                "type": "gazebo_model",
                            }
                        )
                        if len(discovered) >= 40:
                            break
        except Exception:
            continue

    # Always provide rich built-in ROS 2 local workspace models for quick insertion
    builtin_ros_models = [
        {
            "name": "Robot AMR Diferencial (ROS 2 Base)",
            "uri": "model://custebo_amr_robot",
            "source_path": "local_workspace",
            "type": "gazebo_model",
            "proxy_archetype": "fuel_robot",
        },
        {
            "name": "Carretilla Logística AGV (ROS 2)",
            "uri": "model://custebo_forklift_agv",
            "source_path": "local_workspace",
            "type": "gazebo_model",
            "proxy_archetype": "fuel_forklift",
        },
        {
            "name": "Estación de Carga Industrial (ROS 2)",
            "uri": "model://custebo_charging_dock",
            "source_path": "local_workspace",
            "type": "gazebo_model",
            "proxy_archetype": "fuel_industrial_module",
        },
    ]
    for bm in builtin_ros_models:
        if bm["uri"] not in seen_uris:
            discovered.insert(0, bm)

    return discovered
