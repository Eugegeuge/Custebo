"""
ROS 2 & Nav2 Navigation, Sensor Bridge, Robot Spawn & Dynamic Actor Tools (`core/ros2/navigation_tools.py`).

Implements Phase 4.2, 4.3, and 4.4:
  - 4.2: Nav2 Waypoint & Topological Graph nodes (`type="waypoint"`) + YAML exporter
  - 4.3: Robot Spawn Poses (`type="robot_spawn"`), Static Sensors (`type="sensor"`: LiDAR 2D/3D, RGB-D)
         with 3D Frustum visualization parameters & automatic `ros_gz_bridge.yaml` generation
  - 4.4: Dynamic Pedestrian / Obstacle Actors (`type="actor"`) with multi-waypoint trajectories
"""

from __future__ import annotations

import math
from typing import Any, Dict, List, Optional

from core.math_physics.transforms import clean_float, euler_rpy_to_quaternion
from core.schema.scene_graph import SceneGraphDocument, generate_node_id, normalize_node_in_place


# ==============================================================================
# 1. NODE FACTORIES FOR WAYPOINTS, ROBOT SPAWNS, STATIC SENSORS & ACTORS
# ==============================================================================

def create_waypoint_node(
    name: str = "wp_1",
    pose: Optional[List[float]] = None,
    connected_to: Optional[List[str]] = None,
    dwell_time_s: float = 0.0,
    folder_id: str = "folder_ros2",
    node_id: Optional[str] = None,
) -> Dict[str, Any]:
    """Create a Nav2 Waypoint / Topological Graph node (`(x, y, yaw)` on the ground)."""
    if pose is None:
        pose = [0.0, 0.0, 0.02, 0.0, 0.0, 0.0]
    node: Dict[str, Any] = {
        "id": node_id or generate_node_id("waypoint"),
        "name": name,
        "type": "waypoint",
        "parent_id": None,
        "folder_id": folder_id,
        "visible": True,
        "locked": False,
        "static": True,
        "pose": [clean_float(float(v)) for v in pose[:6]],
        "scale": [1.0, 1.0, 1.0],
        "waypoint": {
            "dwell_time_s": clean_float(float(dwell_time_s)),
            "connected_to": list(connected_to or []),
            "task_type": "navigate_through_poses",
        },
        "material": {
            "color": [0.02, 0.78, 0.96, 1.0],
            "roughness": 0.3,
            "metalness": 0.2,
        },
        "collision": {"enabled": False},
    }
    return normalize_node_in_place(node)


def create_robot_spawn_node(
    name: str = "robot_spawn_1",
    robot_model: str = "turtlebot4",
    robot_namespace: str = "robot1",
    package_uri: str = "package://turtlebot4_description/urdf/standard/turtlebot4.urdf.xacro",
    pose: Optional[List[float]] = None,
    folder_id: str = "folder_ros2",
    node_id: Optional[str] = None,
) -> Dict[str, Any]:
    """Create a visual Robot Spawn Pose marker (`-x -y -z -R -P -Y` for `ros_gz_sim create`)."""
    if pose is None:
        pose = [0.0, 0.0, 0.05, 0.0, 0.0, 0.0]
    node: Dict[str, Any] = {
        "id": node_id or generate_node_id("spawn"),
        "name": name,
        "type": "robot_spawn",
        "parent_id": None,
        "folder_id": folder_id,
        "visible": True,
        "locked": False,
        "static": True,
        "pose": [clean_float(float(v)) for v in pose[:6]],
        "scale": [1.0, 1.0, 1.0],
        "robot_spawn": {
            "robot_model": robot_model,
            "robot_namespace": robot_namespace,
            "package_uri": package_uri,
            "use_nav2": True,
        },
        "material": {
            "color": [0.16, 0.85, 0.45, 1.0],
            "roughness": 0.35,
            "metalness": 0.3,
        },
        "collision": {"enabled": False},
    }
    return normalize_node_in_place(node)


def create_static_sensor_node(
    name: str = "sensor_camera_1",
    sensor_type: str = "camera_rgbd",  # "camera_rgbd" | "lidar_3d" | "lidar_2d"
    pose: Optional[List[float]] = None,
    topic: Optional[str] = None,
    update_rate: float = 15.0,
    fov_deg: float = 75.0,
    range_max: float = 8.0,
    folder_id: str = "folder_ros2",
    node_id: Optional[str] = None,
) -> Dict[str, Any]:
    """Create a static environmental sensor (RGB-D Camera, 3D LiDAR, or 2D LiDAR) with 3D Frustum."""
    if pose is None:
        # Place at ceiling/wall monitoring height Z=2.2m pitched slightly down
        pose = [2.0, 2.0, 2.2, 0.0, 0.35, -2.35] if sensor_type == "camera_rgbd" else [0.0, 0.0, 1.8, 0.0, 0.0, 0.0]
    if topic is None:
        clean_name = "".join(c if c.isalnum() or c == "_" else "_" for c in name)
        topic = f"/world_sensors/{clean_name}"

    node: Dict[str, Any] = {
        "id": node_id or generate_node_id("sensor"),
        "name": name,
        "type": "sensor",
        "parent_id": None,
        "folder_id": folder_id,
        "visible": True,
        "locked": False,
        "static": True,
        "pose": [clean_float(float(v)) for v in pose[:6]],
        "scale": [1.0, 1.0, 1.0],
        "sensor": {
            "sensor_type": sensor_type,
            "topic": topic,
            "update_rate": clean_float(float(update_rate)),
            "fov_deg": clean_float(float(fov_deg)),
            "range_min": 0.12,
            "range_max": clean_float(float(range_max)),
            "width": 640,
            "height": 480,
            "horizontal_samples": 720,
            "vertical_layers": 16 if sensor_type == "lidar_3d" else 1,
            "show_frustum": True,
        },
        "material": {
            "color": [0.65, 0.35, 0.96, 1.0],
            "roughness": 0.3,
            "metalness": 0.4,
        },
        "collision": {"enabled": False},
    }
    return normalize_node_in_place(node)


def create_dynamic_actor_node(
    name: str = "actor_walker_1",
    pose: Optional[List[float]] = None,
    waypoints: Optional[List[List[float]]] = None,
    speed_mps: float = 1.1,
    loop: bool = True,
    skin_uri: str = "https://fuel.gazebosim.org/1.0/Mingfei/models/actor/tip/files/meshes/walk.dae",
    folder_id: str = "folder_ros2",
    node_id: Optional[str] = None,
) -> Dict[str, Any]:
    """Create a dynamic pedestrian or moving obstacle `<actor>` following a trajectory."""
    if pose is None:
        pose = [-3.0, -2.0, 1.0, 0.0, 0.0, 0.0]
    x0, y0 = float(pose[0]), float(pose[1])
    if not waypoints or len(waypoints) < 2:
        waypoints = [
            [round(x0, 3), round(y0, 3), 1.0],
            [round(x0 + 4.0, 3), round(y0, 3), 1.0],
            [round(x0 + 4.0, 3), round(y0 + 3.0, 3), 1.0],
            [round(x0, 3), round(y0 + 3.0, 3), 1.0],
        ]
    node: Dict[str, Any] = {
        "id": node_id or generate_node_id("actor"),
        "name": name,
        "type": "actor",
        "parent_id": None,
        "folder_id": folder_id,
        "visible": True,
        "locked": False,
        "static": True,
        "pose": [clean_float(float(v)) for v in pose[:6]],
        "scale": [1.0, 1.0, 1.0],
        "actor": {
            "speed_mps": clean_float(max(0.1, float(speed_mps))),
            "loop": bool(loop),
            "skin_uri": skin_uri,
            "animation_name": "walk",
            "waypoints": [[clean_float(float(c)) for c in pt[:3]] for pt in waypoints],
        },
        "material": {
            "color": [0.96, 0.45, 0.15, 1.0],
            "roughness": 0.4,
            "metalness": 0.1,
        },
        "collision": {"enabled": False},
    }
    return normalize_node_in_place(node)


# ==============================================================================
# 2. NAV2 WAYPOINTS & TOPOLOGICAL ROUTE YAML EXPORTER
# ==============================================================================

def export_nav2_waypoints_yaml(doc: SceneGraphDocument) -> str:
    """
    Export all `waypoint` nodes in the Scene Graph into a YAML file directly compatible
    with `nav2_waypoint_follower` and `nav2_route` topological graph servers.
    """
    wp_nodes = [n for n in doc.nodes if n.get("type") == "waypoint" and n.get("visible", True)]
    lines: List[str] = [
        "# Generated by Custebo 3D World Editor — Nav2 Waypoints & Topological Route",
        "nav2_waypoints:",
        "  ros__parameters:",
        "    frame_id: \"map\"",
        "    stop_on_failure: false",
        "    loop_rate: 20",
        "    waypoints:",
    ]

    if not wp_nodes:
        lines.append("      []")
        return "\n".join(lines) + "\n"

    for idx, wp in enumerate(wp_nodes):
        pose = wp.get("pose", [0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        x, y, z, r, p, yaw = [float(v) for v in pose[:6]]
        qx, qy, qz, qw = euler_rpy_to_quaternion(r, p, yaw)
        wp_info = wp.get("waypoint", {})
        dwell = float(wp_info.get("dwell_time_s", 0.0))
        edges = wp_info.get("connected_to", [])
        if not edges and idx + 1 < len(wp_nodes):
            edges = [wp_nodes[idx + 1].get("name", f"wp_{idx+2}")]

        lines.extend(
            [
                f"      - id: {idx}",
                f"        name: \"{wp.get('name', f'wp_{idx+1}')}\"",
                "        pose:",
                f"          position: {{x: {x:.4f}, y: {y:.4f}, z: 0.0}}",
                f"          orientation: {{x: {qx:.6f}, y: {qy:.6f}, z: {qz:.6f}, w: {qw:.6f}}}",
                f"          yaw_rad: {yaw:.4f}",
                f"        dwell_time_s: {dwell:.2f}",
                f"        connected_to: [{', '.join(f'\"{e}\"' for e in edges)}]",
            ]
        )

    return "\n".join(lines) + "\n"


# ==============================================================================
# 3. AUTOMATIC `ros_gz_bridge.yaml` GENERATOR FOR SENSORS & CLOCK
# ==============================================================================

def generate_ros_gz_bridge_yaml(doc: SceneGraphDocument) -> str:
    """
    Generate a complete `ros_gz_bridge.yaml` mapping Gazebo Harmonic topics
    to ROS 2 topics for `/clock` and every static `<sensor>` in the world.
    """
    sensor_nodes = [n for n in doc.nodes if n.get("type") == "sensor" and n.get("visible", True)]

    lines: List[str] = [
        "# Generated by Custebo 3D World Editor — ros_gz_bridge Configuration",
        "- ros_topic_name: \"/clock\"",
        "  gz_topic_name: \"/clock\"",
        "  ros_type_name: \"rosgraph_msgs/msg/Clock\"",
        "  gz_type_name: \"gz.msgs.Clock\"",
        "  direction: GZ_TO_ROS",
    ]

    for snode in sensor_nodes:
        sdata = snode.get("sensor", {})
        stype = sdata.get("sensor_type", "camera_rgbd")
        topic = sdata.get("topic", f"/world_sensors/{snode.get('name', 'sensor')}")

        if stype == "camera_rgbd":
            lines.extend(
                [
                    f"- ros_topic_name: \"{topic}/image\"",
                    f"  gz_topic_name: \"{topic}/image\"",
                    "  ros_type_name: \"sensor_msgs/msg/Image\"",
                    "  gz_type_name: \"gz.msgs.Image\"",
                    "  direction: GZ_TO_ROS",
                    f"- ros_topic_name: \"{topic}/depth_image\"",
                    f"  gz_topic_name: \"{topic}/depth_image\"",
                    "  ros_type_name: \"sensor_msgs/msg/Image\"",
                    "  gz_type_name: \"gz.msgs.Image\"",
                    "  direction: GZ_TO_ROS",
                    f"- ros_topic_name: \"{topic}/points\"",
                    f"  gz_topic_name: \"{topic}/points\"",
                    "  ros_type_name: \"sensor_msgs/msg/PointCloud2\"",
                    "  gz_type_name: \"gz.msgs.PointCloudPacked\"",
                    "  direction: GZ_TO_ROS",
                ]
            )
        elif stype == "lidar_3d":
            lines.extend(
                [
                    f"- ros_topic_name: \"{topic}/points\"",
                    f"  gz_topic_name: \"{topic}/scan/points\"",
                    "  ros_type_name: \"sensor_msgs/msg/PointCloud2\"",
                    "  gz_type_name: \"gz.msgs.PointCloudPacked\"",
                    "  direction: GZ_TO_ROS",
                ]
            )
        else:  # lidar_2d
            lines.extend(
                [
                    f"- ros_topic_name: \"{topic}/scan\"",
                    f"  gz_topic_name: \"{topic}/scan\"",
                    "  ros_type_name: \"sensor_msgs/msg/LaserScan\"",
                    "  gz_type_name: \"gz.msgs.LaserScan\"",
                    "  direction: GZ_TO_ROS",
                ]
            )

    return "\n".join(lines) + "\n"

