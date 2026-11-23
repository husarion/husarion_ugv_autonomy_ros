# Copyright 2026 Husarion sp. z o.o.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Parameter overlays for the map-nav launch arguments.

Every function here is pure, so the launch files and the unit tests share one code path. With
every map-nav argument at its default, `build_overlay` returns an empty dict and the stack's
parameters are exactly those of `nav2_params.yaml`.
"""

import copy
import hashlib
import math
import os

import yaml

# Robot bounding boxes (base_footprint frame). `bb_padding` widens the footprint for safety.
BB_PADDING = 0.03
ROBOT_BOUNDING_BOX = {
    "panther": {
        "min_x": -0.41 - BB_PADDING,
        "min_y": -0.43 - BB_PADDING,
        "min_z": 0.05,
        "max_x": 0.41 + BB_PADDING,
        "max_y": 0.43 + BB_PADDING,
        "max_z": 0.5,
    },
    "lynx": {
        "min_x": -0.32 - BB_PADDING,
        "min_y": -0.27 - BB_PADDING,
        "min_z": 0.05,
        "max_x": 0.32 + BB_PADDING,
        "max_y": 0.27 + BB_PADDING,
        "max_z": 0.5,
    },
}

# Ouster sensor profiles (datasheet Rev 7). `vertical_fov` is the full vertical field in radians,
# `range_min` the sensor's minimum range in metres, `speed_cap` a linear speed cap in m/s that holds
# at all times (None: the stack's own limits). The OS2-64 cannot see the near field (D55): a
# collision-monitor `limit` polygon acts only while points are inside it, so it would release the
# cap exactly when an obstacle enters the blind zone. The cap therefore goes into the controller
# and the velocity smoother (Dominik, 2026-11-16).
LIDAR_PROFILES = {
    "os0_32": {"vertical_fov": 1.571, "range_min": 0.5, "speed_cap": None},
    "os2_64": {"vertical_fov": 0.393, "range_min": 0.8, "speed_cap": 0.3},
}

# Fixed values of the 3D path (spec SPEC-MAP-NAV-2026-11 §5.7).
OBSTACLE_MIN_HEIGHT = 0.1  # m above base_footprint: below this is ground
ABOVE_ROBOT_TOP = 0.2  # m: obstacles count up to the robot's top plus this
CROP_ABOVE_TOP = 0.05  # m: the crop box reaches the robot's top plus this
SCAN_BINS = 1024  # one LaserScan bin per Ouster column (lidar_mode 1024x10)
SCAN_RANGE_MAX = 20.0
# m above base_footprint, (min, max) of the 2D scan slice per observation type
SCAN_HEIGHTS = {"laserscan": ("0.05", "2.0"), "pointcloud": ("0.25", "1.0")}
STOP_MARGIN = 0.15  # m: the stop polygon is the footprint plus this
SLOWDOWN_MARGIN = 0.6  # m: the slowdown polygon is the footprint plus this
SLOWDOWN_RATIO = 0.5
# The sensor sits on the rail; nothing of the robot (or of the sensor) is above this much over it.
SENSOR_HEIGHT_ABOVE_RAIL = 0.08

SIDECAR_SUFFIX = ".georef.yaml"

LIFECYCLE_NODES = [
    "controller_server",
    "smoother_server",
    "planner_server",
    "behavior_server",
    "bt_navigator",
    "waypoint_follower",
    "velocity_smoother",
]


def scan_heights(observation_topic_type: str, scan_min_height: str, scan_max_height: str):
    """The 2D scan slice's (min, max) launch strings: explicit values, else the type's."""
    default_min, default_max = SCAN_HEIGHTS.get(observation_topic_type, ("", ""))
    return scan_min_height or default_min, scan_max_height or default_max


def velocity_routing(use_collision_monitor: bool, cmd_vel_out: str):
    """Where the velocity smoother and the behaviors publish, and the lifecycle list.

    Defaults (no collision monitor, `cmd_vel_out` = `cmd_vel`) keep the stack as it was: the
    smoother publishes `cmd_vel`, and the behaviors publish `cmd_vel` directly. Otherwise the
    behaviors join the controller on `cmd_vel_nav`, so every command passes the smoother and, when
    it runs, the collision monitor (`cmd_vel_smoothed` -> `collision_monitor` -> `cmd_vel_out`).
    """
    routed = use_collision_monitor or cmd_vel_out != "cmd_vel"
    smoother_out = "cmd_vel_smoothed" if use_collision_monitor else cmd_vel_out
    behavior_out = "cmd_vel_nav" if routed else "cmd_vel"
    lifecycle_nodes = LIFECYCLE_NODES + (["collision_monitor"] if use_collision_monitor else [])
    return smoother_out, behavior_out, lifecycle_nodes


def is_true(value) -> bool:
    """Launch arguments arrive as strings."""
    if isinstance(value, bool):
        return value
    return str(value).strip().lower() in ("true", "1", "yes", "on")


def namespace_prefix(namespace: str) -> str:
    """Frame prefix for a namespace: 'panther_0957' -> 'panther_0957/', '' -> ''."""
    namespace = namespace.strip("/")
    return f"{namespace}/" if namespace else ""


def footprint_points(robot_model: str, margin: float = 0.0) -> str:
    """The footprint rectangle, grown by `margin`, in Nav2's '[[x, y], ...]' string form."""
    box = ROBOT_BOUNDING_BOX[robot_model]
    min_x, max_x = box["min_x"] - margin, box["max_x"] + margin
    min_y, max_y = box["min_y"] - margin, box["max_y"] + margin
    points = [[min_x, min_y], [min_x, max_y], [max_x, max_y], [max_x, min_y]]
    return "[" + ", ".join(f"[{x:.3f}, {y:.3f}]" for x, y in points) + "]"


def render_placeholders(
    text: str,
    namespace: str,
    robot_model: str,
    observation_topic: str,
    observation_topic_type: str,
) -> str:
    """The placeholder replacement `bringup_launch.py` has always done with ReplaceString."""
    box = ROBOT_BOUNDING_BOX[robot_model]
    pointcloud = observation_topic_type == "pointcloud"
    replacements = {
        "<namespace>/": namespace_prefix(namespace),
        "<min_x>": str(box["min_x"]),
        "<max_x>": str(box["max_x"]),
        "<min_y>": str(box["min_y"]),
        "<max_y>": str(box["max_y"]),
        "<min_z>": str(box["min_z"]),
        "<max_z>": str(box["max_z"]),
        "<observation_topic>": observation_topic,
        "<observation_topic_type>": observation_topic_type,
        "<scan_topic>": "scan" if pointcloud else observation_topic,
        "<stvl_layer>": "stvl_pointcloud_layer" if pointcloud else "stvl_laserscan_layer",
    }
    for key, value in replacements.items():
        text = text.replace(key, value)
    return text


def deep_merge(base: dict, overlay: dict) -> dict:
    """Merge `overlay` into a copy of `base`; dicts merge, anything else replaces."""
    merged = copy.deepcopy(base)
    for key, value in overlay.items():
        if isinstance(value, dict) and isinstance(merged.get(key), dict):
            merged[key] = deep_merge(merged[key], value)
        else:
            merged[key] = copy.deepcopy(value)
    return merged


def collision_monitor_params(
    namespace: str,
    robot_model: str,
    observation_topic: str,
    observation_topic_type: str,
    cmd_vel_out: str,
    lidar_profile: str,
    robot_top: float,
) -> dict:
    """Parameters of nav2_collision_monitor between the smoother and the robot's input."""
    prefix = namespace_prefix(namespace)
    polygons = {
        "FootprintStop": {
            "type": "polygon",
            "points": footprint_points(robot_model, STOP_MARGIN),
            "action_type": "stop",
            "min_points": 4,
            "visualize": True,
            "polygon_pub_topic": "collision_monitor/stop_polygon",
            "enabled": True,
        },
        "Slowdown": {
            "type": "polygon",
            "points": footprint_points(robot_model, SLOWDOWN_MARGIN),
            "action_type": "slowdown",
            "slowdown_ratio": SLOWDOWN_RATIO,
            "min_points": 4,
            "visualize": True,
            "polygon_pub_topic": "collision_monitor/slowdown_polygon",
            "enabled": True,
        },
    }
    if observation_topic_type == "pointcloud":
        source = {
            "type": "pointcloud",
            "topic": f"{observation_topic}_filtered",
            "min_height": OBSTACLE_MIN_HEIGHT,
            "max_height": round(robot_top + ABOVE_ROBOT_TOP, 3),
            "enabled": True,
        }
    else:
        source = {"type": "scan", "topic": observation_topic, "enabled": True}

    params = {
        "use_sim_time": False,
        "enable_stamped_cmd_vel": True,
        "base_frame_id": f"{prefix}base_footprint",
        "odom_frame_id": f"{prefix}odom",
        "cmd_vel_in_topic": "cmd_vel_smoothed",
        "cmd_vel_out_topic": cmd_vel_out,
        "state_topic": "collision_monitor_state",
        "transform_tolerance": 0.2,
        "source_timeout": 1.0,
        "base_shift_correction": True,
        "stop_pub_timeout": 2.0,
        "polygons": list(polygons.keys()),
        "observation_sources": ["cloud"],
        "cloud": source,
    }
    params.update(polygons)
    return params


def build_overlay(
    *,
    namespace: str,
    robot_model: str,
    observation_topic: str,
    observation_topic_type: str,
    cmd_vel_out: str = "cmd_vel",
    use_collision_monitor=False,
    lidar_profile: str = "",
    mount_height=None,
    robot_height=None,
    scan_min_height: str = "",
    scan_max_height: str = "",
    map_update_interval: str = "",
    initial_pose=None,
) -> dict:
    """The parameter overlay for the map-nav arguments; {} when every one is at its default.

    `mount_height` is the rail level (mount_link) above base_footprint in metres; it is needed
    when `lidar_profile` is set. `robot_height` is the robot's top with its payload above
    base_footprint; obstacles count up to it plus 0.2 m. Empty with a `lidar_profile`: the rail
    plus the sensor. `initial_pose` is (x, y, yaw) in the map frame, or None.
    """
    nodes = {}

    def node(name):
        return nodes.setdefault(name, {"ros__parameters": {}})["ros__parameters"]

    profile = None
    robot_top = ROBOT_BOUNDING_BOX[robot_model]["max_z"]
    if lidar_profile:
        if lidar_profile not in LIDAR_PROFILES:
            raise ValueError(
                f"lidar_profile '{lidar_profile}' is not one of {sorted(LIDAR_PROFILES)}"
            )
        if mount_height is None:
            raise ValueError("lidar_profile needs the mount_link height above base_footprint")
        profile = LIDAR_PROFILES[lidar_profile]
        robot_top = float(mount_height) + SENSOR_HEIGHT_ABOVE_RAIL
        if robot_height not in (None, ""):
            # the robot's real top with its payload, measured by whoever runs it
            robot_top = float(robot_height)

        # The crop box drops the robot itself: everything inside the footprint up to the robot's
        # top, so a sensor gate (DEV09), cameras and antennas above the rail are not obstacles.
        node("pointcloud_crop_box")["crop_boxes"] = {
            "robot_box": {"max_z": round(robot_top + CROP_ABOVE_TOP, 4)}
        }

        # The 2D slice for SLAM and AMCL: one bin per column, the sensor's minimum range.
        node("pointcloud_to_laserscan").update(
            {
                "angle_min": -math.pi,
                "angle_max": math.pi,
                "angle_increment": 2.0 * math.pi / SCAN_BINS,
                "range_min": profile["range_min"],
                "range_max": SCAN_RANGE_MAX,
            }
        )

        # STVL's clearing frustum becomes a 3D LiDAR's (model_type 1).
        stvl_pointcloud = {
            "model_type": 1,
            "vertical_fov_angle": profile["vertical_fov"],
            "vertical_fov_padding": 0.05,
            "horizontal_fov_angle": 6.29,
            "min_z": profile["range_min"],
            "min_obstacle_height": OBSTACLE_MIN_HEIGHT,
            "max_obstacle_height": round(robot_top + ABOVE_ROBOT_TOP, 3),
        }
        for costmap in ("local_costmap", "global_costmap"):
            nodes.setdefault(costmap, {}).setdefault(costmap, {"ros__parameters": {}})
            nodes[costmap][costmap]["ros__parameters"]["stvl_layer"] = {
                "pointcloud": dict(stvl_pointcloud)
            }

    if scan_min_height != "":
        node("pointcloud_to_laserscan")["min_height"] = float(scan_min_height)
    if scan_max_height != "":
        node("pointcloud_to_laserscan")["max_height"] = float(scan_max_height)

    if map_update_interval != "":
        node(f"{namespace_prefix(namespace)}slam_toolbox")["map_update_interval"] = float(
            map_update_interval
        )

    if is_true(use_collision_monitor):
        nodes["collision_monitor"] = {
            "ros__parameters": collision_monitor_params(
                namespace,
                robot_model,
                observation_topic,
                observation_topic_type,
                cmd_vel_out,
                lidar_profile,
                robot_top,
            )
        }

    if initial_pose is not None:
        x, y, yaw = initial_pose
        node("amcl").update(
            {
                "set_initial_pose": True,
                "always_reset_initial_pose": True,
                "initial_pose": {"x": float(x), "y": float(y), "z": 0.0, "yaw": float(yaw)},
            }
        )

    return {"/**": nodes} if nodes else {}


def speed_cap(lidar_profile: str):
    """The always-on linear speed cap of a LiDAR profile in m/s, or None."""
    profile = LIDAR_PROFILES.get(lidar_profile)
    return profile["speed_cap"] if profile else None


def clamp_linear_speed(tree: dict, cap: float) -> dict:
    """Clamp forward and reverse linear speed to `cap` in the controller and the smoother.

    Values below the cap stay; nothing else in the file changes.
    """
    tree = copy.deepcopy(tree)
    nodes = tree["/**"]
    controller = nodes["controller_server"]["ros__parameters"]
    for name in controller.get("controller_plugins", []):
        plugin = controller.get(name, {})
        if "vx_max" in plugin:
            plugin["vx_max"] = min(float(plugin["vx_max"]), cap)
        if "vx_min" in plugin:
            plugin["vx_min"] = max(float(plugin["vx_min"]), -cap)
    smoother = nodes["velocity_smoother"]["ros__parameters"]
    smoother["max_velocity"] = [min(float(smoother["max_velocity"][0]), cap)] + list(
        smoother["max_velocity"][1:]
    )
    smoother["min_velocity"] = [max(float(smoother["min_velocity"][0]), -cap)] + list(
        smoother["min_velocity"][1:]
    )
    return tree


def apply_overlay(rendered_yaml: str, overlay: dict, cap=None) -> str:
    """The rendered parameter file with the overlay merged in and the speed cap applied."""
    tree = deep_merge(yaml.safe_load(rendered_yaml), overlay)
    if cap is not None:
        tree = clamp_linear_speed(tree, cap)
    return yaml.safe_dump(tree, sort_keys=False)


def sidecar_path(map_yaml: str) -> str:
    """`<dir>/<stem>.georef.yaml` beside `<dir>/<stem>.yaml` (spec §5.5)."""
    stem, _ = os.path.splitext(map_yaml)
    return stem + SIDECAR_SUFFIX


def read_saved_pose(map_yaml: str):
    """The robot's pose when the map was saved, (x, y, yaw), from the map's sidecar.

    Raises ValueError when the sidecar is missing, belongs to another map image, or has no
    usable `saved_pose`: AMCL must never silently start at 0, 0, 0 on a saved map.
    """
    path = sidecar_path(map_yaml)
    if not os.path.isfile(path):
        raise ValueError(f"no sidecar {path} beside {map_yaml}")
    with open(path, encoding="utf-8") as f:
        sidecar = yaml.safe_load(f) or {}
    with open(map_yaml, encoding="utf-8") as f:
        image = (yaml.safe_load(f) or {}).get("image")
    if not image:
        raise ValueError(f"{map_yaml} names no image")
    if not os.path.isabs(image):
        image = os.path.join(os.path.dirname(map_yaml), image)
    with open(image, "rb") as f:
        digest = hashlib.sha256(f.read()).hexdigest()
    if sidecar.get("map_sha256") != digest:
        raise ValueError(
            f"{path} belongs to another map image (map_sha256 does not match {image})"
        )
    pose = sidecar.get("saved_pose")
    if not isinstance(pose, dict):
        raise ValueError(f"{path} has no saved_pose")
    try:
        values = tuple(float(pose[k]) for k in ("x", "y", "yaw"))
    except (KeyError, TypeError, ValueError) as e:
        raise ValueError(f"{path}: saved_pose needs numeric x, y and yaw ({e})") from e
    if not all(math.isfinite(v) for v in values):
        raise ValueError(f"{path}: saved_pose is not finite")
    return values
