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

"""Unit gates of the map-nav launch arguments (SPEC-MAP-NAV-2026-11 R1, §11.6)."""

import hashlib
import math
import os

import pytest
import yaml

from husarion_ugv_navigation import mapnav_params as mp

PARAMS_FILE = os.path.join(os.path.dirname(__file__), "..", "config", "nav2_params.yaml")
NS = "panther_0957"
CLOUD = f"/{NS}/points"

MAPNAV = dict(
    namespace=NS,
    robot_model="panther",
    observation_topic=CLOUD,
    observation_topic_type="pointcloud",
    cmd_vel_out="autonomous/cmd_vel",
    use_collision_monitor="True",
    lidar_profile="os0_32",
    mount_height=0.354,
    scan_min_height="0.25",
    scan_max_height="1.0",
    map_update_interval="5.0",
)

STAMPED_NODES = [
    "controller_server",
    "behavior_server",
    "waypoint_follower",
    "velocity_smoother",
    "collision_monitor",
]


def rendered(**kwargs):
    with open(PARAMS_FILE, encoding="utf-8") as f:
        text = mp.render_placeholders(
            f.read(),
            kwargs["namespace"],
            kwargs["robot_model"],
            kwargs["observation_topic"],
            kwargs["observation_topic_type"],
        )
    overlay = mp.build_overlay(**kwargs)
    cap = mp.speed_cap(kwargs.get("lidar_profile", ""))
    if overlay or cap is not None:
        text = mp.apply_overlay(text, overlay, cap)
    return yaml.safe_load(text)["/**"]


def params(tree, *path):
    node = tree
    for key in path:
        node = node[key]
    return node["ros__parameters"]


# --- defaults keep today's behaviour -------------------------------------------------------------


def test_defaults_give_no_overlay():
    assert (
        mp.build_overlay(
            namespace="panther",
            robot_model="panther",
            observation_topic="/panther/velodyne_points",
            observation_topic_type="pointcloud",
        )
        == {}
    )


def test_defaults_route_velocity_as_before():
    smoother_out, behavior_out, lifecycle = mp.velocity_routing(False, "cmd_vel")
    assert (smoother_out, behavior_out) == ("cmd_vel", "cmd_vel")
    assert lifecycle == mp.LIFECYCLE_NODES
    assert "collision_monitor" not in lifecycle


def test_placeholders_render_as_replace_string_did():
    with open(PARAMS_FILE, encoding="utf-8") as f:
        text = mp.render_placeholders(f.read(), "", "panther", "/points", "pointcloud")
    assert "<" not in text.replace("<<", "")
    tree = yaml.safe_load(text)["/**"]
    assert params(tree, "bt_navigator")["global_frame"] == "map"
    assert "slam_toolbox" in tree


def test_crop_box_uses_the_named_box_schema():
    tree = rendered(
        namespace=NS,
        robot_model="panther",
        observation_topic=CLOUD,
        observation_topic_type="pointcloud",
    )
    crop = params(tree, "pointcloud_crop_box")
    box = mp.ROBOT_BOUNDING_BOX["panther"]
    assert crop["crop_boxes_names"] == ["robot_box"]
    assert crop["crop_boxes"]["robot_box"] == {k: box[k] for k in crop["crop_boxes"]["robot_box"]}
    assert set(crop["crop_boxes"]["robot_box"]) == set(box)
    assert crop["negative"] is True
    assert not {"min_x", "max_x", "min_z", "max_z"} & set(crop)


# --- the airlock-ready output --------------------------------------------------------------------


def test_collision_monitor_routing():
    smoother_out, behavior_out, lifecycle = mp.velocity_routing(True, "autonomous/cmd_vel")
    assert smoother_out == "cmd_vel_smoothed"
    assert behavior_out == "cmd_vel_nav"
    assert lifecycle[-1] == "collision_monitor"


def test_output_without_collision_monitor_still_routes_behaviors():
    smoother_out, behavior_out, lifecycle = mp.velocity_routing(False, "autonomous/cmd_vel")
    assert (smoother_out, behavior_out) == ("autonomous/cmd_vel", "cmd_vel_nav")
    assert "collision_monitor" not in lifecycle


def test_every_velocity_node_is_stamped():
    tree = rendered(**MAPNAV)
    for name in STAMPED_NODES:
        assert params(tree, name)["enable_stamped_cmd_vel"] is True, name


def test_collision_monitor_params():
    cm = params(rendered(**MAPNAV), "collision_monitor")
    assert cm["cmd_vel_in_topic"] == "cmd_vel_smoothed"
    assert cm["cmd_vel_out_topic"] == "autonomous/cmd_vel"
    assert cm["base_frame_id"] == f"{NS}/base_footprint"
    assert cm["odom_frame_id"] == f"{NS}/odom"
    assert cm["observation_sources"] == ["cloud"]
    assert cm["cloud"]["type"] == "pointcloud"
    assert cm["cloud"]["topic"] == f"{CLOUD}_filtered"
    assert cm["cloud"]["min_height"] == pytest.approx(0.1)
    top = 0.354 + mp.SENSOR_HEIGHT_ABOVE_RAIL
    assert cm["cloud"]["max_height"] == pytest.approx(top + 0.2)
    assert cm["polygons"] == ["FootprintStop", "Slowdown"]
    assert cm["FootprintStop"]["action_type"] == "stop"
    for name in cm["polygons"]:
        points = yaml.safe_load(cm[name]["points"])
        assert len(points) == 4


def test_robot_height_sets_the_obstacle_ceiling():
    tree = rendered(**dict(MAPNAV, robot_height="0.7"))
    assert params(tree, "collision_monitor")["cloud"]["max_height"] == pytest.approx(0.9)
    for costmap in ("local_costmap", "global_costmap"):
        stvl = params(tree, costmap, costmap)["stvl_layer"]["pointcloud"]
        assert stvl["max_obstacle_height"] == pytest.approx(0.9)
    # the crop box drops the whole robot, gate and payload included
    crop = params(tree, "pointcloud_crop_box")["crop_boxes"]["robot_box"]
    assert crop["max_z"] == pytest.approx(0.75)


def test_collision_monitor_on_a_laserscan_source():
    kwargs = dict(
        MAPNAV,
        observation_topic="/panther/scan",
        observation_topic_type="laserscan",
        lidar_profile="",
    )
    cm = params(rendered(**kwargs), "collision_monitor")
    assert cm["cloud"] == {"type": "scan", "topic": "/panther/scan", "enabled": True}


def test_laserscan_source_clears_the_whole_circle():
    # a 2D LiDAR sees 360 deg: STVL clears with a 3D LiDAR's frustum, not a depth camera's cone
    kwargs = dict(
        MAPNAV,
        observation_topic="/panther/scan",
        observation_topic_type="laserscan",
        lidar_profile="",
    )
    tree = rendered(**kwargs)
    for costmap in ("local_costmap", "global_costmap"):
        stvl = params(tree, costmap, costmap)["stvl_layer"]
        assert stvl["observation_sources"] == "laserscan"
        scan = stvl["laserscan"]
        assert scan["model_type"] == 1
        assert scan["horizontal_fov_angle"] == pytest.approx(6.29)
        assert scan["topic"] == "/panther/scan"


def test_os2_64_caps_speed_at_all_times():
    tree = rendered(**dict(MAPNAV, lidar_profile="os2_64"))
    follow = params(tree, "controller_server")["FollowPath"]
    assert follow["vx_max"] == pytest.approx(0.3)
    assert follow["vx_min"] == pytest.approx(-0.3)
    smoother = params(tree, "velocity_smoother")
    assert smoother["max_velocity"] == pytest.approx([0.3, 0.0, 1.0])
    assert smoother["min_velocity"] == pytest.approx([-0.3, 0.0, -1.0])
    # the cap is not a collision-monitor polygon, which would act only while it sees points
    assert "limit" not in {
        params(tree, "collision_monitor")[p]["action_type"]
        for p in params(tree, "collision_monitor")["polygons"]
    }


def test_os0_32_keeps_the_stack_speed():
    tree = rendered(**MAPNAV)
    assert params(tree, "controller_server")["FollowPath"]["vx_max"] == pytest.approx(0.8)
    assert params(tree, "velocity_smoother")["max_velocity"][0] == pytest.approx(0.8)


def test_speed_cap_never_raises_a_lower_limit():
    tree = yaml.safe_load(
        mp.apply_overlay(
            yaml.safe_dump(
                {
                    "/**": {
                        "controller_server": {
                            "ros__parameters": {
                                "controller_plugins": ["FollowPath"],
                                "FollowPath": {"vx_max": 0.2, "vx_min": -0.1},
                            }
                        },
                        "velocity_smoother": {
                            "ros__parameters": {
                                "max_velocity": [0.2, 0.0, 1.0],
                                "min_velocity": [-0.1, 0.0, -1.0],
                            }
                        },
                    }
                }
            ),
            {},
            0.3,
        )
    )["/**"]
    assert params(tree, "controller_server")["FollowPath"]["vx_max"] == pytest.approx(0.2)
    assert params(tree, "velocity_smoother")["min_velocity"][0] == pytest.approx(-0.1)


# --- the 3D-to-2D path ---------------------------------------------------------------------------


@pytest.mark.parametrize("profile,fov,range_min", [("os0_32", 1.571, 0.5), ("os2_64", 0.393, 0.8)])
def test_lidar_profile(profile, fov, range_min):
    tree = rendered(**dict(MAPNAV, lidar_profile=profile))
    for costmap in ("local_costmap", "global_costmap"):
        stvl = params(tree, costmap, costmap)["stvl_layer"]["pointcloud"]
        assert stvl["model_type"] == 1
        assert stvl["vertical_fov_angle"] == pytest.approx(fov)
        assert stvl["vertical_fov_padding"] == pytest.approx(0.05)
        assert stvl["horizontal_fov_angle"] == pytest.approx(6.29)
        assert stvl["min_obstacle_height"] == pytest.approx(0.1)
        # the rest of the source stays as it was
        assert stvl["topic"] == f"{CLOUD}_filtered"
        assert stvl["data_type"] == "PointCloud2"
    slice_ = params(tree, "pointcloud_to_laserscan")
    assert slice_["range_min"] == pytest.approx(range_min)
    assert slice_["range_max"] == pytest.approx(20.0)
    assert slice_["angle_increment"] == pytest.approx(2 * math.pi / 1024)
    assert slice_["target_frame"] == f"{NS}/base_footprint"


def test_slice_is_apart_from_the_crop_box():
    tree = rendered(**MAPNAV)
    slice_ = params(tree, "pointcloud_to_laserscan")
    assert (slice_["min_height"], slice_["max_height"]) == (0.25, 1.0)
    crop = params(tree, "pointcloud_crop_box")["crop_boxes"]["robot_box"]
    # no robot_height: the robot's top is the rail plus the sensor
    assert crop["max_z"] == pytest.approx(0.354 + mp.SENSOR_HEIGHT_ABOVE_RAIL + 0.05)
    assert crop["min_z"] == pytest.approx(0.05)


def test_slice_defaults_to_the_crop_box():
    tree = rendered(**dict(MAPNAV, scan_min_height="", scan_max_height="", lidar_profile=""))
    slice_ = params(tree, "pointcloud_to_laserscan")
    assert (slice_["min_height"], slice_["max_height"]) == (0.05, 0.5)


def test_scan_heights_default_per_type_and_explicit_values_win():
    assert mp.scan_heights("laserscan", "", "") == ("0.05", "2.0")
    assert mp.scan_heights("pointcloud", "", "") == ("0.25", "1.0")
    assert mp.scan_heights("pointcloud", "0.3", "") == ("0.3", "1.0")
    assert mp.scan_heights("laserscan", "", "1.5") == ("0.05", "1.5")


def test_lidar_profile_needs_the_mount_height():
    with pytest.raises(ValueError):
        mp.build_overlay(**dict(MAPNAV, mount_height=None))
    with pytest.raises(ValueError):
        mp.build_overlay(**dict(MAPNAV, lidar_profile="os1_128"))


def test_map_update_interval():
    tree = rendered(**MAPNAV)
    assert params(tree, f"{NS}/slam_toolbox")["map_update_interval"] == pytest.approx(5.0)
    tree = rendered(**dict(MAPNAV, map_update_interval=""))
    assert params(tree, f"{NS}/slam_toolbox")["map_update_interval"] == pytest.approx(1.0)


# --- AMCL's starting pose from the sidecar -------------------------------------------------------


def write_pair(tmp_path, saved_pose=None, sha=None, with_sidecar=True):
    image = tmp_path / "map.png"
    image.write_bytes(b"\x89PNG fake map")
    (tmp_path / "map.yaml").write_text(
        "image: map.png\nresolution: 0.05\norigin: [0.0, 0.0, 0.0]\n"
        "negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.25\n"
    )
    if with_sidecar:
        sidecar = {
            "schema": 1,
            "epoch": "0123456789abcdef",
            "map_sha256": sha or hashlib.sha256(image.read_bytes()).hexdigest(),
        }
        if saved_pose is not None:
            sidecar["saved_pose"] = saved_pose
        (tmp_path / "map.georef.yaml").write_text(yaml.safe_dump(sidecar))
    return str(tmp_path / "map.yaml")


def test_saved_pose_is_read(tmp_path):
    map_yaml = write_pair(tmp_path, {"x": 1.5, "y": -2.0, "yaw": 0.75})
    assert mp.read_saved_pose(map_yaml) == (1.5, -2.0, 0.75)
    tree = rendered(**dict(MAPNAV, initial_pose=(1.5, -2.0, 0.75)))
    amcl = params(tree, "amcl")
    assert amcl["set_initial_pose"] is True
    assert amcl["initial_pose"] == {"x": 1.5, "y": -2.0, "z": 0.0, "yaw": 0.75}


@pytest.mark.parametrize(
    "kwargs",
    [
        dict(with_sidecar=False),
        dict(saved_pose=None),
        dict(saved_pose={"x": 1.0, "y": 2.0}),
        dict(saved_pose={"x": "nan", "y": 0.0, "yaw": 0.0}),
        dict(saved_pose={"x": 1.0, "y": 2.0, "yaw": 0.0}, sha="0" * 64),
    ],
)
def test_saved_pose_refuses(tmp_path, kwargs):
    map_yaml = write_pair(tmp_path, **kwargs)
    with pytest.raises(ValueError):
        mp.read_saved_pose(map_yaml)


def test_sidecar_path():
    assert mp.sidecar_path("/maps/hall/map.yaml") == "/maps/hall/map.georef.yaml"
