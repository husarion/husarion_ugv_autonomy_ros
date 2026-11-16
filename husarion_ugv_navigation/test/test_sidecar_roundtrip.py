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

"""The saved pair, written by the gateway and read back (SPEC-MAP-NAV-2026-11 §5.5, §11.2).

Nav2's real map_server and map_saver_server, the map gateway and a fake robot whose pose in the
map frame is known. The gateway's serialize_map saves a pair; R1's Python reader
(`mapnav_params.read_saved_pose`, what the launch uses for AMCL's initial pose) accepts it and
gets the robot's pose back. A second gateway started on that pair takes the sidecar's epoch.
"""

import math
import os
import subprocess
import sys
import tempfile
import time
import unittest

import launch
import launch_testing
import launch_testing.actions
import pytest
import rclpy
import yaml
from ament_index_python.packages import get_package_prefix, get_package_share_directory
from diagnostic_msgs.msg import DiagnosticStatus
from husarion_ugv_navigation.mapnav_params import read_saved_pose, sidecar_path
from launch_ros.actions import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from slam_toolbox.srv import SerializePoseGraph

NS = "mapnav_roundtrip"
NS_RELOAD = "mapnav_roundtrip_reload"
MAPS = tempfile.mkdtemp(prefix="mapnav_roundtrip_")
# The fake robot stands at its odom origin; odom sits here in the map frame.
MAP_TO_ODOM = (2.0, -1.0, 0.5)


@pytest.mark.launch_test
def generate_test_description():
    share = get_package_share_directory("husarion_ugv_navigation")
    world = os.path.join(share, "map", "husarion_world.yaml")
    x, y, yaw = MAP_TO_ODOM
    return launch.LaunchDescription(
        [
            launch.actions.ExecuteProcess(
                cmd=[sys.executable, os.path.join(os.path.dirname(__file__), "fake_robot.py"), NS],
                output="screen",
            ),
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                arguments=[
                    "--x",
                    str(x),
                    "--y",
                    str(y),
                    "--yaw",
                    str(yaw),
                    "--frame-id",
                    f"{NS}/map",
                    "--child-frame-id",
                    f"{NS}/odom",
                ],
            ),
            Node(
                package="nav2_map_server",
                executable="map_server",
                namespace=NS,
                parameters=[{"yaml_filename": world, "frame_id": f"{NS}/map"}],
                output="screen",
            ),
            Node(
                package="nav2_map_server",
                executable="map_saver_server",
                name="map_saver",
                namespace=NS,
                output="screen",
            ),
            Node(
                package="nav2_lifecycle_manager",
                executable="lifecycle_manager",
                name="lifecycle_manager_map",
                namespace=NS,
                parameters=[{"autostart": True, "node_names": ["map_server", "map_saver"]}],
                output="screen",
            ),
            Node(
                package="husarion_ugv_map_gateway",
                executable="map_gateway",
                name="map_gateway",
                namespace=NS,
                parameters=[{"maps_directory": MAPS}],
                output="screen",
            ),
            launch_testing.actions.ReadyToTest(),
        ]
    )


class TestSidecarRoundTrip(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node("mapnav_roundtrip_test")
        cls.states = {}
        latched = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        for ns in (NS, NS_RELOAD):
            cls.node.create_subscription(
                DiagnosticStatus,
                f"/{ns}/cockpit/nav/state",
                lambda msg, ns=ns: cls.states.__setitem__(
                    ns, {kv.key: kv.value for kv in msg.values}
                ),
                latched,
            )

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def spin_until(self, done, timeout):
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            rclpy.spin_once(self.node, timeout_sec=0.05)
            if done():
                return True
        return done()

    def save(self, name):
        client = self.node.create_client(SerializePoseGraph, f"/{NS}/map_gateway/serialize_map")
        self.assertTrue(client.wait_for_service(timeout_sec=60.0), "no serialize_map")
        # The map saver needs the map and the gateway the robot's pose: retry while they come up.
        result = None
        end = time.monotonic() + 60.0
        while time.monotonic() < end:
            future = client.call_async(SerializePoseGraph.Request(filename=name))
            rclpy.spin_until_future_complete(self.node, future, timeout_sec=40.0)
            result = future.result()
            if result is not None and result.result == SerializePoseGraph.Response.RESULT_SUCCESS:
                return result
            self.spin_until(lambda: False, 2.0)
        return result

    def test_1_saved_pair_round_trip(self):
        result = self.save("roundtrip")
        self.assertIsNotNone(result)
        self.assertEqual(result.result, SerializePoseGraph.Response.RESULT_SUCCESS)
        map_yaml = os.path.join(MAPS, "roundtrip", "map.yaml")
        self.assertTrue(os.path.isfile(map_yaml))
        self.assertTrue(os.path.isfile(sidecar_path(map_yaml)))

        # R1's reader, the one bringup_launch.py feeds AMCL's initial pose from.
        x, y, yaw = read_saved_pose(map_yaml)
        ex, ey, eyaw = MAP_TO_ODOM
        self.assertAlmostEqual(x, ex, places=3)
        self.assertAlmostEqual(y, ey, places=3)
        self.assertAlmostEqual(math.remainder(yaw - eyaw, 2 * math.pi), 0.0, places=3)

        with open(sidecar_path(map_yaml), encoding="utf-8") as f:
            sidecar = yaml.safe_load(f)
        self.assertEqual(sidecar["schema"], 1)
        self.assertTrue(self.spin_until(lambda: NS in self.states, 10.0), "no state")
        self.assertEqual(sidecar["epoch"], self.states[NS]["epoch"])
        self.assertNotIn("lat0", sidecar)  # saved before any fit: no alignment
        self.assertIsInstance(sidecar["epoch"], str)

        # The name is never written over.
        again = self.save_once("roundtrip")
        self.assertEqual(again.result, SerializePoseGraph.Response.RESULT_FAILED_TO_WRITE_FILE)

        # A gateway on that pair takes the sidecar's epoch (mode amcl, no alignment).
        exe = os.path.join(
            get_package_prefix("husarion_ugv_map_gateway"),
            "lib",
            "husarion_ugv_map_gateway",
            "map_gateway",
        )
        reload_dir = tempfile.mkdtemp(prefix="mapnav_reload_")
        proc = subprocess.Popen(
            [
                exe,
                "--ros-args",
                "-r",
                f"__ns:=/{NS_RELOAD}",
                "-p",
                f"map_file_name:={map_yaml}",
                "-p",
                f"maps_directory:={reload_dir}",
            ]
        )
        try:
            self.assertTrue(
                self.spin_until(lambda: NS_RELOAD in self.states, 30.0), "no state from reload"
            )
            state = self.states[NS_RELOAD]
            self.assertEqual(state["epoch"], sidecar["epoch"])
            self.assertEqual(state["mode"], "amcl")
            self.assertEqual(state["align"], "not_aligned")
        finally:
            proc.terminate()
            proc.wait(timeout=10)

    def save_once(self, name):
        client = self.node.create_client(SerializePoseGraph, f"/{NS}/map_gateway/serialize_map")
        self.assertTrue(client.wait_for_service(timeout_sec=10.0))
        future = client.call_async(SerializePoseGraph.Request(filename=name))
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=40.0)
        return future.result()

    def test_2_names_are_confined(self):
        for bad in ("../escape", "a/b", "..", ""):
            result = self.save_once(bad)
            self.assertIsNotNone(result, bad)
            self.assertEqual(
                result.result, SerializePoseGraph.Response.RESULT_FAILED_TO_WRITE_FILE, bad
            )
        self.assertFalse(os.path.exists(os.path.join(os.path.dirname(MAPS), "escape")))
