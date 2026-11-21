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

"""Launch gate of the airlock-ready stack (SPEC-MAP-NAV-2026-11 R1, §11.6).

The real bringup with the map-nav arguments, against a standing fake robot (test/fake_robot.py):
- the collision monitor is in Nav2's lifecycle list and active;
- `<ns>/autonomous/cmd_vel` has exactly one publisher, the collision monitor, as TwistStamped, and
  nothing publishes `<ns>/cmd_vel`;
- a command through the velocity smoother arrives there stamped with a live stamp;
- an obstacle in the stop polygon (from the 3D cloud through the crop box) stops it, and so does a
  table edge at 0.8 m, below the robot's top (robot_height 0.7 m) plus 0.2 m;
- the crop box drops the robot's own body, a DEV09 gate included, and keeps the obstacle;
- with use_map_gateway the map gateway reports Nav2 and the collision monitor active, sends the
  map, and its pause and resume triggers take the real Nav2 down and up (spec R2, §11.6);
- the gateway sends the real local costmap, placed in the map frame through AMCL's map -> odom, as
  a PNG under 32 768 B with its paired info (spec R26);
- the gateway sends the slice's scan at about 4 Hz, at most 720 finite ranges and no intensities,
  under 8192 B, in the slice's own frame (M2a).
"""

import math
import os
import sys
import tempfile
import time
import unittest

import launch
import launch_testing
import launch_testing.actions
import pytest
import rclpy
from ament_index_python.packages import get_package_share_directory
from diagnostic_msgs.msg import DiagnosticStatus
from geometry_msgs.msg import Point, TwistStamped
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from nav_msgs.msg import MapMetaData
from rclpy.qos import (
    DurabilityPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from rclpy.serialization import serialize_message
from sensor_msgs.msg import CompressedImage, LaserScan, PointCloud2
from sensor_msgs_py import point_cloud2
from std_srvs.srv import Trigger

NS = "mapnav_test"
LIDAR_Z = 0.1825 + 0.14 + 0.0315 + 0.036  # os_lidar above base_footprint in fake_robot.py
ACTIVE_TIMEOUT_S = 180.0


@pytest.mark.launch_test
def generate_test_description():
    share = get_package_share_directory("husarion_ugv_navigation")
    fake_robot = launch.actions.ExecuteProcess(
        cmd=[sys.executable, os.path.join(os.path.dirname(__file__), "fake_robot.py"), NS],
        output="screen",
    )
    bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(share, "launch", "bringup_launch.py")),
        launch_arguments={
            "namespace": NS,
            "observation_topic": f"/{NS}/points",
            "observation_topic_type": "pointcloud",
            "slam": "False",
            "map": os.path.join(share, "map", "husarion_world.yaml"),
            "robot_model": "panther",
            "use_sim_time": "False",
            "cmd_vel_out": "autonomous/cmd_vel",
            "use_collision_monitor": "True",
            "lidar_profile": "os0_32",
            # mount_height left empty: the launch reads it from the fake robot's TF
            "robot_height": "0.7",
            "scan_min_height": "0.25",
            "scan_max_height": "1.0",
            "map_update_interval": "5.0",
            "use_map_gateway": "True",
            "maps_directory": tempfile.mkdtemp(prefix="mapnav_launch_maps_"),
        }.items(),
    )
    return launch.LaunchDescription(
        [
            fake_robot,
            # the TF lookup in the bringup blocks the launch loop; start the fake robot first
            TimerAction(period=3.0, actions=[bringup]),
            launch_testing.actions.ReadyToTest(),
        ]
    )


class TestMapNavStack(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node("mapnav_launch_test")
        cls.obstacle_pub = cls.node.create_publisher(Point, f"/{NS}/fake_obstacle", 10)
        cls.cmd_pub = cls.node.create_publisher(TwistStamped, f"/{NS}/cmd_vel_nav", 10)
        cls.out = []
        cls.node.create_subscription(
            TwistStamped, f"/{NS}/autonomous/cmd_vel", cls.on_out, qos_profile_sensor_data
        )
        cls.clouds = []
        cls.node.create_subscription(
            PointCloud2, f"/{NS}/points_filtered", cls.clouds.append, qos_profile_sensor_data
        )
        latched = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        cls.state = {}
        cls.node.create_subscription(
            DiagnosticStatus,
            f"/{NS}/cockpit/nav/state",
            lambda m: cls.state.update({kv.key: kv.value for kv in m.values}),
            latched,
        )
        cls.pngs = []
        cls.infos = []
        cls.node.create_subscription(
            CompressedImage, f"/{NS}/cockpit/nav/map/png", cls.pngs.append, latched
        )
        cls.node.create_subscription(
            MapMetaData, f"/{NS}/cockpit/nav/map/info", cls.infos.append, latched
        )
        cls.costmap_pngs = []
        cls.costmap_infos = []
        cls.scans = []
        cls.node.create_subscription(
            LaserScan,
            f"/{NS}/cockpit/nav/scan",
            lambda m: cls.scans.append((time.monotonic(), m)),
            qos_profile_sensor_data,
        )
        cls.node.create_subscription(
            CompressedImage,
            f"/{NS}/cockpit/nav/costmap/png",
            cls.costmap_pngs.append,
            qos_profile_sensor_data,
        )
        cls.node.create_subscription(
            MapMetaData,
            f"/{NS}/cockpit/nav/costmap/info",
            cls.costmap_infos.append,
            qos_profile_sensor_data,
        )
        cls.wait_active()

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    @classmethod
    def on_out(cls, msg):
        age = (cls.node.get_clock().now() - rclpy.time.Time.from_msg(msg.header.stamp)).nanoseconds
        cls.out.append((msg.twist.linear.x, age / 1e9))

    @classmethod
    def spin_for(cls, seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(cls.node, timeout_sec=0.02)

    @classmethod
    def wait_active(cls):
        client = cls.node.create_client(Trigger, f"/{NS}/lifecycle_manager_navigation/is_active")
        deadline = time.monotonic() + ACTIVE_TIMEOUT_S
        while time.monotonic() < deadline:
            if client.wait_for_service(timeout_sec=1.0):
                future = client.call_async(Trigger.Request())
                rclpy.spin_until_future_complete(cls.node, future, timeout_sec=2.0)
                if future.result() is not None and future.result().success:
                    return
            cls.spin_for(1.0)
        raise AssertionError(f"Nav2 was not active within {ACTIVE_TIMEOUT_S:.0f} s")

    def set_obstacle(self, x, y=0.0, z=0.0):
        self.obstacle_pub.publish(Point(x=float(x), y=float(y), z=float(z)))
        self.spin_for(1.5)  # a few clouds through the crop box into the monitor

    def drive(self, x, seconds, obstacle_at=None, obstacle=None):
        """Send `x` m/s into the velocity smoother for `seconds`; return what came out.

        With `obstacle_at`, the fake obstacle `obstacle` (x, y, z) appears that many seconds in.
        """
        self.out.clear()
        start = time.monotonic()
        placed = False
        while time.monotonic() < start + seconds:
            if obstacle_at is not None and not placed and time.monotonic() >= start + obstacle_at:
                ox, oy, oz = obstacle
                self.obstacle_pub.publish(Point(x=float(ox), y=float(oy), z=float(oz)))
                placed = True
            cmd = TwistStamped()
            cmd.header.stamp = self.node.get_clock().now().to_msg()
            cmd.header.frame_id = f"{NS}/base_link"
            cmd.twist.linear.x = float(x)
            self.cmd_pub.publish(cmd)
            self.spin_for(0.05)
        return list(self.out)

    def assert_stops(self, obstacle):
        """Driving freely, then `obstacle` appears: the output falls to zero and stays there.

        (Once stopped, the collision monitor sends zeros for stop_pub_timeout and then goes
        quiet, so the test starts from motion.)
        """
        self.set_obstacle(-1000.0)
        out = self.drive(0.4, 4.0, obstacle_at=1.5, obstacle=obstacle)
        self.set_obstacle(-1000.0)
        self.assertTrue(out, "nothing arrived on autonomous/cmd_vel")
        xs = [x for x, _ in out]
        self.assertGreater(max(xs), 0.1, "it never drove before the obstacle")
        tail = xs[-10:]
        self.assertTrue(all(abs(x) < 1e-6 for x in tail), f"did not stop: {tail}")

    def test_1_output_wiring(self):
        pubs = self.node.get_publishers_info_by_topic(f"/{NS}/autonomous/cmd_vel")
        self.assertEqual([p.node_name for p in pubs], ["collision_monitor"])
        self.assertEqual(pubs[0].topic_type, "geometry_msgs/msg/TwistStamped")
        self.assertEqual(self.node.get_publishers_info_by_topic(f"/{NS}/cmd_vel"), [])
        # the behaviors join the controller in front of the smoother
        behaviors = self.node.get_publishers_info_by_topic(f"/{NS}/cmd_vel_nav")
        self.assertTrue(any(p.node_name == "behavior_server" for p in behaviors))

    def test_2_drives_with_a_live_stamp(self):
        self.set_obstacle(-1000.0)
        out = self.drive(0.4, 3.0)
        self.assertTrue(out, "nothing arrived on autonomous/cmd_vel")
        self.assertGreater(max(x for x, _ in out), 0.3)
        ages = [age for _, age in out]
        self.assertLess(max(abs(a) for a in ages), 0.2, f"stale or future stamps: {ages[:5]}")

    def test_3_stops_for_an_obstacle(self):
        self.assert_stops((0.5, 0.0, 0.0))

    def test_5_stops_for_an_overhang(self):
        # a table edge at 0.8 m: above the slice's body, below robot_height + 0.2 m
        self.assert_stops((0.5, 0.0, 0.8))

    def test_4_crop_box_drops_the_body(self):
        self.set_obstacle(1.0)
        self.clouds.clear()
        self.spin_for(1.0)
        clouds = list(self.clouds)
        self.set_obstacle(-1000.0)
        self.assertTrue(clouds, "no filtered cloud")
        cloud = clouds[-1]
        # back to base_footprint: the cloud is in os_lidar (180 deg yaw, LIDAR_Z up)
        pts = [
            (-float(p[0]), -float(p[1]), float(p[2]) + LIDAR_Z)
            for p in point_cloud2.read_points(cloud, field_names=("x", "y", "z"), skip_nans=True)
        ]
        body = [p for p in pts if abs(p[0]) < 0.4 and abs(p[1]) < 0.4 and p[2] < 0.7]
        obstacle = [p for p in pts if math.hypot(p[0] - 1.0, p[1]) < 0.2]
        self.assertEqual(body, [])
        self.assertGreater(len(obstacle), 10)

    def wait_state(self, key, value, timeout):
        end = time.monotonic() + timeout
        while time.monotonic() < end and self.state.get(key) != value:
            self.spin_for(0.2)
        self.assertEqual(self.state.get(key), value, f"state: {self.state}")

    def trigger(self, name):
        client = self.node.create_client(Trigger, f"/{NS}/cockpit/nav/{name}")
        self.assertTrue(client.wait_for_service(timeout_sec=10.0), f"no {name} trigger")
        future = client.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=30.0)
        self.assertIsNotNone(future.result(), f"{name} did not answer")
        return future.result()

    def test_6_map_gateway(self):
        self.wait_state("nav2", "active", 20.0)
        self.wait_state("collision_monitor", "active", 20.0)
        self.assertEqual(self.state.get("frame"), f"{NS}/map")
        self.assertEqual(self.state.get("mode"), "amcl")
        # husarion_world.yaml has no sidecar: a new epoch, not aligned
        self.assertEqual(self.state.get("align"), "not_aligned")
        self.assertEqual(len(self.state.get("epoch", "")), 16)
        end = time.monotonic() + 20.0
        while time.monotonic() < end and not (self.pngs and self.infos):
            self.spin_for(0.2)
        self.assertTrue(self.pngs and self.infos, "no map from the gateway")
        self.assertEqual(self.pngs[-1].header.stamp, self.infos[-1].map_load_time)
        self.assertLessEqual(len(self.pngs[-1].data), 190000)
        # fromLL and toLL are not served without an alignment (waypoint_follower's /fromLL
        # client is in the graph too, so ask for the gateway's own servers)
        served = [
            n
            for n, _ in self.node.get_service_names_and_types_by_node("map_gateway", f"/{NS}")
            if n.endswith("LL")
        ]
        self.assertEqual(served, [])
        gateway_services = [
            n for n, _ in self.node.get_service_names_and_types_by_node("map_gateway", f"/{NS}")
        ]
        self.assertTrue(f"/{NS}/cockpit/nav/pause" in gateway_services, gateway_services)
        # Pause and resume through the gateway's triggers, against the real lifecycle manager.
        self.assertTrue(self.trigger("pause").success)
        self.wait_state("nav2", "inactive", 15.0)
        self.wait_state("collision_monitor", "inactive", 15.0)
        self.assertTrue(self.trigger("resume").success)
        self.wait_state("nav2", "active", 30.0)
        self.wait_state("collision_monitor", "active", 15.0)

    def test_7_local_costmap(self):
        end = time.monotonic() + 30.0
        while time.monotonic() < end and not (self.costmap_pngs and self.costmap_infos):
            self.spin_for(0.2)
        self.assertTrue(
            self.costmap_pngs and self.costmap_infos, "no local costmap from the gateway"
        )
        png = self.costmap_pngs[-1]
        stamps = [i.map_load_time for i in self.costmap_infos]
        self.assertTrue(png.header.stamp in stamps)  # the pairing rule
        info = next(i for i in self.costmap_infos if i.map_load_time == png.header.stamp)
        self.assertEqual(png.format, "png")
        self.assertEqual(png.header.frame_id, f"{NS}/map")
        self.assertLessEqual(len(png.data), 32768)
        self.assertEqual(bytes(png.data[:8]), b"\x89PNG\r\n\x1a\n")
        # nav2_params.yaml: 8 m at 2 cm, so 400 x 400 unless pooled
        self.assertLessEqual(max(info.width, info.height), 400)
        self.assertGreater(info.width, 0)
        for v in (
            info.resolution,
            info.origin.position.x,
            info.origin.position.y,
            info.origin.orientation.z,
            info.origin.orientation.w,
        ):
            self.assertTrue(math.isfinite(v))
        # the robot stands at the window's centre: the origin is about half the window away
        half = info.width * info.resolution / 2.0
        self.assertLess(
            math.hypot(info.origin.position.x, info.origin.position.y), 1.5 * half + 0.5
        )
        print(
            f"local costmap: {info.width} x {info.height} at {info.resolution:.3f} m, "
            f"{len(png.data)} B, origin ({info.origin.position.x:.2f}, "
            f"{info.origin.position.y:.2f})"
        )

    def test_8_scan(self):
        self.spin_for(1.0)
        self.scans.clear()
        self.spin_for(4.0)
        self.assertTrue(self.scans, "no scan from the gateway")
        times = [t for t, _ in self.scans]
        rate = (len(times) - 1) / (times[-1] - times[0]) if len(times) > 1 else 0.0
        self.assertGreater(rate, 2.5)
        self.assertLess(rate, 5.0)
        scan = self.scans[-1][1]
        size = len(serialize_message(scan))
        self.assertEqual(scan.header.frame_id, f"{NS}/base_footprint")  # the slice's target_frame
        self.assertLessEqual(len(scan.ranges), 720)
        self.assertEqual(list(scan.intensities), [])
        self.assertTrue(all(math.isfinite(r) for r in scan.ranges))
        self.assertLessEqual(size, 8192)
        print(
            f"scan: {len(scan.ranges)} ranges in {scan.header.frame_id}, {size} B, "
            f"{rate:.1f} Hz"
        )
