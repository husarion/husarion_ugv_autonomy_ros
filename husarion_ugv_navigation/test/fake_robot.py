#!/usr/bin/env python3
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

"""A standing robot with a 3D LiDAR, for the map-nav launch test (no simulator needed).

Publishes, under the namespace given as the first argument:
- static TF base_link -> base_footprint, mount_link and the Ouster's os_lidar (180 deg yaw, as the
  Ouster driver publishes it), prefixed `<ns>/`, on /tf_static;
- odom -> base_link (identity) on /tf and `odometry/filtered`, 20 Hz;
- a subscriber on `autonomous/cmd_vel` created only once a publisher appears there, as the
  robot's airlock does (the preflight checks it);
- a PointCloud2 on `points` in `<ns>/os_lidar`, 10 Hz, best effort: a ring wall at 6 m between
  0.2 m and 1.2 m, a few points on the robot's own body, and an obstacle column whose place in
  base_footprint is set by `fake_obstacle` (geometry_msgs/Point; x <= -100 removes it; z > 0 makes
  it an overhang, a 0.4 m square slab at height z only, like a table edge).
"""

import math
import sys

import rclpy
from geometry_msgs.msg import Point, TransformStamped, TwistStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSProfile, qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster

WHEEL_RADIUS = 0.1825  # Panther WH01
MOUNT_ABOVE_BASE_LINK = 0.14 + 0.0315  # body -> cover -> mount (rail)
LIDAR_ABOVE_MOUNT = 0.036
LIDAR_Z = WHEEL_RADIUS + MOUNT_ABOVE_BASE_LINK + LIDAR_ABOVE_MOUNT  # above base_footprint


def transform(parent, child, x=0.0, y=0.0, z=0.0, yaw=0.0):
    t = TransformStamped()
    t.header.frame_id = parent
    t.child_frame_id = child
    t.transform.translation.x = x
    t.transform.translation.y = y
    t.transform.translation.z = z
    t.transform.rotation.z = math.sin(yaw / 2.0)
    t.transform.rotation.w = math.cos(yaw / 2.0)
    return t


class FakeRobot(Node):
    def __init__(self, ns):
        super().__init__("fake_robot", namespace=ns)
        self.prefix = f"{ns}/"
        self.obstacle = None
        self.static = StaticTransformBroadcaster(self)
        now = self.get_clock().now().to_msg()
        static_tfs = [
            transform(self.prefix + "base_link", self.prefix + "base_footprint", z=-WHEEL_RADIUS),
            transform(
                self.prefix + "base_link", self.prefix + "mount_link", z=MOUNT_ABOVE_BASE_LINK
            ),
            transform(
                self.prefix + "mount_link",
                self.prefix + "os_lidar",
                z=LIDAR_ABOVE_MOUNT,
                yaw=math.pi,
            ),
        ]
        for t in static_tfs:
            t.header.stamp = now
        self.static.sendTransform(static_tfs)
        self.tf = TransformBroadcaster(self)
        self.odom_pub = self.create_publisher(Odometry, "odometry/filtered", QoSProfile(depth=10))
        self.cloud_pub = self.create_publisher(PointCloud2, "points", qos_profile_sensor_data)
        self.create_subscription(Point, "fake_obstacle", self.on_obstacle, 10)
        self.cmd_vel_sink = None
        self.create_timer(0.5, self.on_cmd_vel_discovery)
        self.create_timer(0.05, self.on_odom)
        self.create_timer(0.1, self.on_cloud)
        self.wall = [
            (6.0 * math.cos(a), 6.0 * math.sin(a), z)
            for a in [i * 2.0 * math.pi / 720 for i in range(720)]
            for z in (0.2, 0.5, 0.8, 1.2)
        ]
        # points on the robot's own body inside the footprint: the crop box must drop them
        self.body = [(x, y, 0.3) for x in (-0.3, 0.0, 0.3) for y in (-0.3, 0.0, 0.3)]
        # a DEV09 sensor gate: four posts from the rail to 0.25 m above it and a top profile,
        # inside the footprint; the crop box must drop them too
        self.body += [
            (x, y, 0.354 + z)
            for x in (-0.26, 0.26)
            for y in (-0.12, 0.12)
            for z in (0.05, 0.1, 0.15, 0.2, 0.25)
        ]
        self.body += [(x / 10.0, y, 0.6) for x in range(-2, 3) for y in (-0.12, 0.12)]

    def on_cmd_vel_discovery(self):
        if self.cmd_vel_sink is None and self.count_publishers("autonomous/cmd_vel"):
            self.cmd_vel_sink = self.create_subscription(
                TwistStamped, "autonomous/cmd_vel", lambda _: None, 10
            )

    def on_obstacle(self, msg):
        self.obstacle = None if msg.x <= -100.0 else (msg.x, msg.y, msg.z)

    def on_odom(self):
        stamp = self.get_clock().now().to_msg()
        t = transform(self.prefix + "odom", self.prefix + "base_link")
        t.header.stamp = stamp
        self.tf.sendTransform(t)
        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = self.prefix + "odom"
        odom.child_frame_id = self.prefix + "base_link"
        odom.pose.pose.orientation.w = 1.0
        self.odom_pub.publish(odom)

    def on_cloud(self):
        points = list(self.wall) + list(self.body)
        if self.obstacle:
            ox, oy, oz = self.obstacle
            heights = (oz,) if oz > 0.0 else (0.15, 0.3, 0.45)
            points += [
                (ox + dx, oy + dy, z)
                for dx in (-0.1, -0.05, 0.0, 0.05, 0.1)
                for dy in (-0.2, -0.1, -0.05, 0.0, 0.05, 0.1, 0.2)
                for z in heights
            ]
        # base_footprint -> os_lidar: up LIDAR_Z, then a 180 deg yaw
        lidar = [(-x, -y, z - LIDAR_Z) for x, y, z in points]
        header = Header(stamp=self.get_clock().now().to_msg(), frame_id=self.prefix + "os_lidar")
        self.cloud_pub.publish(point_cloud2.create_cloud_xyz32(header, lidar))


def main():
    rclpy.init(args=sys.argv)
    ns = sys.argv[1] if len(sys.argv) > 1 and not sys.argv[1].startswith("--") else "panther"
    rclpy.spin(FakeRobot(ns))


if __name__ == "__main__":
    main()
