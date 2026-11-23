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

import os
import threading
import time

import rclpy
from geometry_msgs.msg import TransformStamped
from husarion_ugv_navigation.preflight import (
    Check,
    Preflight,
    mount_height_of,
    other_robots,
    report,
    waiting_message,
)
from tf2_ros import StaticTransformBroadcaster


def test_other_robots_lists_base_links_under_another_namespace():
    frames = {"lynx_d6b2/base_link", "lynx_d6b2/odom", "panther_0957/base_link", "base_link"}
    assert other_robots(frames, "lynx_d6b2/") == {"panther_0957", "(none)"}
    assert other_robots(frames, "") == {"lynx_d6b2", "panther_0957"}
    assert other_robots({"lynx_d6b2/mount_link"}, "") == set()


def test_waiting_message_names_another_robot_only_when_one_is_seen():
    assert "ROBOT_NAMESPACE" not in waiting_message("lynx_d6b2", 10.0, set())
    text = waiting_message("lynx_d6b2", 20.0, {"panther_0957"})
    assert "robot lynx_d6b2 for 20 s" in text
    assert "panther_0957" in text and "ROBOT_NAMESPACE" in text


def test_report_lists_every_check_and_the_fix_of_each_failure_only():
    checks = [
        Check("driver TF", True, "ok", fix="never shown"),
        Check("lidar", False, "nothing publishes /x", fix="plug it in"),
    ]
    text = report(checks, "", 30.0)
    assert "namespace (none), timeout 30 s" in text
    assert "[ok  ] driver TF" in text and "[FAIL] lidar" in text
    assert "--- lidar ---\nplug it in" in text
    assert "never shown" not in text


def test_mount_height_comes_from_a_passed_mount_check_only():
    assert mount_height_of([Check("mount TF", True, "", value=0.366)]) == 0.366
    assert mount_height_of([Check("mount TF", False, "", value=0.366)]) is None
    assert mount_height_of([]) is None


def test_the_wait_has_no_deadline_and_ends_when_the_robot_appears():
    def robot_after(delay_s, done):
        time.sleep(delay_s)
        context = rclpy.Context()
        rclpy.init(context=context)
        node = rclpy.create_node("late_robot", context=context)
        t = TransformStamped()
        t.header.frame_id, t.child_frame_id = "late/odom", "late/base_link"
        t.transform.rotation.w = 1.0
        StaticTransformBroadcaster(node).sendTransform(t)
        done.wait()
        node.destroy_node()
        rclpy.shutdown(context=context)

    done = threading.Event()
    robot = threading.Thread(target=robot_after, args=(2.0, done))
    robot.start()
    logs = []
    preflight = Preflight("late", "", "pointcloud", "", timeout=1.0, log=logs.append)
    try:
        start = time.monotonic()
        preflight.wait_for_robot(log_period_s=0.5)
        waited = time.monotonic() - start
    finally:
        preflight.close()
        done.set()
        robot.join()
    assert waited >= 1.5
    assert any("waiting for robot late" in line for line in logs)
    assert logs[-1].startswith("preflight: the robot is here after")


def _lazy_bridge(topic, done):
    """Subscribes to `topic` only once a publisher appears on it, like the robot's airlock."""
    from geometry_msgs.msg import TwistStamped

    context = rclpy.Context()
    rclpy.init(context=context)
    node = rclpy.create_node("preflight_test_lazy_bridge", context=context)
    sub = None
    while not done.is_set():
        if sub is None and node.count_publishers(topic) > 0:
            sub = node.create_subscription(TwistStamped, topic, lambda _m: None, 1)
        time.sleep(0.1)
    node.destroy_node()
    rclpy.shutdown(context=context)


def test_velocity_sink_probes_a_lazy_bridge_and_fails_with_no_listener():
    suffix = f"{os.getpid()}_{int(time.monotonic() * 1000)}"
    lazy_topic = f"/preflight_test_{suffix}/lazy/cmd_vel"
    none_topic = f"/preflight_test_{suffix}/none/cmd_vel"
    done = threading.Event()
    bridge = threading.Thread(target=_lazy_bridge, args=(lazy_topic, done))
    bridge.start()
    lazy = Preflight("lazy", "", "pointcloud", lazy_topic, timeout=1.0)
    none = Preflight("none", "", "pointcloud", none_topic, timeout=1.0)
    try:
        behind = lazy.check_cmd_vel_sink()
        bare = none.check_cmd_vel_sink()
    finally:
        lazy.close()
        none.close()
        done.set()
        bridge.join()
    assert behind.ok, behind.detail
    assert not bare.ok and "nothing subscribes" in bare.detail
