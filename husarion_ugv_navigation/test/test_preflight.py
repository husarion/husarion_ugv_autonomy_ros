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
from rclpy.executors import SingleThreadedExecutor
from sensor_msgs.msg import LaserScan, PointCloud2
from std_msgs.msg import String
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


def sensor(ns, done, robot=True, placed=True):
    """A LaserScan at 10 Hz on /<ns>/scan in <ns>/laser, a silent cloud and a string topic."""
    context = rclpy.Context()
    rclpy.init(context=context)
    node = rclpy.create_node("sensor", context=context)
    static = []
    for parent, child, wanted in (
        (f"{ns}/odom", f"{ns}/base_link", robot),
        (f"{ns}/base_link", f"{ns}/laser", placed),
    ):
        if wanted:
            t = TransformStamped()
            t.header.frame_id, t.child_frame_id = parent, child
            t.transform.rotation.w = 1.0
            static.append(t)
    if static:
        StaticTransformBroadcaster(node).sendTransform(static)
    scan_pub = node.create_publisher(LaserScan, f"/{ns}/scan", 10)
    node.create_publisher(PointCloud2, f"/{ns}/points", 10)
    node.create_publisher(String, f"/{ns}/other", 10)
    scan = LaserScan()
    scan.header.frame_id = f"{ns}/laser"
    node.create_timer(0.1, lambda: scan_pub.publish(scan))
    executor = SingleThreadedExecutor(context=context)
    executor.add_node(node)
    while not done.is_set():
        executor.spin_once(timeout_sec=0.05)
    node.destroy_node()
    rclpy.shutdown(context=context)


def with_sensor(ns, run, **kwargs):
    done = threading.Event()
    thread = threading.Thread(target=sensor, args=(ns, done), kwargs=kwargs)
    thread.start()
    try:
        return run()
    finally:
        done.set()
        thread.join()


def observation_check(ns, topic, wanted, timeout=2.0):
    preflight = Preflight(ns, topic, wanted, "", timeout=timeout)
    try:
        return preflight.observation_summary(preflight.run_observation(wanted))
    finally:
        preflight.close()


def test_list_observation_topics_keeps_scans_and_clouds_sorted():
    def run():
        preflight = Preflight("", "", "pointcloud", "", timeout=2.0)
        try:
            return preflight.observation_topics()
        finally:
            preflight.close()

    topics = [t for t in with_sensor("lst", run) if t["name"].startswith("/lst/")]
    assert topics == [
        {"name": "/lst/points", "type": "pointcloud"},
        {"name": "/lst/scan", "type": "laserscan"},
    ]


def test_observation_check_takes_the_type_from_the_graph_and_measures_the_rate():
    result = with_sensor("obs", lambda: observation_check("obs", "/obs/scan", "auto"))
    assert result["ok"], result
    assert (result["type"], result["frame_id"]) == ("laserscan", "obs/laser")
    assert 7.0 < result["rate_hz"] < 13.0
    assert [c["name"] for c in result["checks"]] == ["type", "robot", "lidar", "lidar TF"]
    assert all(c["fix"] is None for c in result["checks"])


def test_observation_check_fails_on_a_wrong_or_missing_topic():
    def run():
        return (
            observation_check("bad", "/bad/scan", "pointcloud"),
            observation_check("bad", "/bad/other", "auto"),
            observation_check("bad", "/bad/none", "auto"),
        )

    wrong_type, not_a_sensor, missing = with_sensor("bad", run)
    for result in (wrong_type, not_a_sensor, missing):
        assert not result["ok"]
        assert result["checks"][0]["name"] == "type" and not result["checks"][0]["ok"]
        assert result["checks"][0]["fix"]
    assert "not a pointcloud" in wrong_type["checks"][0]["detail"]
    assert "std_msgs/msg/String" in not_a_sensor["checks"][0]["detail"]
    assert missing["type"] is None and "nothing publishes" in missing["checks"][0]["detail"]


def test_observation_check_reports_a_missing_robot_instead_of_waiting():
    def run():
        start = time.monotonic()
        result = observation_check("gone", "/gone/scan", "auto")
        return result, time.monotonic() - start

    result, took = with_sensor("gone", run, robot=False)
    assert not result["ok"]
    robot = next(c for c in result["checks"] if c["name"] == "robot")
    assert not robot["ok"] and "robot not visible" in robot["detail"] and robot["fix"]
    assert result["type"] == "laserscan" and result["rate_hz"] is not None
    assert took < 10.0


def test_observation_check_fails_on_a_frame_not_on_the_robot():
    result = with_sensor(
        "loose", lambda: observation_check("loose", "/loose/scan", "auto"), placed=False
    )
    assert not result["ok"]
    frame = result["checks"][-1]
    assert frame["name"] == "lidar TF" and not frame["ok"] and "components.yaml" in frame["fix"]
