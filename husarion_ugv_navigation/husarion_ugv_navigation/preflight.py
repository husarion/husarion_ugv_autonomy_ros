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

"""Map-nav preflight: wait for the robot, then check what Nav2 needs from it.

Two phases, because "the robot is not here yet" and "the robot is here but set up wrong" need
opposite answers. A robot that boots with its user computer, or whose driver restarts, shows up
tens of seconds late; that is normal, so phase 1 waits for it with no deadline and says every
few seconds what it waits for. Once the robot's driver publishes its odom -> base_link, each
check of phase 2 has a timeout and a failure names the problem and its fix.
"""

import time
from dataclasses import dataclass
from typing import Callable, Iterable, List, Optional, Set

from husarion_ugv_navigation.mapnav_params import namespace_prefix

WAIT_LOG_PERIOD_S = 10.0
QUICK_CHECK_S = 5.0


@dataclass
class Check:
    name: str
    ok: bool
    detail: str
    fix: Optional[str] = None
    value: Optional[float] = None


def other_robots(frames: Iterable[str], prefix: str) -> Set[str]:
    """Namespaces of robots whose base_link is in `frames`, other than the one at `prefix`."""
    found = set()
    for frame in frames:
        head, sep, tail = frame.rpartition("/")
        if tail == "base_link" and frame != f"{prefix}base_link":
            found.add(head if sep else "(none)")
    return found


def waiting_message(namespace: str, waited_s: float, others: Set[str]) -> str:
    who = f"robot {namespace}" if namespace else "the robot (no namespace)"
    text = (
        f"preflight: waiting for {who} for {waited_s:.0f} s: no odom -> base_link from it yet. "
        "Normal while the robot boots or its driver restarts."
    )
    if others:
        text += (
            f" TF carries another robot: {', '.join(sorted(others))}. If that is yours, set "
            "ROBOT_NAMESPACE (namespace:=) to it."
        )
    return text


def report(checks: List[Check], namespace: str, timeout: float) -> str:
    lines = [f"\nmap-nav preflight: namespace {namespace or '(none)'}, timeout {timeout:.0f} s\n"]
    for c in checks:
        lines.append(f"  [{'ok  ' if c.ok else 'FAIL'}] {c.name:<14} {c.detail}")
    for c in checks:
        if not c.ok and c.fix:
            lines.append(f"\n--- {c.name} ---\n{c.fix}")
    return "\n".join(lines) + "\n"


class Preflight:
    def __init__(
        self,
        namespace: str,
        observation_topic: str,
        observation_topic_type: str,
        cmd_vel_topic: str,
        timeout: float,
        log: Callable[[str], None] = print,
    ):
        import rclpy
        from rclpy.executors import SingleThreadedExecutor
        from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
        from sensor_msgs.msg import LaserScan, PointCloud2
        from tf2_msgs.msg import TFMessage
        from tf2_ros import Buffer, TransformListener

        self.namespace = namespace
        self.prefix = namespace_prefix(namespace)
        self.timeout = timeout
        self.log = log
        self.observation_topic = observation_topic
        self.cmd_vel_topic = cmd_vel_topic
        self.frames: Set[str] = set()
        self.observation_frame: Optional[str] = None

        self.context = rclpy.Context()
        rclpy.init(context=self.context)
        self.node = rclpy.create_node("mapnav_preflight", context=self.context)
        self.executor = SingleThreadedExecutor(context=self.context)
        self.executor.add_node(self.node)
        self.buffer = Buffer()
        TransformListener(self.buffer, self.node, spin_thread=False)
        self.node.create_subscription(TFMessage, "/tf", self._on_tf, 100)
        static = QoSProfile(depth=100, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.node.create_subscription(TFMessage, "/tf_static", self._on_tf, static)
        if observation_topic:
            msg_type = PointCloud2 if observation_topic_type == "pointcloud" else LaserScan
            self.node.create_subscription(
                msg_type, observation_topic, self._on_observation, qos_profile_sensor_data
            )

    def close(self):
        import rclpy

        self.node.destroy_node()
        if self.context.ok():
            rclpy.shutdown(context=self.context)

    def _on_tf(self, msg):
        for t in msg.transforms:
            self.frames.add(t.child_frame_id)
            self.frames.add(t.header.frame_id)

    def _on_observation(self, msg):
        self.observation_frame = msg.header.frame_id

    def _spin_until(self, predicate, deadline: Optional[float]) -> bool:
        while deadline is None or time.monotonic() < deadline:
            # rclpy's SIGINT/SIGTERM handler shuts the context down; spin_once then returns at once
            if not self.context.ok():
                raise KeyboardInterrupt
            self.executor.spin_once(timeout_sec=0.1)
            if predicate():
                return True
        return False

    def _can_transform(self, target: str, source: str) -> bool:
        from rclpy.time import Time

        return self.buffer.can_transform(target, source, Time())

    def wait_for_robot(self, log_period_s: float = WAIT_LOG_PERIOD_S) -> None:
        """Phase 1: block until the robot's driver publishes odom -> base_link. No deadline."""
        odom, base = f"{self.prefix}odom", f"{self.prefix}base_link"
        start = next_log = time.monotonic()
        while not self._spin_until(
            lambda: self._can_transform(odom, base), next_log + log_period_s
        ):
            next_log = time.monotonic()
            self.log(
                waiting_message(
                    self.namespace, next_log - start, other_robots(self.frames, self.prefix)
                )
            )
        waited = time.monotonic() - start
        if waited >= log_period_s:
            self.log(f"preflight: the robot is here after {waited:.0f} s")

    def check_mount(self) -> Check:
        target, source = f"{self.prefix}base_footprint", f"{self.prefix}mount_link"
        ok = self._spin_until(
            lambda: self._can_transform(target, source), time.monotonic() + self.timeout
        )
        if ok:
            from rclpy.time import Time

            z = self.buffer.lookup_transform(target, source, Time()).transform.translation.z
            return Check(
                "mount TF", True, f"mount_link is {z:.3f} m above base_footprint", value=z
            )
        return Check(
            "mount TF",
            False,
            f"no transform {target} -> {source}",
            "The LiDAR profile places the sensor from the robot's mount_link (the rail), and "
            "the robot's description has none. Pass mount_height:=<metres> (MOUNT_HEIGHT in the "
            "project) to give the rail's height above the ground instead.",
        )

    def check_observation(self) -> Check:
        topic = self.observation_topic
        ok = self._spin_until(
            lambda: self.observation_frame is not None, time.monotonic() + self.timeout
        )
        if ok:
            return Check("lidar", True, f"{topic} is publishing (frame {self.observation_frame})")
        if not self.node.get_publishers_info_by_topic(topic):
            return Check(
                "lidar",
                False,
                f"nothing publishes {topic}",
                "The LiDAR driver is not running on this computer, or publishes on another "
                "topic. Check the project's LiDAR service (docker compose ps, its log) and that "
                "the LiDAR is plugged in (an RPLIDAR on /dev/ttyUSB0).",
            )
        return Check(
            "lidar",
            False,
            f"{topic} is advertised but no message arrived in {self.timeout:.0f} s",
            "The LiDAR driver runs but sends nothing: look at its log, and for an RPLIDAR at "
            "its USB cable and the RPLIDAR_MODEL (a3, s3) the project was installed with.",
        )

    def check_observation_frame(self) -> Check:
        frame, base = self.observation_frame, f"{self.prefix}base_link"
        ok = self._spin_until(
            lambda: self._can_transform(base, frame), time.monotonic() + QUICK_CHECK_S
        )
        if ok:
            return Check("lidar TF", True, f"{frame} is placed on the robot")
        return Check(
            "lidar TF",
            False,
            f"no transform {base} -> {frame}",
            f"The LiDAR's frame {frame} is not in the robot's description, so Nav2 cannot "
            "place what it sees. Add the LiDAR to the robot's components file in the Cockpit "
            "(Manage > Robot > Driver files, components.yaml) and apply.",
        )

    def check_cmd_vel_sink(self) -> Check:
        topic = self.cmd_vel_topic
        # On a Husarion UGV OS robot the fleet half of the airlock bridge subscribes to a
        # fleet topic only once a publisher appears on it. A publisher that sends nothing
        # makes it subscribe for real, and an unshared topic still has no subscriber.
        from geometry_msgs.msg import TwistStamped

        probe = self.node.create_publisher(TwistStamped, topic, 1)
        try:
            ok = self._spin_until(
                lambda: bool(self.node.get_subscriptions_info_by_topic(topic)),
                time.monotonic() + max(QUICK_CHECK_S, self.timeout),
            )
        finally:
            self.node.destroy_publisher(probe)
        if ok:
            return Check("velocity sink", True, f"the robot listens on {topic}")
        return Check(
            "velocity sink",
            False,
            f"nothing subscribes {topic}",
            "Nav2's velocity would reach nothing. On a Husarion UGV OS robot the user computer "
            "drives through <namespace>/autonomous/cmd_vel (cmd_vel_out:=autonomous/cmd_vel); "
            "check that the robot shares it with this computer (Manage > ROS networking).",
        )

    def run(self, need_mount: bool, checks: bool) -> List[Check]:
        self.wait_for_robot()
        results = []
        if need_mount:
            results.append(self.check_mount())
        if checks:
            results.append(
                Check(
                    "driver TF",
                    True,
                    f"{self.prefix}odom -> {self.prefix}base_link is available",
                )
            )
            if self.observation_topic:
                results.append(self.check_observation())
                if results[-1].ok:
                    results.append(self.check_observation_frame())
            results.append(self.check_cmd_vel_sink())
        return results


def mount_height_of(checks: List[Check]) -> Optional[float]:
    return next((c.value for c in checks if c.name == "mount TF" and c.ok), None)
