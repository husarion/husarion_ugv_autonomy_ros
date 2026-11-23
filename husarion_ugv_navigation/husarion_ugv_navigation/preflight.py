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
from typing import Callable, Dict, Iterable, List, Optional, Set

from husarion_ugv_navigation.mapnav_params import namespace_prefix

WAIT_LOG_PERIOD_S = 10.0
QUICK_CHECK_S = 5.0
OBSERVATION_TYPES = {
    "sensor_msgs/msg/LaserScan": "laserscan",
    "sensor_msgs/msg/PointCloud2": "pointcloud",
}


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
        from rclpy.qos import DurabilityPolicy, QoSProfile
        from tf2_msgs.msg import TFMessage
        from tf2_ros import Buffer, TransformListener

        self.namespace = namespace
        self.prefix = namespace_prefix(namespace)
        self.timeout = timeout
        self.log = log
        self.observation_topic = observation_topic
        self.cmd_vel_topic = cmd_vel_topic
        self.frames: Set[str] = set()
        self.observation_type: Optional[str] = None
        self.observation_frame: Optional[str] = None
        self.observation_since = 0.0
        self.observation_times: List[float] = []

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
        if observation_topic and observation_topic_type != "auto":
            self._subscribe_observation(observation_topic_type)

    def _subscribe_observation(self, observation_type: str) -> None:
        from rclpy.qos import qos_profile_sensor_data
        from sensor_msgs.msg import LaserScan, PointCloud2

        msg_type = PointCloud2 if observation_type == "pointcloud" else LaserScan
        self.node.create_subscription(
            msg_type, self.observation_topic, self._on_observation, qos_profile_sensor_data
        )
        self.observation_type = observation_type
        self.observation_since = time.monotonic()

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
        self.observation_times.append(time.monotonic())

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

    def wait_for_robot(
        self, log_period_s: float = WAIT_LOG_PERIOD_S, deadline: Optional[float] = None
    ) -> bool:
        """Phase 1: block until the robot's driver publishes odom -> base_link.

        No deadline unless one is given; False when it passes first.
        """
        odom, base = f"{self.prefix}odom", f"{self.prefix}base_link"
        start = next_log = time.monotonic()
        while not self._spin_until(
            lambda: self._can_transform(odom, base),
            (
                next_log + log_period_s
                if deadline is None
                else min(next_log + log_period_s, deadline)
            ),
        ):
            next_log = time.monotonic()
            if deadline is not None and next_log >= deadline:
                return False
            self.log(
                waiting_message(
                    self.namespace, next_log - start, other_robots(self.frames, self.prefix)
                )
            )
        waited = time.monotonic() - start
        if waited >= log_period_s:
            self.log(f"preflight: the robot is here after {waited:.0f} s")
        return True

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

    def observation_topics(self) -> List[Dict[str, str]]:
        """LaserScan and PointCloud2 topics in the graph after `timeout` s of discovery."""
        self._spin_until(lambda: False, time.monotonic() + self.timeout)
        return [
            {"name": name, "type": OBSERVATION_TYPES[t]}
            for name, types in sorted(self.node.get_topic_names_and_types())
            for t in types
            if t in OBSERVATION_TYPES
        ]

    def check_observation_type(self, wanted: str) -> Check:
        """The topic's type from the graph; subscribes to the topic when it fits."""
        topic = self.observation_topic

        def graph_types() -> List[str]:
            return next((t for n, t in self.node.get_topic_names_and_types() if n == topic), [])

        self._spin_until(lambda: bool(graph_types()), time.monotonic() + self.timeout)
        types = graph_types()
        found = next((OBSERVATION_TYPES[t] for t in types if t in OBSERVATION_TYPES), None)
        if found is None and not types:
            return Check(
                "type",
                False,
                f"nothing publishes {topic}",
                "No program on this network publishes this topic. Start the sensor's driver, "
                "check the topic's name, and check that this computer and the sensor's computer "
                "use the same ROS network settings (ROS_DOMAIN_ID, the DDS configuration).",
            )
        if found is None:
            return Check(
                "type",
                False,
                f"{topic} carries {', '.join(types)}",
                "Navigation reads a sensor_msgs/LaserScan or a sensor_msgs/PointCloud2 topic. "
                "Pick the topic of the LiDAR or depth camera itself.",
            )
        if wanted not in ("auto", found):
            return Check(
                "type",
                False,
                f"{topic} carries a {found}, not a {wanted}",
                f"Set the observation type to {found}.",
            )
        if self.observation_type is None:
            self._subscribe_observation(found)
        return Check("type", True, f"{topic} carries a {found}")

    def check_robot_visible(self) -> Check:
        odom, base = f"{self.prefix}odom", f"{self.prefix}base_link"
        if self.wait_for_robot(deadline=time.monotonic() + self.timeout):
            return Check("robot", True, f"{odom} -> {base} is available")
        others = other_robots(self.frames, self.prefix)
        fix = (
            "This computer does not see the robot's driver. Check that the robot is on, that "
            "this computer shares its ROS network (Manage > ROS networking) and that the "
            "project's namespace is the robot's."
        )
        if others:
            fix += f" TF carries another robot: {', '.join(sorted(others))}."
        return Check(
            "robot",
            False,
            f"robot not visible: no {odom} -> {base} in {self.timeout:.0f} s",
            fix,
        )

    def observation_rate(self) -> Optional[float]:
        """Messages per second over the window, from the first message to the last."""
        self._spin_until(lambda: False, self.observation_since + self.timeout)
        times = self.observation_times
        if len(times) < 2 or times[-1] <= times[0]:
            return None
        return (len(times) - 1) / (times[-1] - times[0])

    def run_observation(self, wanted_type: str) -> List[Check]:
        """The user computer's check of an observation topic, before Nav2 is installed."""
        results = [self.check_observation_type(wanted_type)]
        robot = self.check_robot_visible()
        results.append(robot)
        if results[0].ok:
            data = self.check_observation()
            results.append(data)
            if data.ok:
                data.value = self.observation_rate()
                if data.value is not None:
                    data.detail += f", {data.value:.1f} Hz"
                if robot.ok:
                    results.append(self.check_observation_frame())
        return results

    def observation_summary(self, checks: List[Check]) -> dict:
        rate = next((c.value for c in checks if c.name == "lidar" and c.ok), None)
        return {
            "ok": all(c.ok for c in checks),
            "type": self.observation_type,
            "frame_id": self.observation_frame,
            "rate_hz": None if rate is None else round(rate, 1),
            "checks": [
                {"name": c.name, "ok": c.ok, "detail": c.detail, "fix": None if c.ok else c.fix}
                for c in checks
            ],
        }

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
