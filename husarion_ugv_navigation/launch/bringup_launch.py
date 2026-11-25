# Copyright 2024 Husarion sp. z o.o.
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

import json
import os
import tempfile

from husarion_ugv_navigation.mapnav_params import (
    apply_overlay,
    build_overlay,
    is_true,
    read_saved_pose,
    render_placeholders,
    scan_heights,
    speed_cap,
)
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
    RegisterEventHandler,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    EnvironmentVariable,
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.descriptions import ParameterFile
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from nav2_common.launch import RewrittenYaml


def render_params_file(context, measured_mount_height=None) -> str:
    """Render the parameter file once: placeholders, the map-nav overlay, the speed cap.

    Returns the rendered file's path, the mount height used and AMCL's starting pose.
    """

    def arg(name):
        return LaunchConfiguration(name).perform(context)

    namespace = arg("namespace")
    robot_model = arg("robot_model")
    observation_topic = arg("observation_topic")
    observation_topic_type = arg("observation_topic_type")
    lidar_profile = arg("lidar_profile")
    slam = is_true(arg("slam"))

    with open(arg("params_file"), encoding="utf-8") as f:
        text = render_placeholders(
            f.read(), namespace, robot_model, observation_topic, observation_topic_type
        )

    mount_height = None
    if lidar_profile:
        mount_height = float(arg("mount_height")) if arg("mount_height") else measured_mount_height

    scan_min_height, scan_max_height = scan_heights(
        observation_topic_type, arg("scan_min_height"), arg("scan_max_height")
    )

    initial_pose = None
    if is_true(arg("initial_pose_from_sidecar")) and not slam:
        initial_pose = read_saved_pose(arg("map"))

    overlay = build_overlay(
        namespace=namespace,
        robot_model=robot_model,
        observation_topic=observation_topic,
        observation_topic_type=observation_topic_type,
        cmd_vel_out=arg("cmd_vel_out"),
        use_collision_monitor=arg("use_collision_monitor"),
        lidar_profile=lidar_profile,
        mount_height=mount_height,
        robot_height=arg("robot_height"),
        scan_min_height=scan_min_height,
        scan_max_height=scan_max_height,
        map_update_interval=arg("map_update_interval"),
        initial_pose=initial_pose,
    )
    cap = speed_cap(lidar_profile)
    if overlay or cap is not None:
        text = apply_overlay(text, overlay, cap)

    rendered = tempfile.NamedTemporaryFile(
        mode="w", prefix="husarion_nav2_params_", suffix=".yaml", delete=False
    )
    with rendered:
        rendered.write(text)
    return rendered.name, mount_height, initial_pose


def launch_setup(context, mount_height=None):
    husarion_ugv_navigation = FindPackageShare("husarion_ugv_navigation")
    launch_dir = PathJoinSubstitution([husarion_ugv_navigation, "launch"])

    autostart = LaunchConfiguration("autostart")
    log_level = LaunchConfiguration("log_level")
    map = LaunchConfiguration("map")
    namespace = LaunchConfiguration("namespace")
    observation_topic = LaunchConfiguration("observation_topic")
    observation_topic_type = LaunchConfiguration("observation_topic_type")
    slam = LaunchConfiguration("slam")
    use_composition = LaunchConfiguration("use_composition")
    use_respawn = LaunchConfiguration("use_respawn")
    use_sim_time = LaunchConfiguration("use_sim_time")
    use_collision_monitor = LaunchConfiguration("use_collision_monitor")
    cmd_vel_out = LaunchConfiguration("cmd_vel_out")
    use_map_gateway = LaunchConfiguration("use_map_gateway")
    maps_directory = LaunchConfiguration("maps_directory")

    params_file, mount_height, initial_pose = render_params_file(context, mount_height)
    # On a saved map the gateway runs on that pair; while mapping on none (spec §5.5).
    gateway_map_file = "" if is_true(slam.perform(context)) else map.perform(context)

    # Create our own temporary YAML files that include substitutions
    param_substitutions = {"use_sim_time": use_sim_time, "yaml_filename": map}

    observation_topic_filtered = PythonExpression(
        ["'", observation_topic, "_filtered'"],
    )

    configured_params = ParameterFile(
        RewrittenYaml(
            source_file=params_file,
            param_rewrites=param_substitutions,
            convert_types=True,
        ),
        allow_substs=True,
    )

    notes = []
    if mount_height is not None:
        notes.append(LogInfo(msg=f"map-nav: mount_link is {mount_height:.3f} m above the ground"))
    if initial_pose is not None:
        notes.append(LogInfo(msg=f"map-nav: AMCL starts at the saved pose {initial_pose}"))

    bringup_cmd_group = GroupAction(
        [
            PushRosNamespace(namespace),
            Node(
                condition=IfCondition(
                    PythonExpression(["'", observation_topic_type, "' == 'pointcloud'"])
                ),
                package="pointcloud_crop_box",
                executable="pointcloud_crop_box_node",
                name="pointcloud_crop_box",
                parameters=[configured_params],
                output="screen",
            ),
            Node(
                condition=IfCondition(
                    PythonExpression(["'", observation_topic_type, "' == 'pointcloud'"])
                ),
                package="pointcloud_to_laserscan",
                executable="pointcloud_to_laserscan_node",
                name="pointcloud_to_laserscan",
                parameters=[configured_params],
                remappings=[("cloud_in", observation_topic_filtered)],
                output="screen",
            ),
            Node(
                condition=IfCondition(use_composition),
                name="nav2_container",
                package="rclcpp_components",
                executable="component_container_isolated",
                parameters=[configured_params, {"autostart": autostart}],
                arguments=["--ros-args", "--log-level", log_level],
                output="screen",
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution([launch_dir, "slam_launch.py"])
                ),
                condition=IfCondition(slam),
                launch_arguments={
                    "autostart": autostart,
                    "namespace": namespace,
                    "params_file": params_file,
                    "use_respawn": use_respawn,
                    "use_sim_time": use_sim_time,
                }.items(),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution([launch_dir, "localization_launch.py"])
                ),
                condition=UnlessCondition(slam),
                launch_arguments={
                    "autostart": autostart,
                    "container_name": "nav2_container",
                    "map": map,
                    "namespace": namespace,
                    "params_file": params_file,
                    "use_composition": use_composition,
                    "use_respawn": use_respawn,
                    "use_sim_time": use_sim_time,
                }.items(),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution([launch_dir, "navigation_launch.py"])
                ),
                launch_arguments={
                    "namespace": namespace,
                    "use_sim_time": use_sim_time,
                    "autostart": autostart,
                    "params_file": params_file,
                    "use_composition": use_composition,
                    "use_respawn": use_respawn,
                    "container_name": "nav2_container",
                    "use_collision_monitor": use_collision_monitor,
                    "cmd_vel_out": cmd_vel_out,
                }.items(),
            ),
            # The Cockpit's map gateway (SPEC-MAP-NAV-2026-11 §5): only when asked. On a saved
            # map it runs on that pair (its sidecar gives the epoch); while mapping on none.
            Node(
                condition=IfCondition(use_map_gateway),
                package="husarion_ugv_map_gateway",
                executable="map_gateway",
                name="map_gateway",
                parameters=[
                    {
                        "map_file_name": gateway_map_file,
                        "maps_directory": maps_directory,
                        "use_sim_time": ParameterValue(use_sim_time, value_type=bool),
                    }
                ],
                arguments=["--ros-args", "--log-level", log_level],
                output="screen",
            ),
            Node(
                condition=IfCondition(slam),
                name="map_autosaver",
                package="husarion_ugv_navigation",
                executable="map_autosaver_node",
                parameters=[configured_params],
                arguments=["--ros-args", "--log-level", log_level],
                output="screen",
            ),
        ]
    )

    return [*notes, bringup_cmd_group]


def preflight_gate(context):
    """Start Nav2 only once the robot is here and set up for it (husarion_ugv_navigation.preflight).

    The wait runs in its own process so the launch keeps answering a stop while the robot boots.
    """

    def arg(name):
        return LaunchConfiguration(name).perform(context)

    need_mount = bool(arg("lidar_profile")) and not arg("mount_height")
    checks = is_true(arg("preflight"))
    if not need_mount and not checks:
        return [OpaqueFunction(function=launch_setup)]

    namespace = arg("namespace")
    cmd_vel_out = arg("cmd_vel_out")
    if not cmd_vel_out.startswith("/"):
        cmd_vel_out = "/" + "/".join(p for p in (namespace.strip("/"), cmd_vel_out) if p)
    fd, result_file = tempfile.mkstemp(prefix="mapnav_preflight_", suffix=".json")
    os.close(fd)
    preflight = Node(
        package="husarion_ugv_navigation",
        executable="autonomy_preflight",
        name="autonomy_preflight",
        arguments=[
            "--namespace",
            namespace,
            "--observation-topic",
            arg("observation_topic"),
            "--observation-topic-type",
            arg("observation_topic_type"),
            "--cmd-vel-topic",
            cmd_vel_out,
            "--timeout",
            arg("preflight_timeout"),
            "--result-file",
            result_file,
            *(["--mount"] if need_mount else []),
            *([] if checks else ["--skip-checks"]),
        ],
        output="screen",
    )

    def on_exit(event, exit_context):
        if exit_context.is_shutdown:
            return None
        if event.returncode != 0:
            raise RuntimeError("map-nav preflight failed (see its report above): Nav2 not started")
        with open(result_file, encoding="utf-8") as f:
            mount_height = json.load(f)["mount_height"]
        os.unlink(result_file)
        return [OpaqueFunction(function=launch_setup, kwargs={"mount_height": mount_height})]

    return [
        preflight,
        RegisterEventHandler(OnProcessExit(target_action=preflight, on_exit=on_exit)),
    ]


def generate_launch_description():
    husarion_ugv_navigation = FindPackageShare("husarion_ugv_navigation")

    declare_autostart_arg = DeclareLaunchArgument(
        "autostart",
        default_value="true",
        description="Automatically startup the nav2 stack.",
    )
    declare_log_level_arg = DeclareLaunchArgument(
        "log_level",
        default_value="info",
        description="Logging level.",
        choices=["debug", "info", "warning", "error"],
    )
    declare_map_arg = DeclareLaunchArgument(
        "map",
        default_value="/maps/map.yaml",
        description="Path to map yaml file to load.",
    )
    declare_namespace_arg = DeclareLaunchArgument(
        "namespace",
        default_value=EnvironmentVariable("ROBOT_NAMESPACE", default_value=""),
        description="Add namespace to all launched nodes.",
    )
    declare_observation_topic_arg = DeclareLaunchArgument(
        "observation_topic",
        default_value="",
        description="Topic name for LaserScan or PointCloud2 observation messages type.",
    )
    declare_observation_topic_type_arg = DeclareLaunchArgument(
        "observation_topic_type",
        default_value="pointcloud",
        description="Observation topic type.",
        choices=["laserscan", "pointcloud"],
    )
    declare_params_file_arg = DeclareLaunchArgument(
        "params_file",
        default_value=PathJoinSubstitution(
            [husarion_ugv_navigation, "config", "nav2_params.yaml"]
        ),
        description="Path to the parameters file to use for all nav2 related nodes",
    )

    declare_robot_model_arg = DeclareLaunchArgument(
        "robot_model",
        default_value=EnvironmentVariable(name="ROBOT_MODEL_NAME", default_value="panther"),
        description="Specify robot model",
        choices=["lynx", "panther"],
    )
    declare_slam_arg = DeclareLaunchArgument(
        "slam", default_value="False", description="Whether run a SLAM."
    )
    declare_use_composition_arg = DeclareLaunchArgument(
        "use_composition",
        default_value="True",
        description="Whether to use composed bringup.",
    )
    declare_use_respawn_arg = DeclareLaunchArgument(
        "use_respawn",
        default_value="False",
        description="Whether to respawn if a node crashes. Applied when composition is disabled.",
    )
    declare_use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
        description="Use simulation (Gazebo) clock if true.",
    )

    # map-nav arguments (SPEC-MAP-NAV-2026-11). Each default keeps the stack as it was.
    declare_cmd_vel_out_arg = DeclareLaunchArgument(
        "cmd_vel_out",
        default_value="cmd_vel",
        description="Topic of the stack's final velocity output, relative to the namespace. "
        "On a Husarion UGV OS robot: autonomous/cmd_vel, the only drive input the user computer "
        "reaches.",
    )
    declare_use_collision_monitor_arg = DeclareLaunchArgument(
        "use_collision_monitor",
        default_value="False",
        description="Run nav2_collision_monitor between the velocity smoother and cmd_vel_out, "
        "on the observation source.",
    )
    declare_lidar_profile_arg = DeclareLaunchArgument(
        "lidar_profile",
        default_value="",
        description="3D LiDAR profile: the STVL frustum, the scan slice's range and, for os2_64, "
        "a 0.3 m/s linear speed cap (its near field is blind). Empty: the parameter file as it "
        "is.",
        choices=["", "os0_32", "os2_64"],
    )
    declare_mount_height_arg = DeclareLaunchArgument(
        "mount_height",
        default_value="",
        description="Height of mount_link (the rail) above base_footprint in metres, used with "
        "lidar_profile. Empty: read from TF at launch.",
    )
    declare_robot_height_arg = DeclareLaunchArgument(
        "robot_height",
        default_value="",
        description="Top of the robot with its payload above base_footprint in metres, used with "
        "lidar_profile: the costmaps and the collision monitor count obstacles up to it plus "
        "0.2 m. Empty: the rail plus the sensor.",
    )
    declare_scan_min_height_arg = DeclareLaunchArgument(
        "scan_min_height",
        default_value="",
        description="Lower edge of the 2D scan slice above base_footprint. Empty: 0.05 for a "
        "laserscan, 0.25 for a pointcloud.",
    )
    declare_scan_max_height_arg = DeclareLaunchArgument(
        "scan_max_height",
        default_value="",
        description="Upper edge of the 2D scan slice above base_footprint. Empty: 2.0 for a "
        "laserscan, 1.0 for a pointcloud.",
    )
    declare_map_update_interval_arg = DeclareLaunchArgument(
        "map_update_interval",
        default_value="",
        description="slam_toolbox's map_update_interval in seconds. Empty: the parameter file's.",
    )
    declare_initial_pose_from_sidecar_arg = DeclareLaunchArgument(
        "initial_pose_from_sidecar",
        default_value="False",
        description="With slam:=False, start AMCL at saved_pose from the map's sidecar "
        "(<map stem>.georef.yaml). The launch fails if the sidecar is missing or does not match "
        "the map image.",
    )

    declare_preflight_arg = DeclareLaunchArgument(
        "preflight",
        default_value="True",
        description="Before Nav2: wait for the robot with no deadline, then check its TF, the "
        "LiDAR and the velocity sink, and stop with a report if one fails. False skips the checks; "
        "the launch still waits for the robot when it reads mount_height from TF.",
    )
    declare_preflight_timeout_arg = DeclareLaunchArgument(
        "preflight_timeout",
        default_value="30.0",
        description="Seconds each preflight check waits once the robot is here.",
    )

    declare_use_map_gateway_arg = DeclareLaunchArgument(
        "use_map_gateway",
        default_value="False",
        description="Run the Cockpit's map gateway (husarion_ugv_map_gateway): the PNG map, the "
        "pose, the plan, the GNSS alignment with fromLL/toLL, pause and resume, serialize_map and save_map.",
    )
    declare_maps_directory_arg = DeclareLaunchArgument(
        "maps_directory",
        default_value="/maps",
        description="Where the map gateway's map saves write the pairs and where it keeps "
        "its session file.",
    )

    return LaunchDescription(
        [
            SetEnvironmentVariable("RCUTILS_LOGGING_BUFFERED_STREAM", "1"),
            declare_autostart_arg,
            declare_log_level_arg,
            declare_map_arg,
            declare_namespace_arg,
            declare_observation_topic_arg,
            declare_observation_topic_type_arg,
            declare_params_file_arg,
            declare_robot_model_arg,
            declare_slam_arg,
            declare_use_composition_arg,
            declare_use_respawn_arg,
            declare_use_sim_time_arg,
            declare_cmd_vel_out_arg,
            declare_use_collision_monitor_arg,
            declare_lidar_profile_arg,
            declare_mount_height_arg,
            declare_robot_height_arg,
            declare_scan_min_height_arg,
            declare_scan_max_height_arg,
            declare_map_update_interval_arg,
            declare_initial_pose_from_sidecar_arg,
            declare_use_map_gateway_arg,
            declare_maps_directory_arg,
            declare_preflight_arg,
            declare_preflight_timeout_arg,
            OpaqueFunction(function=preflight_gate),
        ]
    )
