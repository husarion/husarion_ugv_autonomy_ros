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

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from nav2_common.launch import ReplaceString


def generate_launch_description():
    husarion_ugv_docking_dir = FindPackageShare("husarion_ugv_docking")

    docking_server_config_path = LaunchConfiguration("docking_server_config_path")
    declare_docking_server_config_path_arg = DeclareLaunchArgument(
        "docking_server_config_path",
        default_value=PathJoinSubstitution(
            [husarion_ugv_docking_dir, "config", "docking_server.yaml"]
        ),
        description=("Path to docking server configuration file."),
    )

    apriltag_config_path = LaunchConfiguration("apriltag_config_path")
    declare_apriltag_config_path_arg = DeclareLaunchArgument(
        "apriltag_config_path",
        default_value=PathJoinSubstitution([husarion_ugv_docking_dir, "config", "apriltag.yaml"]),
        description=("Path to apriltag configuration file. Only used in simulation."),
    )

    camera_color_topic = LaunchConfiguration("camera_image_topic")
    declare_camera_color_topic_arg = DeclareLaunchArgument(
        "camera_image_topic",
        default_value="/camera/color/image_raw",
        description="Color camera topic",
    )

    camera_info_topic = LaunchConfiguration("camera_info_topic")
    declare_camera_info_topic_arg = DeclareLaunchArgument(
        "camera_info_topic",
        default_value="/camera/color/camera_info",
        description="Camera info topic",
    )

    bt_project_path = LaunchConfiguration("bt_project_path")
    declare_bt_project_path_arg = DeclareLaunchArgument(
        "bt_project_path",
        default_value=PathJoinSubstitution(
            [husarion_ugv_docking_dir, "behavior_trees", "docking.xml"]
        ),
        description=("Path to behavior tree project file."),
    )

    namespace = LaunchConfiguration("namespace", default="")
    use_sim = LaunchConfiguration("use_sim", default="False")

    log_level = LaunchConfiguration("log_level")
    declare_log_level = DeclareLaunchArgument(
        "log_level",
        default_value="info",
        description="Logging level",
        choices=["debug", "info", "warning", "error"],
    )

    use_wibotic_info = LaunchConfiguration("use_wibotic_info")
    declare_use_wibotic_info_arg = DeclareLaunchArgument(
        "use_wibotic_info",
        default_value="True",
        description="Whether Wibotic information is used",
        choices=[True, False, "True", "False", "true", "false", "1", "0"],
    )

    ns = PythonExpression(["'", namespace, "' + '/' if '", namespace, "' else ''"])
    namespaced_docking_server_config = ReplaceString(
        source_file=docking_server_config_path,
        replacements={
            "<robot_namespace>": ns,
            "//": "",
            "<use_wibotic_info_param>": PythonExpression(
                ["'false' if ", use_sim, " else '", use_wibotic_info, "'"]
            ),
        },
    )

    send_to_dock_config_path = LaunchConfiguration("send_to_dock_config_path")
    declare_send_to_dock_config_path = DeclareLaunchArgument(
        "send_to_dock_config_path",
        default_value=PathJoinSubstitution(
            [husarion_ugv_docking_dir, "config", "send_to_dock.yaml"]
        ),
        description="Specify path to configuration file for send robot to dock service",
    )

    docking_server_node = Node(
        package="opennav_docking",
        executable="opennav_docking",
        name="docking_server",
        namespace=namespace,
        parameters=[
            namespaced_docking_server_config,
            {"use_sim_time": use_sim},
        ],
        arguments=["--ros-args", "--log-level", log_level, "--log-level", "rcl:=INFO"],
        remappings=[("~/transition_event", "~/_transition_event")],
        emulate_tty=True,
    )

    docking_server_activate_node = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="nav2_docking_lifecycle_manager",
        namespace=namespace,
        parameters=[
            {
                "autostart": True,
                "node_names": [
                    "docking_server",
                ],
                "use_sim_time": use_sim,
            },
        ],
    )

    dock_pose_publisher = Node(
        package="husarion_ugv_docking",
        executable="dock_pose_publisher",
        parameters=[
            namespaced_docking_server_config,
            {"use_sim_time": use_sim},
        ],
        name="dock_pose_publisher",
        namespace=namespace,
        emulate_tty=True,
        arguments=["--ros-args", "--log-level", log_level, "--log-level", "rcl:=INFO"],
    )

    apriltag_node = Node(
        package="apriltag_ros",
        executable="apriltag_node",
        parameters=[{"use_sim_time": use_sim}, apriltag_config_path],
        namespace=namespace,
        emulate_tty=True,
        remappings={
            "camera_info": camera_info_topic,
            "image_rect": camera_color_topic,
            "detections": "docking/april_tags",
        }.items(),
    )

    dock_database_updater = Node(
        package="husarion_ugv_docking",
        executable="dock_database_updater",
        name="dock_database_updater",
        namespace=namespace,
        parameters=[
            namespaced_docking_server_config,
            {"use_sim_time": use_sim},
        ],
        emulate_tty=True,
        arguments=["--ros-args", "--log-level", log_level, "--log-level", "rcl:=INFO"],
    )

    station_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    husarion_ugv_docking_dir,
                    "launch",
                    "station.launch.py",
                ]
            ),
        ),
        launch_arguments={"namespace": namespace}.items(),
    )

    wibotic_connector_can = Node(
        package="wibotic_connector_can",
        executable="wibotic_connector_can",
        namespace=namespace,
        emulate_tty=True,
        parameters=[{"max_service_call_retries": 20}],
        arguments=["--ros-args", "--log-level", log_level, "--log-level", "rcl:=INFO"],
        condition=IfCondition(PythonExpression(["not ", use_sim, " and ", use_wibotic_info])),
    )

    docking_manager_node = Node(
        package="husarion_ugv_docking",
        executable="docking_manager_node",
        name="docking_manager",
        parameters=[
            PathJoinSubstitution([husarion_ugv_docking_dir, "config", "docking_manager.yaml"]),
            {"bt_project_path": bt_project_path},
        ],
        arguments=["--ros-args", "--log-level", log_level, "--log-level", "rcl:=INFO"],
        namespace=namespace,
        emulate_tty=True,
    )

    send_to_dock_node = Node(
        package="husarion_ugv_docking",
        executable="send_to_dock_node",
        name="send_to_dock",
        parameters=[send_to_dock_config_path],
        namespace=namespace,
        emulate_tty=True,
    )

    spawn_charging_docks = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    husarion_ugv_docking_dir,
                    "launch",
                    "spawn_charging_docks.launch.py",
                ]
            ),
        ),
        launch_arguments={
            "namespace": namespace,
        }.items(),
        condition=IfCondition(use_sim),
    )

    return LaunchDescription(
        [
            declare_apriltag_config_path_arg,
            declare_bt_project_path_arg,
            declare_camera_color_topic_arg,
            declare_camera_info_topic_arg,
            declare_docking_server_config_path_arg,
            declare_log_level,
            declare_send_to_dock_config_path,
            declare_use_wibotic_info_arg,
            station_launch,
            docking_server_node,
            docking_server_activate_node,
            dock_pose_publisher,
            apriltag_node,
            dock_database_updater,
            docking_manager_node,
            wibotic_connector_can,
            send_to_dock_node,
            spawn_charging_docks,
        ]
    )
