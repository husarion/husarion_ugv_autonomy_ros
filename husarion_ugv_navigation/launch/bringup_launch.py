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
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    EnvironmentVariable,
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.descriptions import ParameterFile
from launch_ros.substitutions import FindPackageShare
from nav2_common.launch import ReplaceString, RewrittenYaml


def generate_launch_description():
    husarion_ugv_navigation = FindPackageShare("husarion_ugv_navigation")
    launch_dir = PathJoinSubstitution([husarion_ugv_navigation, "launch"])

    autostart = LaunchConfiguration("autostart")
    log_level = LaunchConfiguration("log_level")
    map = LaunchConfiguration("map")
    namespace = LaunchConfiguration("namespace")
    observation_topic = LaunchConfiguration("observation_topic")
    observation_topic_type = LaunchConfiguration("observation_topic_type")
    params_file = LaunchConfiguration("params_file")
    pc2ls_params_file = LaunchConfiguration("pc2ls_params_file")
    robot_model = LaunchConfiguration("robot_model")
    slam = LaunchConfiguration("slam")
    use_composition = LaunchConfiguration("use_composition")
    use_respawn = LaunchConfiguration("use_respawn")
    use_sim_time = LaunchConfiguration("use_sim_time")

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
    declare_pc2ls_params_file_arg = DeclareLaunchArgument(
        "pc2ls_params_file",
        default_value=PathJoinSubstitution(
            [husarion_ugv_navigation, "config", "pc2ls_params.yaml"]
        ),
        description="Path to the parameters file to use for pointcloud_to_laserscan node.",
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

    # Create our own temporary YAML files that include substitutions
    param_substitutions = {"use_sim_time": use_sim_time, "yaml_filename": map}

    namespace_ext = PythonExpression(["'", namespace, "' + '/' if '", namespace, "' else ''"])
    scan_topic = PythonExpression(
        [
            "'scan' if '",
            observation_topic_type,
            "' == 'pointcloud' else '",
            observation_topic,
            "'",
        ]
    )
    add_obstacle_layer = PythonExpression(
        ["'obstacle_layer,' if '", observation_topic_type, "' == 'laserscan' else ''"]
    )
    add_voxel_layer = PythonExpression(
        ["'voxel_layer,' if '", observation_topic_type, "' == 'pointcloud' else ''"]
    )
    robot_footprint = PythonExpression(
        [
            "'[[0.45, 0.47], [0.45, -0.47], [-0.45, -0.47], [-0.45, 0.47]]' if '",
            robot_model,
            "' == 'panther' else '[[0.38, 0.33], [0.38, -0.33], [-0.38, -0.33], [-0.38, 0.33]]'",
        ]
    )

    params_file = ReplaceString(
        source_file=params_file,
        replacements={
            "<namespace>/": namespace_ext,
            "<observation_topic>": observation_topic,
            "<scan_topic>": scan_topic,
            "<obstacle_layer>,": add_obstacle_layer,
            "<voxel_layer>,": add_voxel_layer,
            "<robot_footprint>": robot_footprint,
        },
    )

    configured_params = ParameterFile(
        RewrittenYaml(
            source_file=params_file,
            root_key=namespace,
            param_rewrites=param_substitutions,
            convert_types=True,
        ),
        allow_substs=True,
    )

    bringup_cmd_group = GroupAction(
        [
            PushRosNamespace(namespace),
            Node(
                condition=IfCondition(
                    PythonExpression(["'", observation_topic_type, "' == 'pointcloud'"])
                ),
                package="pointcloud_to_laserscan",
                executable="pointcloud_to_laserscan_node",
                name="pointcloud_to_laserscan",
                parameters=[pc2ls_params_file],
                remappings=[("cloud_in", observation_topic)],
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
                }.items(),
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
            declare_pc2ls_params_file_arg,
            declare_robot_model_arg,
            declare_slam_arg,
            declare_use_composition_arg,
            declare_use_respawn_arg,
            declare_use_sim_time_arg,
            bringup_cmd_group,
        ]
    )
