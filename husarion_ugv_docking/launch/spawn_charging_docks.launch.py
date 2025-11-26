#!/usr/bin/env python3

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

import os

import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def spawn_stations(context, *args, **kwargs):
    docking_server_config_path = LaunchConfiguration("docking_server_config_path").perform(context)
    apriltag_mount_height = LaunchConfiguration("apriltag_mount_height").perform(context)
    docking_server_config = None

    try:
        with open(os.path.join(docking_server_config_path)) as file:
            docking_server_config = yaml.safe_load(file)
        if not isinstance(docking_server_config, dict) or "/**" not in docking_server_config:
            raise ValueError("Invalid configuration structure")
    except Exception as e:
        print(f"Error loading docking server config: {str(e)}")
        return []

    actions = []

    ros_parameters = docking_server_config["/**"]["ros__parameters"]
    docks_names = ros_parameters["docks"]
    for dock_name in docks_names:
        pose = ros_parameters[dock_name]["pose"]

        spawn_station = Node(
            package="ros_gz_sim",
            executable="create",
            name=[dock_name, "_station_spawner"],
            arguments=[
                "-name",
                [dock_name, "_station"],
                "-topic",
                [dock_name, "_station_description"],
                "-x",
                str(pose[0]),
                "-y",
                str(pose[1]),
                "-z",
                apriltag_mount_height,  # AprilTag is a root link of the station
                "-R",
                "1.57",
                "-P",
                "0.0",
                "-Y",
                str(pose[2] - 1.57),
            ],
            emulate_tty=True,
        )

        actions.append(spawn_station)

    return actions


def generate_launch_description():
    declare_docking_server_config_path_arg = DeclareLaunchArgument(
        "docking_server_config_path",
        default_value=PathJoinSubstitution(
            [
                FindPackageShare("husarion_ugv_docking"),
                "config",
                "docking_server.yaml",
            ]
        ),
        description=("Path to docking server configuration file."),
    )
    declare_apriltag_mount_height_arg = DeclareLaunchArgument(
        "apriltag_mount_height",
        default_value="0.5",
        description=("Height of the AprilTag."),
    )

    return LaunchDescription(
        [
            declare_docking_server_config_path_arg,
            declare_apriltag_mount_height_arg,
            OpaqueFunction(function=spawn_stations),
        ]
    )
