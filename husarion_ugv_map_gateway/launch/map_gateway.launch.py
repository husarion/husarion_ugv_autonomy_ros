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

"""The map gateway on its own, under the robot's namespace (SPEC-MAP-NAV-2026-11 §5)."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    namespace = LaunchConfiguration("namespace")
    map_file_name = LaunchConfiguration("map_file_name")
    maps_directory = LaunchConfiguration("maps_directory")
    use_sim_time = LaunchConfiguration("use_sim_time")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "namespace",
                default_value="panther",
                description="The robot's namespace; frames are '<namespace>/map' and so on.",
            ),
            DeclareLaunchArgument(
                "map_file_name",
                default_value="",
                description="The saved pair's map.yaml when map_server and AMCL run on it; "
                "empty while mapping.",
            ),
            DeclareLaunchArgument(
                "maps_directory",
                default_value="/maps",
                description="Where serialize_map writes saved pairs and the gateway keeps its "
                "session file.",
            ),
            DeclareLaunchArgument("use_sim_time", default_value="False"),
            Node(
                package="husarion_ugv_map_gateway",
                executable="map_gateway",
                name="map_gateway",
                namespace=namespace,
                parameters=[
                    {
                        "map_file_name": map_file_name,
                        "maps_directory": maps_directory,
                        "use_sim_time": use_sim_time,
                    }
                ],
                output="screen",
            ),
        ]
    )
