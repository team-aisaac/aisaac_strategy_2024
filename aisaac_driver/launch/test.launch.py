# Copyright 2021 Roots
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

import launch
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch.actions import GroupAction
from launch_ros.actions import PushRosNamespace
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    """Generate launch description with multiple components."""

    declare_arg_invert = DeclareLaunchArgument(
        "invert",
        default_value="false",
        description=('Set "true" to invert detection_tracked data.'),
    )

    declare_arg_yellow = DeclareLaunchArgument(
        "yellow",
        default_value="false",
        description=('Set "true" to control yellow team robots.'),
    )
    container = ComposableNodeContainer(
        name="aisaac_driver_container",
        namespace="",
        package="rclcpp_components",
        executable="component_container",
        composable_node_descriptions=[
            ComposableNode(
                package="aisaac_driver",
                plugin="main_board::MainBoard",
                name="main_board_node",
                namespace="robot0",
                extra_arguments=[{"use_intra_process_comms": True}],
                parameters=[
                    {"team_is_yellow": LaunchConfiguration("yellow")},
                    {"invert": LaunchConfiguration("invert")},
                ],
            ),
            ComposableNode(
                package="robocup_ssl_comm",
                plugin="robocup_ssl_comm::GrSim",
                name="grsim",
            ),
        ],
        output="both",
    )
    # with_namespace = GroupAction(
    #     actions=[
    #         PushRosNamespace("turtlesim2"),
    #         container,
    #     ]
    # )

    descriptions = [declare_arg_invert, declare_arg_yellow, container]
    return launch.LaunchDescription(descriptions)
