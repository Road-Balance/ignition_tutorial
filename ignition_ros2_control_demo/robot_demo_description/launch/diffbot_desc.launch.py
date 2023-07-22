# Copyright 2023 RoadBalance Inc.

# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at

#     http://www.apache.org/licenses/LICENSE-2.0

# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os

from ament_index_python.packages import get_package_share_directory
from launch.actions import TimerAction
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


from launch.substitutions import (
    Command,
    FindExecutable,
    PathJoinSubstitution,
)

def generate_launch_description():

    pkg_path = os.path.join(get_package_share_directory("robot_demo_description"))
    rviz_config_file = os.path.join(pkg_path, "rviz", "desc.rviz")

    joint_state_publisher_gui = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        name="joint_state_publisher_gui",
    )

    # Get URDF via xacro
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare("robot_demo_description"), "urdf", "diffbot.urdf.xacro"]
            ),
            " ",
            "prefix:=",
            "",
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    # Launch Robot State Publisher
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name='robot_state_publisher',
        output="screen",
        parameters=[robot_description],
    )

    # Launch RViz
    rviz2 = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config_file],
    )

    return LaunchDescription([
        TimerAction(
            period=3.0,
            actions=[rviz2]
        ),

        robot_state_publisher_node,
        joint_state_publisher_gui,
    ])