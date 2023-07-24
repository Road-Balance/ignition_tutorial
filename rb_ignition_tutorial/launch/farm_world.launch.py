# Copyright 2023 Road Balance
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

from pathlib import Path

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node

import xacro

def generate_launch_description():

    # Package Directories
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    pkg_ros_gz_sim_demos = get_package_share_directory('ros_gz_sim_demos')
    
    pkg_path = get_package_share_directory('rb_ignition_tutorial')
    pkg_turtlebot4_ignition_bringup = get_package_share_directory('turtlebot4_ignition_bringup')

    # Set ignition resource path
    ign_resource_path = SetEnvironmentVariable(
        name='IGN_GAZEBO_RESOURCE_PATH',
        value=[
            os.path.join(pkg_turtlebot4_ignition_bringup, 'worlds'), ':' +
            os.path.join(pkg_path, 'models'), ':' +
            os.path.join(pkg_path, 'models', 'tomato_field'), ':' +
            os.path.join(pkg_path, 'worlds'), ':'
        ]
    )

    ign_model_path = SetEnvironmentVariable(
        name='IGN_GAZEBO_MODEL_PATH',
        value=[
            os.path.join(pkg_path, 'models', 'tomato_field')
        ]
    )

    # world_path = os.path.join(pkg_path, 'worlds', 'farm_with_one_crop_row.sdf')

    # Gazebo Sim
    # maze.sdf, lab.sdf
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': f'-r tomato_field.sdf'}.items(),
    )

    # Gz - ROS Bridge
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            # Clock (IGN -> ROS2)
            '/clock@rosgraph_msgs/msg/Clock@gz.msgs.Clock',
            # Joint states (IGN -> ROS2)
            '/world/empty/model/rrbot/joint_state@sensor_msgs/msg/JointState@gz.msgs.Model',
        ],
        remappings=[
            ('/world/empty/model/rrbot/joint_state', 'joint_states'),
        ],
        output='screen'
    )

    return LaunchDescription(
        [
            # Nodes and Launches
            ign_resource_path,
            ign_model_path,
            gazebo,
            # bridge,
        ]
    )
