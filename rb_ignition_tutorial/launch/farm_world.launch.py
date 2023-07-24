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

    # Gazebo Sim
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': f'-r tomato_field.sdf'}.items(),
    )

    transform_publisher = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments = ["--x", "0.0",
                    "--y", "0.0",
                    "--z", "0.0",
                    "--yaw", "0.0",
                    "--pitch", "0.0",
                    "--roll", "0.0",
                    "--frame-id", "costar_husky_sensor_config_1/base_link",
                    "--child-frame-id", "costar_husky_sensor_config_1"]
    )

    # Gz - ROS Bridge
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/model/costar_husky_sensor_config_1/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
            '/model/costar_husky_sensor_config_1/pose@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
            '/model/costar_husky_sensor_config_1/pose_static@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
            '/model/costar_husky_sensor_config_1/cmd_vel_relay@geometry_msgs/msg/Twist@gz.msgs.Twist',
            '/model/costar_husky_sensor_config_1/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            # '/model/costar_husky_sensor_config_1/camera_front/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
            # '/model/costar_husky_sensor_config_1/camera_front/image@sensor_msgs/msg/Image[gz.msgs.Image',
            # '/model/costar_husky_sensor_config_1/camera_front/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked',
            # '/model/costar_husky_sensor_config_1/camera_front/depth_image@sensor_msgs/msg/Image[gz.msgs.Image',
            # '/model/costar_husky_sensor_config_1/front_laser/scan/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked',
            # '/model/costar_husky_sensor_config_1/front_cliff_laser/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            # '/model/costar_husky_sensor_config_1/imu_sensor/imu@sensor_msgs/msg/Imu[gz.msgs.IMU',
            # '/model/costar_husky_sensor_config_1/gas_detected@std_msgs/msg/Bool[gz.msgs.Boolean',

            # Clock (IGN -> ROS2)
            # Joint states (IGN -> ROS2)
            # '/world/empty/model/rrbot/joint_state@sensor_msgs/msg/JointState@gz.msgs.Model',
        ],
        remappings=[
            ('/model/costar_husky_sensor_config_1/tf', '/tf'),
            ('/model/costar_husky_sensor_config_1/pose', '/tf'),
            ('/model/costar_husky_sensor_config_1/pose_static', '/tf_static'),
            ('/model/costar_husky_sensor_config_1/cmd_vel_relay', '/cmd_vel'),
            ('/model/costar_husky_sensor_config_1/odometry', '/odom'),
            # ('/model/costar_husky_sensor_config_1/battery/linear_battery/state', '/battery'),
            # ('/model/costar_husky_sensor_config_1/camera_front/camera_info', '/camera_front/camera_info'),
            # ('/model/costar_husky_sensor_config_1/camera_front/image', '/camera_front/image_raw'),
            # ('/model/costar_husky_sensor_config_1/camera_front/points', '/camera_front/points'),
            # ('/model/costar_husky_sensor_config_1/camera_front/depth_image', '/camera_front/depth/image_raw'),
            # ('/model/costar_husky_sensor_config_1/front_laser/scan/points', '/front_laser/scan/points'),
            # ('/model/costar_husky_sensor_config_1/front_cliff_laser/scan', '/front_cliff_laser/scan'),
            # ('/model/costar_husky_sensor_config_1/imu_sensor/imu', '/imu'),
            # ('/model/costar_husky_sensor_config_1/gas_detected', '/gas_detected'),
        ],
        output='screen'
    )

    return LaunchDescription(
        [
            # Nodes and Launches
            ign_resource_path,
            ign_model_path,
            gazebo,
            bridge,
            transform_publisher,
        ]
    )
