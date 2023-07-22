# ignition_tutorial
ROS 2 Humble + Gazebo Garden Tutorial 

* Install Dependencies

```
sudo apt install ros-humble-turtlebot4-ignition-bringup -y
sudo apt install ros-humble-diff-drive-controller -y
sudo apt install ros-humble-ros2-control -y
```

```
ros2 launch rb_ignition_tutorial camera.launch.py 

ros2 launch rb_ignition_tutorial depth_camera.launch.py 

ros2 launch rb_ignition_tutorial depth_camera_img_bridge.launch.py 

ros2 launch rb_ignition_tutorial diff_drive.launch.py 

ros2 launch rb_ignition_tutorial gpu_lidar.launch.py 

ros2 launch rb_ignition_tutorial gpu_lidar.launch.py 

ros2 launch rb_ignition_tutorial imu.launch.py 

ros2 launch rb_ignition_tutorial joint_states.launch.py 

ros2 launch rb_ignition_tutorial robot_description_publisher.launch.py 

ros2 launch rb_ignition_tutorial tf_bridge.launch.py 
```

* ROS 2 Control Demo

```
ros2 launch robot_demo_description diffbot_desc.launch.py 

```