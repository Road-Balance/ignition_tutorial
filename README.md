# ignition_tutorial

| ROS 2 Humble + Gazebo Garden Tutorial 

* Install Dependencies

```
sudo apt install ros-humble-turtlebot4-ignition-bringup -y
sudo apt install ros-humble-diff-drive-controller -y
sudo apt install ros-humble-ros2-control -y
```

* Part1. Ignition Tutorials

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

ros2 launch rb_ignition_tutorial farm_world.launch.py
ros2 launch rb_ignition_tutorial bridge.launch.py
```

* Part2. CAD to Sim, BCR Bot

```
ros2 launch bcr_bot gz.launch.py
ros2 launch bcr_bot rviz.launch.py
```

* Part3. Mobile Robot Navigation


```
# SLAM Toolbox
ros2 launch bcr_bot gz.launch.py
ros2 launch bcr_slam slam_toolbox.launch.py

# Nav2
ros2 launch bcr_bot gz.launch.py
ros2 launch bcr_navigation bringup_launch.py
```

* Part4. 

```
ros2 launch robot_demo_description diffbot_desc.launch.py 
ros2 launch robot_demo_description quadbot_desc.launch.py 

ros2 launch ignition_diff_drive empty_world.launch.py 

cbp rqt_robot_steering_qd
ros2 run rqt_robot_steering_qd rqt_robot_steering_qd --force-discover
ros2 launch ignition_quad_drive empty_world.launch.py
```