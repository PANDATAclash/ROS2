## SmartCAR ROS 2 Project Overview

This repository contains our SmartCAR ROS 2 project developed for the Robot Architecture final assignment. The goal is to build a complete mobile-robot system in ROS 2 that can be simulated in Gazebo and used as a foundation for real hardware work. The project includes a full robot architecture: robot description (URDF/Xacro) and TF tree, simulation integration, sensor topics (e.g., LiDAR and IMU), vehicle command/status interfaces, wheel odometry and state estimation (localization), and autonomous navigation using the ROS 2 Navigation Stack (Nav2).

This project was developed in collaboration with my colleague **Henryk Nowacki**.

## Repository Layout

The ROS 2 workspace is located in `ROS2_main/`. The main package is under:
`ROS2_main/src/smart_car/`

Launch files (bringup entrypoints) are located in:
`ROS2_main/src/smart_car/launch/`

The recommended “run everything” launch file is:
`setup.launch.py` (it typically starts the simulator + robot description + navigation stack).

## How to Run (Simulation + Navigation)

### 1) Build the workspace
Open a terminal in the `ROS2_main` folder:

```bash
cd ROS2_main
# Source your ROS 2 distro first (example: humble)
source /opt/ros/$ROS_DISTRO/setup.bash

colcon build
source install/setup.bash
2) Start the full system (recommended)
This should bring up the full project using the main setup launch file:

ros2 launch smart_car setup.launch.py
If you want to run parts separately (optional), you can try:

ros2 launch smart_car gazebo.launch.py
ros2 launch smart_car nav2.launch.py
# or
ros2 launch smart_car smartcar.launch.py
How to Operate the System
Navigation in RViz
After launching, open RViz (if it doesn’t open automatically):

rviz2
In RViz, set the correct Fixed Frame (commonly map for Nav2).

Use:

“2D Pose Estimate” to set the initial pose (AMCL / localization).

“Nav2 Goal” (or “2D Nav Goal”) to send a navigation goal.

What “working” looks like
Common checks (topic names may vary by your setup):

ros2 node list
ros2 topic list
Typical topics you should see when everything is running:

/tf, /tf_static (TF tree)

/scan (LiDAR)

/imu (IMU)

/odom (odometry)

/cmd_vel (velocity commands from Nav2)

Quick debug commands
ros2 topic echo /cmd_vel
ros2 topic echo /scan
ros2 run tf2_tools view_frames
Troubleshooting
If the launch fails after pulling new changes: rebuild and re-source:

colcon build --symlink-install
source install/setup.bash
If RViz shows no data: confirm TF and sensors are publishing (/tf, /scan, /odom).

If Nav2 won’t move: check whether /cmd_vel is being published when you set a goal.
