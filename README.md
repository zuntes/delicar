# Delicar-ROS2

## Overview

This repository contains the code for running the Delicar Robot in ROS2.

### Included Packages:
- **`delicar_bringup`**: Launch files to start the robot.
- **`delicar_manual_control`**: Joystick and keyboard controller for manual robot control.

## Installation
Use the following commands to download and compile the package.
```bash
cd ~/your_ws/src
git clone https://github.com/zuntes/vtp_delicar.git
cd ..
rosdep install --from-paths src -y --ignore-src
```
## Usage

### Start the Simulation in Gazebo
```bash
ros2 launch delicar_gazebo gazebo.launch.py
```
### Control the robot with joy stick
```bash
ros2 launch delicar_manual_control joy_controller.launch.py
```

### Run the 3D SLAM Package
```bash
ros2 launch lio_sam run.launch.py
```

### Obtain the 2D Grid Map
To generate a 2D grid map, run both the 3D SLAM node and the Octomap node simultaneously:
```bash
ros2 launch octomap_server octomap_mapping.launch.xml
```

### Save the 2D Map
```bash
ros2 run nav2_map_server map_saver_cli -f ~/your_ws/src/ --ros-args --remap map:=/projected_map
```

