# Delicar-ROS2

## Overview

This repository contains the code for running the Delicar Robot in ROS2.

### Included Packages:
- **`delicar_bringup`**: Launch files to start the robot.
- **`delicar_manual_control`**: Joystick and keyboard controller for manual robot control.

## Installation

### Dependencies

List any dependencies required to run the packages.

## Usage

### Start the Simulation in Gazebo
```bash
ros2 launch delicar_bringup gazebo.launch.py
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

