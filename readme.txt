#Delicar-ROS2 
=====================

## Overview

This repository contains the code for running Delicar Robot in ROS2.
Including:
- `delicar_bringup`: Launch files to start the robot.
- `delicar_manual_control`: Joystick and keyboard controller to control the robot manually.

## Installation

## Dependencies

## Usage
- To start the simualtion in Gazebo
``` ros2 launch delicar_bringup gazebo.launch.py ```
- To run the 3D SLAM package
``` ros2 launch lio_sam run.launch.py ```
- To obtain the 2D grid map, you need to run both 3D SLAM node and octomap node at the same time
``` ros2 launch octomap_server octomap_mapping.launch.xml ```
- To save the 2D map:
``` ros2 run nav2_map_server map_saver_cli -f ~/demo_ws/src/ --ros-args --remap map:=/projected_map ```