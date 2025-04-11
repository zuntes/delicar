# Delicar-ROS2

## Overview

This repository contains the code for running the Viettel Post's Delicar Robot in ROS2.

## Dependencies
The following dependencies are required to run the Delicar:
- ROS2 (tested on Humble)
- ros2_control
- libmodbus
- gazebo-ros
- SLAM packages: Lego-LOAM, LIO-SAM (follow their instruction for installation)
- octomap-ros
- nav2-ros

## Installation
Use the following commands to download and compile the package.
```bash
cd ~/your_ws/src
git clone https://github.com/zuntes/vtp_delicar.git
cd ..
rosdep install --from-paths src -y --ignore-src
```
## How to run

### Run the real robot
If you want to use the real robot, run this

```bash
ros2 launch delicar_bringup delicar.launch.py
```
Remember to check if the RS485 port at /dev/ttyUSB0 or not. Change the config in delicar_controllers.xacro file

### Run the Simulation in Gazebo
If you want to use the simulation robot

```bash
ros2 launch delicar_gazebo gazebo.launch.py
```
In order to run simulation with 3D lidar, you need to install the velodyne gazebo package. If the gazebo fail to start, maybe you miss some model in gazebo, start with empty.world instead.

### Manual control the robot
With joy stick

```bash
ros2 launch delicar_manual_control joy_controller.launch.py
```
or with keyboard
```bash
ros2 run delicar_manual_control keyboard_controller
```

### Run the 3D SLAM Package
Right now we using LeGo_LOAM to get the 3D Map
```bash
ros2 launch lego_loam_sr run.launch.py 
```

#### Obtain the 2D Grid Map by octomap 

To generate a 2D grid map, run both the 3D SLAM node and the Octomap node simultaneously:
```bash
ros2 launch octomap_server octomap_mapping.launch.xml
```

To save the 2D Map 
```bash
ros2 run nav2_map_server map_saver_cli -f ~/your_ws/src/ --ros-args --remap map:=/projected_map
```

#### Obtain the 2D Grid Map by projection
Remember to change the link to the .pcd file in the launch params
```bash
ros2 launch pointcloud_to_2dmap pointcloud_to_2dmap.launch.py
```

## TODO
- Need to fix the problem with IMU
- The rslidar points is different with the velodyne points that are use for many 3D SLAM methods; need to be convert correctly



