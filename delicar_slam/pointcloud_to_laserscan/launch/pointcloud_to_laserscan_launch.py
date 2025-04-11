from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            name='scanner', default_value='',
            description='Namespace for sample topics'
        ),
        Node(
            package='pointcloud_to_laserscan', executable='pointcloud_to_laserscan_node',
            remappings=[('cloud_in', [LaunchConfiguration(variable_name='scanner'), '/lidar/points_raw']),
                        ('scan', [LaunchConfiguration(variable_name='scanner'), '/scan'])],
            parameters=[{
                'target_frame': 'laser_2d',
                'transform_tolerance': 0.01,
                'queue_size': 10,
                'min_height': 0.0,
                'max_height': 2.0,
                'angle_min': -3.1415,  
                'angle_max': 3.1415, 
                'angle_increment': 0.0087, 
                'scan_time': 0.3333,
                'range_min': 0.1,
                'range_max': 50.0,
                'use_inf': True,
                'inf_epsilon': 1.0,
            }],
            name='pointcloud_to_laserscan'
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_transform_publisher',
            arguments=[
                '--x', '0', '--y', '0', '--z', '0',
                '--qx', '0', '--qy', '0', '--qz', '0', '--qw', '1',
                '--frame-id', 'base_footprint', '--child-frame-id', 'laser_2d'
            ]
        ),
    ])
