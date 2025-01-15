from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_stamped_twist = LaunchConfiguration('use_stamped_twist')
    
    joy_params = os.path.join(get_package_share_directory('delicar_manual_control'),'config','joy_controller.yaml')

    joy_node = Node(
            package='joy',
            executable='joy_node',
            parameters=[joy_params, {'use_sim_time': use_sim_time}],
         )

    teleop_node = Node(
            package='teleop_twist_joy',
            executable='teleop_node',
            name='joy_teleop_node',
            parameters=[joy_params, {'use_sim_time': use_sim_time}, {'publish_stamped_twist': use_stamped_twist}],
            remappings=[('/cmd_vel','/ackermann_steering_controller/reference')]
         )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use sim time if true'),
        DeclareLaunchArgument(
            'use_stamped_twist',
            default_value='true',
            description='Use stamped twist if true'),
        joy_node,
        teleop_node,
    ])