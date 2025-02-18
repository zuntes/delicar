import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch.conditions import IfCondition, UnlessCondition


MAP_NAME='playground'

def generate_launch_description():

    nav2_launch_path = PathJoinSubstitution(
        [FindPackageShare('nav2_bringup'), 'launch', 'bringup_launch.py']
    )

    rviz_config_path = PathJoinSubstitution(
        [FindPackageShare('delicar_navigation'), 'rviz', 'delicar_navigation.rviz']
    )

    default_map_path = PathJoinSubstitution(
        [FindPackageShare('delicar_navigation'), 'maps', f'{MAP_NAME}.yaml']
    )

    nav2_config_path = PathJoinSubstitution(
        [FindPackageShare('delicar_navigation'), 'config', 'navigation.yaml']
    )

    nav2_sim_config_path = PathJoinSubstitution(
        [FindPackageShare('delicar_navigation'), 'config', 'navigation_sim.yaml']
    )


    return LaunchDescription([
        DeclareLaunchArgument(
            name='sim', 
            default_value='true',
            description='Enable use_sime_time to true'
        ),

        DeclareLaunchArgument(
            name='rviz', 
            default_value='true',
            description='Run rviz'
        ),

       DeclareLaunchArgument(
            name='map', 
            default_value=default_map_path,
            description='Navigation map path'
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(nav2_launch_path),
            condition=UnlessCondition(LaunchConfiguration("sim")),
            launch_arguments={
                'map': LaunchConfiguration("map"),
                'use_sim_time': LaunchConfiguration("sim"),
                'params_file': nav2_config_path
            }.items()
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(nav2_launch_path),
            condition=IfCondition(LaunchConfiguration("sim")),
            launch_arguments={
                'map': LaunchConfiguration("map"),
                'use_sim_time': LaunchConfiguration("sim"),
                'params_file': nav2_sim_config_path
            }.items()
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config_path],
            condition=IfCondition(LaunchConfiguration("rviz")),
            parameters=[{'use_sim_time': LaunchConfiguration("sim")}]
        )
    ])