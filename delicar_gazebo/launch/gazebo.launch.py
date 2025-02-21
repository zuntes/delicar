import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, RegisterEventHandler
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition

def generate_launch_description():
    use_sim_time = True

    world_path = PathJoinSubstitution(
        [FindPackageShare("delicar_gazebo"), "worlds", "playground.world"]
    )
    urdf_path = PathJoinSubstitution(
        [FindPackageShare("delicar_description"), "urdf", "delicar.urdf.xacro"]
    )

    ekf_config_path = PathJoinSubstitution(
        [FindPackageShare("delicar_bringup"), "config", "ekf.yaml"]
    )

    arg_urdf = DeclareLaunchArgument(
        name='urdf', 
        default_value=urdf_path,
        description='URDF path'
    )
    arg_world = DeclareLaunchArgument(
        name='world', 
        default_value=world_path,
        description='Gazebo world'
    )
    arg_spawn_x = DeclareLaunchArgument(
        name='spawn_x', 
        default_value='0.0',
        description='Robot spawn position in X axis'
    )
    arg_spawn_y = DeclareLaunchArgument(
        name='spawn_y', 
        default_value='0.0',
        description='Robot spawn position in Y axis'
    )
    arg_spawn_z = DeclareLaunchArgument(
        name='spawn_z', 
        default_value='0.0',
        description='Robot spawn position in Z axis'
    )
    arg_spawn_yaw = DeclareLaunchArgument(
        name='spawn_yaw', 
        default_value='0.0',
        description='Robot spawn heading'
    )
    arg_publish_joints = DeclareLaunchArgument(
        name='publish_joints', 
        default_value='true',
        description='Launch joint_states_publisher'
    )
# 
    gazebo_process = ExecuteProcess(
        cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_factory.so', 
             '-s', 'libgazebo_ros_init.so', LaunchConfiguration('world')],
        output='screen'
    )

    # Need to add arguments to the joy_controller and twist_mux launch files
    joystick = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('delicar_manual_control'),'launch','joy_controller.launch.py')]),
    )
    
    twist_mux =IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('twist_mux'),'launch','twist_mux.launch.py'
                )]),
    )
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='urdf_spawner',
        output='screen',
        arguments=[
            '-topic', 'robot_description',
            '-entity', 'delicar',
            '-x', LaunchConfiguration('spawn_x'),
            '-y', LaunchConfiguration('spawn_y'),
            '-z', LaunchConfiguration('spawn_z'),
            '-Y', LaunchConfiguration('spawn_yaw'),
        ]
    )

    joint_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        condition=IfCondition(LaunchConfiguration("publish_joints")),
        parameters=[{'use_sim_time': use_sim_time}]
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': Command(['xacro ', LaunchConfiguration('urdf')])
        }]
    )

    load_joint_broadcaster = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'joint_state_broadcaster'],
        output='screen'
    )

    load_ackermann_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'ackermann_steering_controller'],
        output='screen'
    )

    joint_broadcaster_event = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_entity,
            on_exit=[load_joint_broadcaster]
        )
    )

    ackermann_controller_event = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=load_joint_broadcaster,
            on_exit=[load_ackermann_controller]
        )
    )
    
    localiztion_node = Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[
                {'use_sim_time': use_sim_time}, 
                ekf_config_path
            ],
    )

    return LaunchDescription([
        arg_urdf,
        arg_world,
        arg_spawn_x,
        arg_spawn_y,
        arg_spawn_z,
        arg_spawn_yaw,
        arg_publish_joints,
        
        joystick,
        twist_mux,
        gazebo_process,
        spawn_entity,
        
        joint_publisher,
        robot_state_publisher,
        
        joint_broadcaster_event,
        ackermann_controller_event,
        
        localiztion_node
    ])