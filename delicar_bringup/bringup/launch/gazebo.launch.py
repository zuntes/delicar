import os

from ament_index_python.packages import get_package_share_directory


from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node

import xacro


def generate_launch_description():
    gazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('gazebo_ros'), 'launch'), '/gazebo.launch.py']),
             )

    package_path = os.path.join(
        get_package_share_directory('delicar_bringup'))

    xacro_file = os.path.join(package_path,
                              'urdf',
                              'delicar.urdf.xacro')

    doc = xacro.parse(open(xacro_file))
    xacro.process_doc(doc)
    params = {'robot_description': doc.toxml()}

    node_joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
    )
    
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params]
    )

    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                        arguments=['-topic', 'robot_description',
                                   '-entity', 'delicar'],
                        output='screen')

    load_joint_state_broadcaster = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'joint_state_broadcaster'],
        output='screen'
    )

    ackermann_steering_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state',
             'active', 'ackermann_steering_controller'],
        output='screen')

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=[
            '-d',
            os.path.join(package_path, 'rviz/delicar.rviz'),
        ],
        output='screen',
    )

    return LaunchDescription([
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=spawn_entity,
                on_exit=[load_joint_state_broadcaster],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=load_joint_state_broadcaster,
                on_exit=[ackermann_steering_controller],
            )
        ),
        gazebo,
        rviz,
        node_joint_state_publisher,
        node_robot_state_publisher,
        spawn_entity,
    ])