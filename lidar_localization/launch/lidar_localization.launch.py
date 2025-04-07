import os
import time

import launch
import launch.actions
import launch.events
import launch_ros
import launch_ros.actions
from launch import LaunchDescription
from launch_ros.actions import LifecycleNode, Node
from launch.actions import EmitEvent, LogInfo, RegisterEventHandler
from launch_ros.events.lifecycle import ChangeState
from launch_ros.event_handlers import OnStateTransition
from ament_index_python.packages import get_package_share_directory
import lifecycle_msgs.msg

from lifecycle_msgs.msg import Transition

def generate_launch_description():
    # RViz配置檔案
    rviz_config = os.path.join(
        get_package_share_directory('lidar_localization'), 'rviz', 'localization.rviz')

    # 地圖文件路徑
    map_file_path = os.path.join(
        get_package_share_directory('lidar_localization'), 'maps', 'map.yaml')

    # RViz節點
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config]
    )

    # 確認RViz啟動成功
    rviz_start_checker = launch.actions.RegisterEventHandler(
        launch.event_handlers.OnProcessStart(
            target_action=rviz_node,
            on_start=[
                LogInfo(msg="RViz successfully started."),
                LogInfo(msg="Proceeding with other nodes.")
            ]
        )
    )

    # 地圖服務節點
    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{'yaml_filename': map_file_path}]
    )

    lidar_tf = launch_ros.actions.Node(
        name='lidar_tf',
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0.60','0','0.69','0','0','0','1','base_link','laser_3d']
        )

    imu_tf = launch_ros.actions.Node(
        name='imu_tf',
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0','0','0','0','0','0','1','base_link','imu_link']
        )

    localization_param_dir = launch.substitutions.LaunchConfiguration(
        'localization_param_dir',
        default=os.path.join(
            get_package_share_directory('lidar_localization'),
            'param',
            'localization.yaml'))

    pcl_localization = launch_ros.actions.LifecycleNode(
        name='pcl_localization',
        namespace='',
        package='lidar_localization',
        executable='pcl_localization_node',
        parameters=[localization_param_dir],
        remappings=[('/cloud','/lidar/points_raw')],
        output='screen')
    
    # Lifecycle transitions
    to_inactive = launch.actions.EmitEvent(
        event=launch_ros.events.lifecycle.ChangeState(
            lifecycle_node_matcher=launch.events.matches_action(pcl_localization),
            transition_id=Transition.TRANSITION_CONFIGURE,  # Correct reference
        )
    )

    from_unconfigured_to_inactive = launch.actions.RegisterEventHandler(
        launch_ros.event_handlers.OnStateTransition(
            target_lifecycle_node=pcl_localization,
            goal_state='unconfigured',
            entities=[
                launch.actions.LogInfo(msg="-- Unconfigured --"),
                launch.actions.EmitEvent(event=launch_ros.events.lifecycle.ChangeState(
                    lifecycle_node_matcher=launch.events.matches_action(pcl_localization),
                    transition_id=Transition.TRANSITION_CONFIGURE,  # Correct reference
                )),
            ],
        )
    )

    from_inactive_to_active = launch.actions.RegisterEventHandler(
        launch_ros.event_handlers.OnStateTransition(
            target_lifecycle_node=pcl_localization,
            start_state='configuring',
            goal_state='inactive',
            entities=[
                launch.actions.LogInfo(msg="-- Inactive --"),
                launch.actions.EmitEvent(event=launch_ros.events.lifecycle.ChangeState(
                    lifecycle_node_matcher=launch.events.matches_action(pcl_localization),
                    transition_id=Transition.TRANSITION_ACTIVATE,  # Correct reference
                )),
            ],
        )
    )


    '''
    # 配置生命周期節點狀態轉換
    to_inactive = launch.actions.EmitEvent(
        event=launch_ros.events.lifecycle.ChangeState(
            lifecycle_node_matcher=launch.events.matches_action(pcl_localization),
            transition_id=lifecycle_msgs.msg.Transition.TRANSITION_CONFIGURE,
        )
    )

    from_unconfigured_to_inactive = launch.actions.RegisterEventHandler(
        launch_ros.event_handlers.OnStateTransition(
            target_lifecycle_node=pcl_localization,
            goal_state='unconfigured',
            entities=[
                launch.actions.LogInfo(msg="-- Unconfigured --"),
                launch.actions.EmitEvent(event=launch_ros.events.lifecycle.ChangeState(
                    lifecycle_node_matcher=launch.events.matches_action(pcl_localization),
                    transition_id=lifecycle_msgs.msg.Transition.TRANSITION_CONFIGURE,
                )),
            ],
        )
    )

    from_inactive_to_active = launch.actions.RegisterEventHandler(
        launch_ros.event_handlers.OnStateTransition(
            target_lifecycle_node=pcl_localization,
            start_state='configuring',
            goal_state='inactive',
            entities=[
                launch.actions.LogInfo(msg="-- Inactive --"),
                launch.actions.EmitEvent(event=launch_ros.events.lifecycle.ChangeState(
                    lifecycle_node_matcher=launch.events.matches_action(pcl_localization),
                    transition_id=lifecycle_msgs.msg.Transition.TRANSITION_ACTIVATE,
                )),
            ],
        )
    )
    '''
    
    # Launch描述
    ld = LaunchDescription()
    ld.add_action(rviz_node)
    ld.add_action(rviz_start_checker)
    time.sleep(1)
    ld.add_action(map_server_node)
    ld.add_action(from_unconfigured_to_inactive)
    ld.add_action(from_inactive_to_active)

    ld.add_action(pcl_localization)
    # ld.add_action(lidar_tf)
    ld.add_action(to_inactive)

    return ld
