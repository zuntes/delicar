import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    # Create launch arguments for configurable parameters
    input_topic_arg = DeclareLaunchArgument(
        'input_topic',
        default_value='/rslidar_points',
        description='Input RoboSense point cloud topic'
    )
    
    output_topic_arg = DeclareLaunchArgument(
        'output_topic',
        default_value='/velodyne_points',
        description='Output Velodyne point cloud topic'
    )
    
    frame_id_arg = DeclareLaunchArgument(
        'frame_id',
        default_value='velodyne',
        description='Frame ID for the output point cloud'
    )
    
    flat_organization_arg = DeclareLaunchArgument(
        'flat_organization',
        default_value='True',
        description='Whether to use Velodyne flat organization (height=1)'
    )
    
    max_ring_arg = DeclareLaunchArgument(
        'max_ring_value',
        default_value='16',
        description='Maximum ring value for synthetic time calculation'
    )
    
    scan_time_arg = DeclareLaunchArgument(
        'synthetic_scan_time',
        default_value='0.1',
        description='Synthetic scan time in seconds'
    )

    # Create the RoboSense to Velodyne converter node
    rs_to_velodyne_converter_node = Node(
        package='rs_to_velodyne',
        executable='rs_to_velodyne_converter',
        name='rs_to_velodyne_converter',
        output='screen',
        parameters=[{
            'input_topic': LaunchConfiguration('input_topic'),
            'output_topic': LaunchConfiguration('output_topic'),
            'frame_id': LaunchConfiguration('frame_id'),
            'flat_organization': LaunchConfiguration('flat_organization'),
            'max_ring_value': LaunchConfiguration('max_ring_value'),
            'synthetic_scan_time': LaunchConfiguration('synthetic_scan_time')
        }]
    )

    return LaunchDescription([
        input_topic_arg,
        output_topic_arg,
        frame_id_arg,
        flat_organization_arg,
        max_ring_arg,
        scan_time_arg,
        rs_to_velodyne_converter_node
    ])