from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='pointcloud_to_2dmap',
            executable='pointcloud_to_2dmap_node',
            name='pointcloud_to_2dmap_node',
            output='screen',
            parameters=[
                {"resolution": 0.05},
                {"map_width": 600},
                {"map_height": 600},
                {"min_points_in_pix": 1},
                {"max_points_in_pix": 2},
                {"min_height": 0.1},
                {"max_height": 10.0},
                {"dest_directory": "/home/zuntes/test_ws/src/lidar_localization_ros2/maps/2dmap"}, #change to your directory    
                {"input_pcd": "/home/zuntes/test_ws/src/lidar_localization_ros2/maps/map.pcd"}, #change to your directory
                {"map_name": "2dmap"}
            ]
        )
    ])
