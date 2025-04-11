from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Declare launch arguments
    input_topic_arg = DeclareLaunchArgument(
        'input_topic',
        default_value='/twist_mux/cmd_vel_out',
        description='Input topic for Twist messages'
    )
    
    output_topic_arg = DeclareLaunchArgument(
        'output_topic',
        default_value='/ackermann_steering_controller/reference',
        description='Output topic for TwistStamped messages'
    )
    
    # frame_id_arg = DeclareLaunchArgument(
    #     'frame_id',
    #     default_value='base_link',
    #     description='Frame ID for the TwistStamped header'
    # )

    # Create node
    twist_converter_node = Node(
        package='twist_converter',
        executable='twist_converter',
        name='twist_converter',
        parameters=[{
            'input_topic': LaunchConfiguration('input_topic'),
            'output_topic': LaunchConfiguration('output_topic'),
            # 'frame_id': LaunchConfiguration('frame_id')
        }],
        output='screen'
    )

    return LaunchDescription([
        input_topic_arg,
        output_topic_arg,
        # frame_id_arg,
        twist_converter_node
    ])