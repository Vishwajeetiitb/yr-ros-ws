from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('duration', default_value='5',
                              description='Duration for which the node runs'),
        DeclareLaunchArgument('data_size', default_value='1024',
                              description='Size of data to send'),
        DeclareLaunchArgument(
            'data_rate_hz', default_value='1000.0', description='Data rate in Hz'),

        Node(
            package='yr_lle_driver',  
            executable='mcu_bridge_ros_node',  
            name='mcu_bridge_ros_node',
            output='screen',
            parameters=[
                {'duration': LaunchConfiguration('duration')},
                {'data_size': LaunchConfiguration('data_size')},
                {'data_rate_hz': LaunchConfiguration('data_rate_hz')}
            ]
        )
    ])
