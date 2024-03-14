from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='yr_lle_app_demo',
            executable='app_demo_node',
            name='app_demo_node',
            parameters=[{'print_rx_msgs': True}],
        ),
    ])

