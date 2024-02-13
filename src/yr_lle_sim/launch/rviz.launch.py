import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Get the package directory
    pkg_dir = get_package_share_directory('yr_lle_sim')

    rviz_file_path = os.path.join(pkg_dir, 'rviz', 'exo.rviz')
    print(f'rviz_file_path: {rviz_file_path}')
        
    arg_value = LaunchConfiguration('example_arg')

    # Launch RViz
    return LaunchDescription([
        DeclareLaunchArgument(
            'arg_name', default_value='default_value', description='Description of this argument.'
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', os.path.join(pkg_dir, 'rviz', 'exo.rviz')],
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use sim time if true'),
    ])
