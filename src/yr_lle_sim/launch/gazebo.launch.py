from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Define the path to the xacro file
    pkg_dir = get_package_share_directory('yr_lle_sim')
    xacro_file = os.path.join(pkg_dir, 'urdf', 'exo.xacro')

    # Convert xacro to URDF
    robot_description = {'robot_description': Command(['xacro ', xacro_file])}

    # Gazebo launch command
    gazebo = ExecuteProcess(
        cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_factory.so'],
        output='screen'
    )

    # Spawn robot model in Gazebo
    spawn_entity = Node(
        package='gazebo_ros', executable='spawn_entity.py',
        arguments=['-entity', 'your_robot_model',
                   '-topic', 'robot_description'],
        output='screen'
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'xacro_file', default_value=xacro_file,
            description='Path to robot xacro file'
        ),
        # DeclareLaunchArgument(
        #     'use_sim_time',
        #     default_value='true',
        #     description='Use sim time if true'),
        gazebo,
        # spawn_entity
    ])
