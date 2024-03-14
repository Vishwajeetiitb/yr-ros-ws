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

    # Define the path to the world file
    # world_file_path = os.path.join(pkg_dir, 'worlds', 'empty.world')
    world_file_path = os.path.join(pkg_dir, 'worlds', 'exoskeleton.world')

    # Gazebo launch command
    gazebo = ExecuteProcess(
        cmd=['gazebo', '--verbose', '-s',
             'libgazebo_ros_factory.so', '-world', world_file_path],
        output='screen'
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'xacro_file', default_value=xacro_file,
            description='Path to robot xacro file'
        ),
        gazebo,
    ])
