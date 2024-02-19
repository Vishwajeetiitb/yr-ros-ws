import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, Command
from launch.actions import ExecuteProcess


def generate_launch_description():

    urdf_file_path = os.path.join(get_package_share_directory(
        'yr_lle_sim'), 'urdf', 'simple_leg.urdf')

    with open(urdf_file_path, 'r') as file:
        robot_desc = file.read()

    config_file_path = os.path.join(
        get_package_share_directory('yr_lle_sim'),
        'config',
        'simple_leg_control.yaml'
    )

    return LaunchDescription([
        Node(
            package='gazebo_ros', executable='spawn_entity.py',
            # arguments=['-entity', 'simple_leg', '-topic', 'robot_description'],
            arguments=['-entity', 'yr_lle_model',
                       '-topic', 'robot_description',
                       '-x', '0.3',
                       '-y', '0.3',
                       '-z', '2'],
            output='screen',
            parameters=[{'robot_description': robot_desc}]
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='both',
            parameters=[{'robot_description': robot_desc}],
        ),
        Node(
            package='controller_manager',
            executable='ros2_control_node',
            parameters=[config_file_path],
            output={
                'stdout': 'screen',
                'stderr': 'screen',
            },
        ),
        ExecuteProcess(
            cmd=['ros2', 'control', 'load_controller', 'joint_state_controller'],
            shell=True,
            output='screen',
        ),
        ExecuteProcess(
            cmd=['ros2', 'control', 'load_controller', 'knee_effort_controller'],
            shell=True,
            output='screen',
        ),
        ExecuteProcess(
            cmd=['ros2', 'control', 'set_controller_state',
                 'joint_state_controller', 'active'],
            shell=True,
            output='screen',
        ),
        ExecuteProcess(
            cmd=['ros2', 'control', 'set_controller_state',
                 'knee_effort_controller', 'active'],
            shell=True,
            output='screen',
        ),
    ])


generate_launch_description()
