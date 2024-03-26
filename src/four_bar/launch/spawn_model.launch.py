import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterValue
from launch.actions import ExecuteProcess
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.actions import ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit



def generate_launch_description():
    # Get the package directory
    pkg_dir = get_package_share_directory('four_bar')

    # use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    arg_value = LaunchConfiguration('example_arg')

    # Xacro
    # xacro_file_name = 'exo.xacro'
    xacro_file_name = 'model.xacro'
    xacro_file_path = os.path.join(pkg_dir, 'urdf', xacro_file_name)
    print(f'xacro_file_path: {xacro_file_path}')
    robot_description = Command(['xacro ', xacro_file_path])

    # Command to delete the existing model
    delete_entity_cmd = ExecuteProcess(
        cmd=['ros2', 'service', 'call', '/delete_entity',
             'gazebo_msgs/srv/DeleteEntity', f"{{name: 'four_bar'}}"],
        output='screen'
    )

    spawn_entity_cmd = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-entity', 'four_bar',
                   '-topic', 'robot_description',
                   '-x', '0.3',
                   '-y', '0.3',
                   '-z', '1'],
        output='screen')
    
    load_joint_state_broadcaster = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'joint_state_broadcaster'],
        output='screen'
    )

    load_joint_trajectory_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'effort_controller'],
        output='screen'
    )

    delay_seconds = 1.0
    spawn_entity_delayed_delayed = TimerAction(
        period=delay_seconds,
        actions=[spawn_entity_cmd]
    )

    # Launch RViz
    return LaunchDescription([
        DeclareLaunchArgument(
            'arg_name', default_value='default_value', description='Description of this argument.'
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='both',
            parameters=[
                {'robot_description': robot_description},
                {'use_sim_time': use_sim_time},
            ],
        ),
        delete_entity_cmd,
        spawn_entity_delayed_delayed,
        load_joint_state_broadcaster,
        load_joint_trajectory_controller


        
    ])
