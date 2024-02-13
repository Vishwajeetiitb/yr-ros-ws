import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterValue


def generate_launch_description():
    # Get the package directory
    pkg_dir = get_package_share_directory('yr_lle_sim')

    # use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    arg_value = LaunchConfiguration('example_arg')

    # Define the URDF file path
    # urdf_file_path = os.path.join(pkg_dir, 'urdf', 'exo.urdf')
    # print(f'urdf_file_path: {urdf_file_path}')
    # # Read the URDF file
    # with open(urdf_file_path, 'r') as file:
    #     robot_description = file.read()

    # Xacro
    xacro_file_path = os.path.join(pkg_dir, 'urdf', 'exo.xacro')
    print(f'xacro_file_path: {xacro_file_path}')
    robot_description = Command(['xacro ', xacro_file_path])

    spawn_entity_cmd = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-entity', 'yr_lle_model',
                   '-topic', 'robot_description',
                   '-x', '0.3',
                   '-y', '0.3',
                   '-z', '1'],
        output='screen')

    pub_joint_states = Node(
        package='yr_lle_sim',
        executable='publish_joint_states.py',
        name='publish_joint_states',
        output='screen',
        # remappings=[
            # ('/joint_states','/model/yr_lle/joint_states')
        # ]
    )

    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'source_list': ['/model/yr_lle/joint_states']},
        ],
    )

    joint_state_publisher_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
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
                # {'robot_description': ParameterValue(Command([
                #     'xacro', ' ', xacro_file_path, ' ',
                #     'gazebo:=ignition', ' ',]), value_type=str)},
                # 'namespace:=', namespace]), value_type=str)},
                {'use_sim_time': use_sim_time},
                # {'use_sim_time': LaunchConfiguration('use_sim_time')},
            ],
        ),
        # joint_state_publisher,
        # joint_state_publisher_gui,
        pub_joint_states,
        # spawn_entity_cmd,
        # ExecuteProcess(
        #     cmd=['ros2', 'service', 'call', '/spawn_entity', 'gazebo_msgs/SpawnEntity', swpan_args],
        #     output='screen'),
    ])
