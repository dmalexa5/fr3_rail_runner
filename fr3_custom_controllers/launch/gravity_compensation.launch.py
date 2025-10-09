import os
import xacro
from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution, Command, FindExecutable, LaunchConfiguration
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, ExecuteProcess, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.event_handlers import OnProcessStart, OnProcessExit
from launch_ros.parameter_descriptions import ParameterValue

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    robot_description_content = Command([
        PathJoinSubstitution([FindExecutable(name='xacro')]),
        ' ',
        PathJoinSubstitution([
            FindPackageShare('fr3_moveit_config'),
            'config',
            'fr3.urdf.xacro'
        ]),
        ' ',
        'use_gazebo:=true ',
        'initial_positions_file:=',
        PathJoinSubstitution([
            FindPackageShare('fr3_moveit_config'),
            'config',
            'initial_positions.yaml'
        ]),
    ],
    on_stderr='warn'
    )

    robot_description = {'robot_description': ParameterValue(robot_description_content, value_type=str)}

    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare('fr3_custom_controllers'),
            'config',
            'ros2_controllers.yaml',
        ]
    )
    
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [PathJoinSubstitution([FindPackageShare('ros_gz_sim'),
                                    'launch',
                                    'gz_sim.launch.py'])]),
        launch_arguments=[('gz_args', [' -r -v 1 empty.sdf'])])

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[
            robot_description
            ],
    )

    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-topic', 'robot_description',
                   '-name', 'fr3',],
        output='screen'
    )


    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'joint_state_broadcaster',
            '--param-file',
            robot_controllers,
            ],
    )

    gripper_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'hand_controller',
            '--param-file',
            robot_controllers,
            ],
    )
    gravity_compensation_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'gravity_compensation_controller',
            '--param-file',
            robot_controllers,
            ],
    )
    # Bridge
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        output='screen'
    )

    use_sim_time = LaunchConfiguration('use_sim_time', default=True)

    return LaunchDescription([
        SetEnvironmentVariable(
            name='IGN_GAZEBO_RESOURCE_PATH',
            value=[
                os.environ.get('IGN_GAZEBO_RESOURCE_PATH', ''),
                os.pathsep,
                os.path.dirname(get_package_share_directory('fr3_description'))
            ]
        ),
        bridge,
        gazebo_launch,
        robot_state_publisher_node,
        spawn_entity,
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=spawn_entity,
                on_exit=[joint_state_broadcaster_spawner],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=joint_state_broadcaster_spawner,
                on_exit=[gripper_controller_spawner, gravity_compensation_controller_spawner],
            )
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value=use_sim_time,
            description='If true, use simulated clock'),
    ])
