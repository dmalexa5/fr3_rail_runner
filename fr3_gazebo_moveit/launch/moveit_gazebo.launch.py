import os
from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution, Command, FindExecutable, LaunchConfiguration
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, ExecuteProcess, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.event_handlers import OnProcessStart, OnProcessExit

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launch_utils import (
    add_debuggable_node,
    DeclareBooleanLaunchArg,
)

def generate_launch_description():

    # Robot description
    urdf_file = get_package_share_directory('fr3_moveit_config') + '/config/fr3.urdf.xacro'
        
    # MoveIt configuration
    moveit_config = (
        MoveItConfigsBuilder("fr3", package_name="fr3_moveit_config")
        .robot_description(
            file_path=urdf_file,
            mappings={
                "initial_positions_file": PathJoinSubstitution([
                    FindPackageShare("fr3_moveit_config"), "config", "initial_positions.yaml"
                ]),
                "use_gazebo": "true",
            }
        )
        .to_moveit_configs()
    )

    ld = LaunchDescription()


    # Use sim time
    ld.add_action(
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation clock if true'
        )
    )
    use_sim_time = LaunchConfiguration('use_sim_time', default=True)

    # Publish the robot state to tf
    ld.add_action(
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            respawn=True,
            output="screen",
            parameters=[
                moveit_config.robot_description,
                {'use_sim_time': use_sim_time},
            ],
        )
    )

    # Bridge /clock topic for sim time
    ld.add_action(
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
            output='screen'
        )
    )

    # Add gazebo model path to launch description
    ld.add_action(
        SetEnvironmentVariable(
            name='IGN_GAZEBO_RESOURCE_PATH',
            value=[
                os.environ.get('IGN_GAZEBO_RESOURCE_PATH', ''),
                os.pathsep,
                os.path.dirname(get_package_share_directory('fr3_description'))
            ]
        )
    )

    # Launch gazebo with an empty world
    ld.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [PathJoinSubstitution([FindPackageShare('ros_gz_sim'),
                                        'launch',
                                        'gz_sim.launch.py'])]),
            launch_arguments=[('gz_args', [' -r -v 1 empty.sdf'])])
    )

    # If there are virtual joints, broadcast static tf by including virtual_joints launch
    virtual_joints_launch = (
        moveit_config.package_path / "launch/static_virtual_joint_tfs.launch.py"
    )

    if virtual_joints_launch.exists():
        ld.add_action(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(str(virtual_joints_launch)),
            )
        )

    # Start the move_group node
    # TODO: abstract this away into (likely) a seperate file (maybe .yaml config file?)
    for launch_arg in [
        DeclareBooleanLaunchArg("debug", default_value=False),
        DeclareBooleanLaunchArg("allow_trajectory_execution", default_value=True),
        DeclareBooleanLaunchArg("publish_monitored_planning_scene", default_value=True),
        DeclareLaunchArgument(
            "capabilities",
            default_value=moveit_config.move_group_capabilities["capabilities"],
        ),
        DeclareLaunchArgument(
            "disable_capabilities",
            default_value=moveit_config.move_group_capabilities["disable_capabilities"],
        ),
        DeclareBooleanLaunchArg("monitor_dynamics", default_value=False)
    ]:
        ld.add_action(launch_arg)

    should_publish = LaunchConfiguration("publish_monitored_planning_scene")

    move_group_configuration = {
        "publish_robot_description_semantic": True,
        "allow_trajectory_execution": LaunchConfiguration("allow_trajectory_execution"),
        # Note: Wrapping the following values is necessary so that the parameter value can be the empty string
        "capabilities": ParameterValue(
            LaunchConfiguration("capabilities"), value_type=str
        ),
        "disable_capabilities": ParameterValue(
            LaunchConfiguration("disable_capabilities"), value_type=str
        ),
        # Publish the planning scene of the physical robot so that rviz plugin can know actual robot
        "publish_planning_scene": should_publish,
        "publish_geometry_updates": should_publish,
        "publish_state_updates": should_publish,
        "publish_transforms_updates": should_publish,
        "monitor_dynamics": False,
        
    }

    move_group_params = [
        moveit_config.to_dict(),
        move_group_configuration,
        {'use_sim_time': use_sim_time},
        {"allowed_start_tolerance": 0.05} # TODO: remove once a better controller is in place
    ]

    add_debuggable_node(
        ld,
        package="moveit_ros_move_group",
        executable="move_group",
        commands_file=str(moveit_config.package_path / "launch" / "gdb_settings.gdb"),
        output="screen",
        parameters=move_group_params,
        extra_debug_args=["--debug"],
        # Set the display variable, in case OpenGL code is used internally
        additional_env={"DISPLAY": os.environ.get("DISPLAY", "")},
    )

    # Run Rviz and load the default config to see the state of the move_group node
    ld.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                str(moveit_config.package_path / "launch/moveit_rviz.launch.py")
            )
        )
    )

    # Spawn the controllers
    for controller in ["joint_state_broadcaster", "hand_controller", "fr3_arm_controller"]:
        ld.add_action(
                Node(
                package='controller_manager',
                executable='spawner',
                arguments=[
                    controller,
                    '--param-file',
                    os.path.join(get_package_share_directory('fr3_moveit_config'), 'config', 'ros2_controllers.yaml'),
                    ],
            )
        )


    # Spawn the robot in Gazebo
    ld.add_action(
        Node(
            package='ros_gz_sim',
            executable='create',
            arguments=['-topic', 'robot_description',
                    '-name', 'fr3',],
            output='screen'
        )
    )

    return ld