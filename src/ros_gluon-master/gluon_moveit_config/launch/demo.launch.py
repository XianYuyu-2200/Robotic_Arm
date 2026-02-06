from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    # Robot Description
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare("gluon"), "urdf", "gluon_ros2.urdf"]
            ),
        ]
    )
    robot_description = {"robot_description": ParameterValue(robot_description_content, value_type=str)}

    # Controller Config
    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare("gluon_control"),
            "config",
            "gluon_controllers_ros2.yaml",
        ]
    )

    # Robot State Publisher
    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )

    # Controller Manager
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, robot_controllers],
        output="both",
    )

    # Spawners
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    robot_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_trajectory_controller", "--controller-manager", "/controller_manager"],
    )

    # Move Group
    move_group_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("gluon_moveit_config"), "launch", "move_group.launch.py"]
            )
        )
    )

    # RViz
    rviz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("gluon_moveit_config"), "launch", "moveit_rviz.launch.py"]
            )
        )
    )

    # Delay RViz and Move Group until controllers are started
    
    # Let's chain them
    delay_rviz_after_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=robot_controller_spawner,
            on_exit=[rviz_launch],
        )
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "use_fake_hardware",
            default_value="true",
            description="Start robot with fake hardware mirroring command to its states.",
        ),
        control_node,
        robot_state_pub_node,
        joint_state_broadcaster_spawner,
        robot_controller_spawner,
        move_group_launch,
        delay_rviz_after_spawner,
    ])
