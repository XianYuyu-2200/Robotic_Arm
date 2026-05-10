import os
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, "r") as file:
            return yaml.safe_load(file)
    except EnvironmentError:
        return None


def generate_launch_description():
    use_realsense = LaunchConfiguration("use_realsense")
    use_ros2_control = LaunchConfiguration("use_ros2_control")
    use_move_group = LaunchConfiguration("use_move_group")
    use_rviz = LaunchConfiguration("use_rviz")
    use_static_camera_tf = LaunchConfiguration("use_static_camera_tf")
    target_label = LaunchConfiguration("target_label")
    score_threshold = LaunchConfiguration("score_threshold")
    approach_distance = LaunchConfiguration("approach_distance")
    stop_distance = LaunchConfiguration("stop_distance")
    dry_run = LaunchConfiguration("dry_run")
    position_only_target = LaunchConfiguration("position_only_target")
    ee_link = LaunchConfiguration("ee_link")
    camera_frame = LaunchConfiguration("camera_frame")
    max_velocity_scaling = LaunchConfiguration("max_velocity_scaling")
    max_acceleration_scaling = LaunchConfiguration("max_acceleration_scaling")
    planning_frame = LaunchConfiguration("planning_frame")
    min_x = LaunchConfiguration("min_x")
    max_x = LaunchConfiguration("max_x")
    min_y = LaunchConfiguration("min_y")
    max_y = LaunchConfiguration("max_y")
    min_z = LaunchConfiguration("min_z")
    max_z = LaunchConfiguration("max_z")
    static_camera_parent_frame = LaunchConfiguration("static_camera_parent_frame")
    static_camera_child_frame = LaunchConfiguration("static_camera_child_frame")
    static_camera_x = LaunchConfiguration("static_camera_x")
    static_camera_y = LaunchConfiguration("static_camera_y")
    static_camera_z = LaunchConfiguration("static_camera_z")
    static_camera_roll = LaunchConfiguration("static_camera_roll")
    static_camera_pitch = LaunchConfiguration("static_camera_pitch")
    static_camera_yaw = LaunchConfiguration("static_camera_yaw")

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([FindPackageShare("gluon"), "urdf", "gluon_ros2.urdf"]),
        ]
    )
    robot_description = {"robot_description": ParameterValue(robot_description_content, value_type=str)}

    robot_description_semantic_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([FindPackageShare("gluon_moveit_config"), "config", "gluon.srdf"]),
        ]
    )
    robot_description_semantic = {
        "robot_description_semantic": ParameterValue(robot_description_semantic_content, value_type=str)
    }

    kinematics_yaml = load_yaml("gluon_moveit_config", "config/kinematics.yaml")

    robot_controllers = PathJoinSubstitution(
        [FindPackageShare("gluon_control"), "config", "gluon_controllers_ros2.yaml"]
    )

    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        condition=IfCondition(use_ros2_control),
        parameters=[robot_description],
    )

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        output="both",
        condition=IfCondition(use_ros2_control),
        parameters=[robot_description, robot_controllers],
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        condition=IfCondition(use_ros2_control),
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    robot_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        condition=IfCondition(use_ros2_control),
        arguments=["joint_trajectory_controller", "--controller-manager", "/controller_manager"],
    )

    realsense_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare("realsense2_camera"), "launch", "rs_launch.py"])
        ),
        condition=IfCondition(use_realsense),
        launch_arguments={
            "enable_color": "true",
            "enable_depth": "true",
            "align_depth.enable": "true",
            "publish_tf": "true",
            "pointcloud.enable": "false",
        }.items(),
    )

    static_camera_tf_node = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_camera_tf_publisher",
        output="screen",
        condition=IfCondition(use_static_camera_tf),
        arguments=[
            "--x",
            static_camera_x,
            "--y",
            static_camera_y,
            "--z",
            static_camera_z,
            "--roll",
            static_camera_roll,
            "--pitch",
            static_camera_pitch,
            "--yaw",
            static_camera_yaw,
            "--frame-id",
            static_camera_parent_frame,
            "--child-frame-id",
            static_camera_child_frame,
        ],
    )

    ompl_planning_pipeline_config = {
        "move_group": {
            "planning_plugin": "ompl_interface/OMPLPlanner",
            "request_adapters": (
                "default_planner_request_adapters/AddTimeOptimalParameterization "
                "default_planner_request_adapters/FixWorkspaceBounds "
                "default_planner_request_adapters/FixStartStateBounds "
                "default_planner_request_adapters/FixStartStateCollision "
                "default_planner_request_adapters/FixStartStatePathConstraints"
            ),
            "start_state_max_bounds_error": 0.1,
        }
    }
    ompl_planning_yaml = load_yaml("gluon_moveit_config", "config/ompl_planning.yaml")
    if ompl_planning_yaml:
        ompl_planning_pipeline_config["move_group"].update(ompl_planning_yaml)

    moveit_simple_controllers_yaml = load_yaml("gluon_moveit_config", "config/moveit_controllers.yaml")
    moveit_controllers = {
        "moveit_simple_controller_manager": moveit_simple_controllers_yaml,
        "moveit_controller_manager": "moveit_simple_controller_manager/MoveItSimpleControllerManager",
    }

    trajectory_execution = {
        "moveit_manage_controllers": True,
        "trajectory_execution.allowed_execution_duration_scaling": 1.2,
        "trajectory_execution.allowed_goal_duration_margin": 0.5,
        "trajectory_execution.allowed_start_tolerance": 0.01,
    }

    planning_scene_monitor_parameters = {
        "publish_planning_scene": True,
        "publish_geometry_updates": True,
        "publish_state_updates": True,
        "publish_transforms_updates": True,
    }

    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        name="move_group",
        output="screen",
        condition=IfCondition(use_move_group),
        parameters=[
            robot_description,
            robot_description_semantic,
            kinematics_yaml,
            ompl_planning_pipeline_config,
            trajectory_execution,
            moveit_controllers,
            planning_scene_monitor_parameters,
        ],
    )

    moveit_rviz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare("gluon_moveit_config"), "launch", "moveit_rviz.launch.py"])
        ),
        condition=IfCondition(use_rviz),
    )

    target_approach_node = Node(
        package="gluon_moveit_config",
        executable="target_approach_node",
        name="target_approach_node",
        output="screen",
        parameters=[
            robot_description,
            robot_description_semantic,
            kinematics_yaml,
            {
                "planning_group": "gluon",
                "target_label": ParameterValue(target_label, value_type=str),
                "score_threshold": ParameterValue(score_threshold, value_type=float),
                "approach_distance": ParameterValue(approach_distance, value_type=float),
                "stop_distance": ParameterValue(stop_distance, value_type=float),
                "dry_run": ParameterValue(dry_run, value_type=bool),
                "position_only_target": ParameterValue(position_only_target, value_type=bool),
                "ee_link": ParameterValue(ee_link, value_type=str),
                "camera_frame": ParameterValue(camera_frame, value_type=str),
                "max_velocity_scaling": ParameterValue(max_velocity_scaling, value_type=float),
                "max_acceleration_scaling": ParameterValue(max_acceleration_scaling, value_type=float),
                "stable_required_frames": 5,
                "stable_position_tolerance": 0.03,
                "z_offset": 0.0,
                "lateral_offset": 0.0,
                "min_x": ParameterValue(min_x, value_type=float),
                "max_x": ParameterValue(max_x, value_type=float),
                "min_y": ParameterValue(min_y, value_type=float),
                "max_y": ParameterValue(max_y, value_type=float),
                "min_z": ParameterValue(min_z, value_type=float),
                "max_z": ParameterValue(max_z, value_type=float),
                "planning_frame": ParameterValue(planning_frame, value_type=str),
            },
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_realsense", default_value="false"),
            DeclareLaunchArgument("use_ros2_control", default_value="true"),
            DeclareLaunchArgument("use_move_group", default_value="true"),
            DeclareLaunchArgument("use_rviz", default_value="true"),
            DeclareLaunchArgument("use_static_camera_tf", default_value="false"),
            DeclareLaunchArgument("target_label", default_value=""),
            DeclareLaunchArgument("score_threshold", default_value="0.5"),
            DeclareLaunchArgument("approach_distance", default_value="0.18"),
            DeclareLaunchArgument("stop_distance", default_value="0.16"),
            DeclareLaunchArgument("dry_run", default_value="true"),
            DeclareLaunchArgument("position_only_target", default_value="true"),
            DeclareLaunchArgument("ee_link", default_value="6_Link"),
            DeclareLaunchArgument("camera_frame", default_value="camera_color_optical_frame"),
            DeclareLaunchArgument("max_velocity_scaling", default_value="0.15"),
            DeclareLaunchArgument("max_acceleration_scaling", default_value="0.15"),
            DeclareLaunchArgument("min_x", default_value="0.05"),
            DeclareLaunchArgument("max_x", default_value="0.70"),
            DeclareLaunchArgument("min_y", default_value="-0.35"),
            DeclareLaunchArgument("max_y", default_value="0.35"),
            DeclareLaunchArgument("min_z", default_value="0.02"),
            DeclareLaunchArgument("max_z", default_value="0.55"),
            DeclareLaunchArgument("static_camera_parent_frame", default_value="base_link"),
            DeclareLaunchArgument("static_camera_child_frame", default_value="camera_link"),
            DeclareLaunchArgument("static_camera_x", default_value="0.0"),
            DeclareLaunchArgument("static_camera_y", default_value="0.0"),
            DeclareLaunchArgument("static_camera_z", default_value="0.0"),
            DeclareLaunchArgument("static_camera_roll", default_value="0.0"),
            DeclareLaunchArgument("static_camera_pitch", default_value="0.0"),
            DeclareLaunchArgument("static_camera_yaw", default_value="0.0"),
            DeclareLaunchArgument("planning_frame", default_value="base_link"),
            LogInfo(msg="target_approach.launch.py: starting optional RealSense, ros2_control, MoveIt, RViz, and target approach node"),
            realsense_launch,
            LogInfo(msg="target_approach.launch.py: publishing static camera TF when use_static_camera_tf=true"),
            static_camera_tf_node,
            LogInfo(msg="target_approach.launch.py: starting ros2_control hardware and controllers when use_ros2_control=true"),
            control_node,
            robot_state_pub_node,
            joint_state_broadcaster_spawner,
            robot_controller_spawner,
            LogInfo(msg="target_approach.launch.py: starting move_group node when use_move_group=true"),
            move_group_node,
            LogInfo(msg="target_approach.launch.py: including moveit_rviz.launch.py when use_rviz=true"),
            moveit_rviz_launch,
            target_approach_node,
        ]
    )
