from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, FindExecutable
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import Command
from launch_ros.parameter_descriptions import ParameterValue
import os
import yaml
from ament_index_python.packages import get_package_share_directory

def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)
    
    try:
        with open(absolute_file_path, 'r') as file:
            return yaml.safe_load(file)
    except EnvironmentError:
        return None

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

    # Robot Description Semantic
    robot_description_semantic_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare("gluon_moveit_config"), "config", "gluon.srdf"]
            ),
        ]
    )
    robot_description_semantic = {
        "robot_description_semantic": ParameterValue(robot_description_semantic_content,value_type=str)
    }

    # Kinematics
    kinematics_yaml = load_yaml(
        "gluon_moveit_config", "config/kinematics.yaml"
    )

    # RViz
    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("gluon_moveit_config"), "launch", "moveit.rviz"]
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        parameters=[
            robot_description,
            robot_description_semantic,
            kinematics_yaml,
        ],
    )

    return LaunchDescription([rviz_node])
