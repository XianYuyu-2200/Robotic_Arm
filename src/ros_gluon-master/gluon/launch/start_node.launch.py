from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='gluon',
            executable='gluon_node',
            name='gluon_node',
            output='screen',
            parameters=[
                # Add parameters here if any
            ]
        )
    ])
