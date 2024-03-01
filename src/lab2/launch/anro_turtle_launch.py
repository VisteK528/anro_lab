from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='lab2',
            namespace='anro',
            executable='anro_turtle',
            parameters=[
                {"turtle2_name": "Joe"},
                {"turtle3_name": "Bono"}
            ]
        ),
        Node(
            package='turtlesim',
            namespace='anro',
            executable='turtlesim_node',
        )
    ])