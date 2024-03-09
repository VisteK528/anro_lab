from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    arg1 = DeclareLaunchArgument('turtle1_name', default_value='Rico',
                                 description='Name of the first turtle to be spawned')
    arg2 = DeclareLaunchArgument('turtle2_name', default_value='Kowalsky',
                                 description='Name of the second turtle to be spawned')

    return LaunchDescription([
        arg1,
        arg2,
        Node(
            package='turtlesim',
            namespace='anro_lab2',
            executable='turtlesim_node',
        ),
        Node(
            package='lab2_v2',
            namespace='anro_lab2',
            executable='anro_turtle',
            parameters=[
                {"turtle1_name": LaunchConfiguration("turtle1_name")},
                {"turtle2_name": LaunchConfiguration("turtle2_name")}
            ]
        )
    ])