from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    arg1 = DeclareLaunchArgument('turtle2_name', default_value='Rico', description='Name of the first spawned turtle')
    arg2 = DeclareLaunchArgument('turtle3_name', default_value='Kowalsky', description='Name of the second spawned turtle')

    return LaunchDescription([
        arg1,
        arg2,
        Node(
            package='turtlesim',
            namespace='anro',
            executable='turtlesim_node',
        ),
        Node(
            package='lab2',
            namespace='anro',
            executable='anro_turtle',
            parameters=[
                {"turtle2_name": LaunchConfiguration("turtle2_name")},
                {"turtle3_name": LaunchConfiguration("turtle3_name")}
            ]
        )
    ])