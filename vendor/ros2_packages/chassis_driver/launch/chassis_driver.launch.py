from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    params_file = PathJoinSubstitution([
        FindPackageShare('chassis_driver'), 'config', 'params.yaml'
    ])
    return LaunchDescription([
        Node(
            package='chassis_driver',
            executable='chassis_node',
            name='chassis_node',
            parameters=[params_file],
            output='screen',
        ),
    ])
