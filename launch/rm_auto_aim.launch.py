import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('rm_auto_aim'), 'config', 'default.yaml')

    rm_auto_aim_node = Node(
        name='rm_auto_aim_node',
        package='rm_auto_aim',
        executable='rm_auto_aim_node',
        namespace='',
        output='screen',
        parameters=[config],
        # Uncomment this line to change log level to DEBUG
        # arguments=['--ros-args', '--log-level', 'DEBUG'],
    )

    return LaunchDescription([rm_auto_aim_node])
