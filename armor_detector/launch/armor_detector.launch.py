import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('armor_detector'), 'config', 'armor_detector.yaml')

    armor_detector = Node(
        name='armor_detector',
        package='armor_detector',
        executable='armor_detector_node',
        namespace='',
        output='screen',
        parameters=[config],
        # Uncomment this line to change log level to DEBUG
        # arguments=['--ros-args', '--log-level', 'DEBUG'],
    )

    return LaunchDescription([armor_detector])
