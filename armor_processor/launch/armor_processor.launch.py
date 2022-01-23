import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('armor_processor'), 'config', 'armor_processor.yaml')

    armor_processor = Node(
        name='armor_processor',
        package='armor_processor',
        executable='armor_processor_node',
        namespace='',
        output='screen',
        parameters=[config],
        # Uncomment this line to change log level to DEBUG
        # arguments=['--ros-args', '--log-level', 'DEBUG'],
    )

    # [Warning] : this is only for testing!
    static_transform_publisher = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'shooter_link', 'camera_link']
    )

    return LaunchDescription([armor_processor, static_transform_publisher])
