import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    params_file = os.path.join(get_package_share_directory(
        'auto_aim_bringup'), 'config/default.yaml')

    detector_node = Node(
        package='armor_detector',
        executable='armor_detector_node',
        emulate_tty=True,
        output='screen',
        parameters=[params_file],
        ros_arguments=['--log-level', 'armor_detector:=DEBUG'],
    )

    processor_node = Node(
        package='armor_processor',
        executable='armor_processor_node',
        output='screen',
        emulate_tty=True,
        parameters=[params_file],
        ros_arguments=['--log-level', 'armor_processor:=DEBUG'],
    )

    return LaunchDescription([
        detector_node,
        processor_node,
    ])
