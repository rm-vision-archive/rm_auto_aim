import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    default_params_file = os.path.join(
        get_package_share_directory('armor_detector'), 'config', 'detector.yaml')

    armor_detector = Node(
        package='armor_detector',
        executable='armor_detector_node',
        namespace='',
        output='screen',
        emulate_tty=True,
        parameters=[default_params_file, {
                'debug': True,
                'detect_color': 0,
        }],
    )

    return LaunchDescription([armor_detector])
