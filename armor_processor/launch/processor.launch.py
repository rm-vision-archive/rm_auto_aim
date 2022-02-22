import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    default_params_file = os.path.join(
        get_package_share_directory('armor_processor'), 'config', 'processor.yaml')

    armor_processor = Node(
        package='armor_processor',
        executable='armor_processor_node',
        output='screen',
        emulate_tty=True,
        parameters=[default_params_file, {
                'debug': True,
        }],
    )

    return LaunchDescription([armor_processor])
