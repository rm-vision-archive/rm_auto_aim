import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_detector_params_file = os.path.join(
        get_package_share_directory('armor_detector'), 'config', 'detector.yaml')

    default_processor_params_file = os.path.join(
        get_package_share_directory('armor_processor'), 'config', 'processor.yaml')

    armor_detector = Node(
        package='armor_detector',
        executable='armor_detector_node',
        output='screen',
        emulate_tty=True,
        parameters=[LaunchConfiguration('detector_params_file'), {
                'debug': LaunchConfiguration('debug'),
                'detect_color': LaunchConfiguration('detect_color'),
        }],
    )

    armor_processor = Node(
        package='armor_processor',
        executable='armor_processor_node',
        output='screen',
        emulate_tty=True,
        parameters=[LaunchConfiguration('processor_params_file'), {
                'debug': LaunchConfiguration('debug'),
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument(name='detector_params_file',
                              default_value=default_detector_params_file),
        DeclareLaunchArgument(name='processor_params_file',
                              default_value=default_processor_params_file),
        DeclareLaunchArgument(name='debug',
                              default_value='true'),
        DeclareLaunchArgument(name='detect_color',
                              default_value='0', description='0-BLUE 1-Red'),
        armor_detector,
        armor_processor
    ])
