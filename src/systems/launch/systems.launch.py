import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    mux_config = os.path.join(get_package_share_directory('systems'), 'config', 'twist_mux.yaml')

    return LaunchDescription([
        Node(package='systems', executable='watchdog', output='screen'),
        Node(package='systems', executable='state_machine', output='screen'),
        Node(
            package='twist_mux',
            executable='twist_mux',
            output='screen',
            parameters=[mux_config],
            remappings=[('cmd_vel_out', '/cmd_vel')],
        ),
    ])
