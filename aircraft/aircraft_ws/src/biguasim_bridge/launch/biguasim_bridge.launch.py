import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    params_file = os.path.join(
        get_package_share_directory('biguasim_bridge'), 'config', 't2_bridge_params.yaml'
    )

    return LaunchDescription([
        Node(
            package='biguasim_bridge',
            executable='biguasim_bridge',
            name='biguasim_bridge',
            output='screen',
            parameters=[params_file],
        ),
    ])
