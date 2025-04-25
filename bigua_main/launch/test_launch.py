## Holocean launch file 
# Author: Braden Meyers

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
# from launch.substitutions import LaunchConfiguration
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory
from pathlib import Path
import os

def generate_launch_description():
    print('Launching Bigua-Sim Vehicle Simulation')

    # Set log level
    log_level = 'info'

    base = Path(get_package_share_directory('bigua_main'))
    params_file = base / 'config' / 'config.yaml'

    log_dir = os.path.join(os.getenv('HOME'), 'ros2ws', 'log')
    # List contents of the directory to debug
    
    bigua_namespace = 'bigua'

    bigua_test_node = launch_ros.actions.Node(
        name='test_node',
        package='bigua_main',
        executable='test_node',  
        namespace=bigua_namespace,
        output='screen',
        emulate_tty=True,
        parameters=[{'params_file': str(params_file)}],  # Pass parameters in the correct format
        remappings=[
                ('/bigua/ControlCommand', '/control_command'),
            ]    
        )


    return LaunchDescription([
        bigua_test_node
        # rosbag                            
    ])

