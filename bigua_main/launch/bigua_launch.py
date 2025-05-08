## Holocean launch file 
# Author: Braden Meyers

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
# from launch.substitutions import LaunchConfiguration
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory
from pathlib import Path

def generate_launch_description():
    print('Launching Bigua-Sim Vehicle Simulation')

    base = Path(get_package_share_directory('bigua_main'))
    params_file = base / 'config' / 'config.yaml'

    # List contents of the directory to debug
    
    bigua_namespace = 'bigua'

    bigua_main_node = launch_ros.actions.Node(
        name='bigua_node',
        package='bigua_main',
        executable='bigua_node',  
        namespace=bigua_namespace,
        output='screen',
        emulate_tty=True,
        parameters=[{'params_file': str(params_file)}],  # Pass parameters in the correct format
        remappings=[
                ('/bigua/ControlCommand', '/control_command'),
            ]    
        )


    return LaunchDescription([
        bigua_main_node
            # rosbag                            
    ])

