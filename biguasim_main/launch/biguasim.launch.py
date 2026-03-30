## BiguaSim launch file 
# Author: Matheus G. Mateus

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
# from launch.substitutions import LaunchConfiguration
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory
from pathlib import Path

def generate_launch_description():
    print('Launching BiguaSim Vehicle Simulation')

    base = Path(get_package_share_directory('biguasim_main'))
    params_file = base / 'config' / 'config.yaml'

    # List contents of the directory to debug
    
    biguasim_namespace = 'biguasim'

    biguasim_main_node = launch_ros.actions.Node(
        name='biguasim_node',
        package='biguasim_main',
        executable='biguasim_node',  
        namespace=biguasim_namespace,
        output='screen',
        emulate_tty=True,
        parameters=[{'params_file': str(params_file)}],  # Pass parameters in the correct format
        remappings=[
                ('/biguasim/ControlCommand', '/control_command'),
            ]    
        )


    return LaunchDescription([
        biguasim_main_node
            # rosbag                            
    ])

