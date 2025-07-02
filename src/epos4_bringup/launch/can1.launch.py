from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    config_path = os.path.join(
        get_package_share_directory('epos4_bringup'),
        'config',
        'can1',
        
    )

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory('canopen_core'),
                    'launch',
                    'canopen1.launch.py'
                )
            ),
            launch_arguments={
                'bus_config': os.path.join(config_path, 'bus.yml'),
                'master_config': os.path.join(config_path, 'master.dcf'),  # Updated path
                # 'master_bin': os.path.join(config_path, 'master.bin'),  # Optional
                'can_interface_name': 'can1'
                
            }.items()
        )
    ])
