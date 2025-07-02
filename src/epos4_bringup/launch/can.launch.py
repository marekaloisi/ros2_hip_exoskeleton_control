from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    launch_left = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory('epos4_bringup'),
                    'can0.launch.py'
                )
            )   
        )
    launch_right = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory('epos4_bringup'),
                    'can1.launch.py'
                )
            )
            
        )


    return LaunchDescription([
        launch_left,
        launch_right

    ])
