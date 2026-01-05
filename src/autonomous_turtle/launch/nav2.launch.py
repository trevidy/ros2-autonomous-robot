import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    pkg_share = get_package_share_directory('autonomous_turtle')
    nav2_params = os.path.join(pkg_share, 'config', 'nav2_params.yaml')
    
    # Get the path to the actual Nav2 bringup launch file
    nav2_launch_dir = os.path.join(get_package_share_directory('nav2_bringup'), 'launch')

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(nav2_launch_dir, 'navigation_launch.py')),
            launch_arguments={
                'params_file': nav2_params,
                'use_sim_time': 'True'
            }.items()
        )
    ])