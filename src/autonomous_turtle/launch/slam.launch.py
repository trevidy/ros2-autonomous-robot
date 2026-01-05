from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[{
                'use_sim_time': True,
                'odom_frame': 'odom',
                'map_frame': 'map',
                'base_frame': 'base_footprint',
                'scan_topic': '/scan',
                'resolution': 0.05,
                'max_laser_range': 3.5,
            }]
        )
    ])