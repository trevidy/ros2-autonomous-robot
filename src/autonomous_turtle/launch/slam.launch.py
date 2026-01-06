from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description(): # Math behind how the laser beams are calculated into a map
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
                'max_laser_range': 10.0,      # INCREASE THIS (e.g., to 10.0)
                
                'minimum_time_interval': 0.1, # Caps the laser processing to 10Hz so CPU won't get overwhelmed.
                'max_condition_number': 0.5,  # Measure of how "stable" the scan matching is. In an empty world, keeping this around 0.5 keeps the robot "sane" when it has no walls to look at
                'use_scan_matching': True, # Instead of relying only on odometry, this refines the robot's estimated position (pose) and the map concurrently by aligning the new sensor data with existing map data
                'do_loop_closure': True, # Recognizes past map data and pieces it together with current data to fix any drifting errors that accumulated.
                'minimum_distance_for_gauge_score': 0.1, # Prevents updating until the robot has moved 0.1 meters. This prevents the map from jittering.
                'map_update_interval': 1.0,   # How often (seconds) to update the map in RViz
            }]
        )
    ])