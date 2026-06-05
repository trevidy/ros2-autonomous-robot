import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration  
from launch_ros.actions import Node

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')

    joy_params = os.path.join(get_package_share_directory('ros2-autonomous-robot'), 'config', 'joystick.yaml')

    # joy_node = Node(
    #     package = 'joy',
    #     executable='joy_node',
    #     parameters = [joy_params, {'use_sim_time': use_sim_time}],
    # )

    teleop_node = Node (
        package = 'teleop_twist_joy', # Name of the ROS2 package where the node lives. This package must be installed on the computer.
        executable = 'teleop_node', # This is the specific script within the package that should be run. A single package can have multiple executables.
        name = 'teleop_node', # this renames this 'teleop_node = Node' into 'teleop_node'. If not, the name of this node will be 'teleop_twist_joy_node'
        parameters = [joy_params, {'use_sim_time': use_sim_time}], # passes a list of configurations. joy_params is a variable pointing to a .yaml file 
        remappings = [('/cmd_vel', '/diff_cont/cmd_vel_unstamped')]
    )


    # Launch!
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use sim time if true'),
        teleop_node
    ])
