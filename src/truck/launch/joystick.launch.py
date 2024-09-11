from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    joy_params = os.path.join(get_package_share_directory('truck'),'config','joystick.yaml')

    joy_node = Node(
            package='joy',
            executable='joy_node',
            parameters=[joy_params]
         )

    teleop_node = Node(
            package='teleop_twist_joy',
            executable='teleop_node',
            name='teleop_node',
            parameters=[joy_params]
         )

    twist_stamper = Node(
        package='twist_stamper',
        executable='twist_stamper',
        remappings=[
            ('/cmd_vel_in', '/cmd_vel'),
            ('/cmd_vel_out', '/bicycle_steering_controller/reference')
        ]
    )

    return LaunchDescription([
        joy_node,
        teleop_node,
        twist_stamper       
    ])