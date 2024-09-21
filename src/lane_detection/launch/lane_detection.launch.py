import os

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.conditions import UnlessCondition

from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    
 

    detect_lane_node = Node(
            package='lane_detection',
            executable='detect_lane',
            remappings=[('/image_in','/camera/image_raw')]
         )
    detect_sign_node = Node(
            package='lane_detection',
            executable='detect_sign',
            remappings=[('/image_i','/camera/image_raw')]
         )

    return LaunchDescription([
            detect_lane_node,
            detect_sign_node
        ])
