
import os
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription

def generate_launch_description():
    lane_detection = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('lane_detection'),'launch','lane_detection.launch.py'
                )])
    )
    return LaunchDescription([
        lane_detection,
        Node(
            package='v4l2_camera',
            executable='v4l2_camera_node',
            output='screen',
            namespace='camera',
            parameters=[{
                'video_device': '/dev/video0', 
                'image_size': [640,480],
                'time_per_frame': [1, 6],
                'camera_frame_id': 'camera_link_optical'
                }],      
        )
    ])
