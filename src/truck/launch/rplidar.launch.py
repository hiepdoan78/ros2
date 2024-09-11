import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    serial_baudrate = LaunchConfiguration('serial_baudrate', default='460800')
    inverted = LaunchConfiguration('inverted', default='false')
 
    return LaunchDescription([
        Node(
            package='rplidar_ros',
            executable='rplidar_composition',
            output='screen',
            parameters=[{
                'chanel_type': 'serial',
                'serial_port': '/dev/ttyUSB0',
                'frame_id': 'laser',
                'angle_compensate': True,
                'scan_mode': 'Standard',
                'inverted': inverted, 
                'serial_baudrate': serial_baudrate
            }]
        )
    ])