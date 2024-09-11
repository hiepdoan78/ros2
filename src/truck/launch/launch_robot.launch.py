import os

from ament_index_python.packages import get_package_share_directory


from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessStart
from launch_ros.actions import Node


def generate_launch_description():

    package_name='truck' 

    rsp = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(package_name),'launch','rsp.launch.py'
                )]), launch_arguments={'use_sim_time': 'false', 'use_ros2_control': 'true'}.items()
    )

    robot_description = Command(['ros2 param get --hide-type /robot_state_publisher robot_description'])

    controller_params_file = os.path.join(get_package_share_directory(package_name),'config','my_controllers.yaml')
    

    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[{'robot_description': robot_description},
                    controller_params_file],
         remappings=[('bicycle_steering_controller/odometry', '/odom'),
                    ('bicycle_steering_controller/tf_odometry', '/tf')]
    )
    
    delayed_controller_manager = TimerAction(period=3.0, actions=[controller_manager])
    
    bicycle_steering_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["bicycle_steering_controller"],
    )
    delayed_bicycle_steering_controller_spawner = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=controller_manager,
            on_start=[bicycle_steering_controller_spawner],
        )
    )
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
    )

    delayed_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=controller_manager,
            on_start=[joint_state_broadcaster_spawner],
        )
    )
    
    serial_controller = Node(
            package='truck', 
            executable='serial_controller.py', 
            name='serial_controller',
            output='screen'
            )
    joystick = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(package_name),'launch','joystick.launch.py'
                )])
    )

    twist_mux_params = os.path.join(get_package_share_directory(package_name),'config','twist_mux.yaml')
    twist_mux = Node(
            package="twist_mux",
            executable="twist_mux",
            parameters=[twist_mux_params],
            remappings=[('/cmd_vel_out','/bicycle_steering_controller/reference')]
        )

    rplidar = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(package_name),'launch','rplidar.launch.py'
                )])
    )

    camera = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(package_name),'launch','camera.launch.py'
                )])
    )
    
    rviz_config_file = os.path.join(get_package_share_directory(package_name), 'config', 'drive_truck.rviz')
    rviz2 = ExecuteProcess(
        cmd=['rviz2', '-d', rviz_config_file],
        output='screen'
    )
   
    # Launch them all!
    return LaunchDescription([
        rsp,
        rplidar,
        # camera,
        twist_mux,
        rviz2,
        delayed_controller_manager,
        delayed_bicycle_steering_controller_spawner,
        delayed_joint_state_broadcaster_spawner,
        serial_controller,
        joystick,
    ])