import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command

def generate_launch_description():
    package_name = 'my_robot_slam'
    xacro_file = os.path.join(
        get_package_share_directory(package_name),
        'description',
        'my_robot.urdf.xacro'
    )
    
    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': Command(['xacro ', xacro_file])}]
        ),
        Node(
            package='slam_toolbox',
            executable='sync_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[os.path.join(
                get_package_share_directory(package_name),
                'config',
                'slam_toolbox_params.yaml'
            )],
            remappings=[
                ('scan', '/my_robot/laser_scan/center'),
                ('odom', 'odom'),
                ('map', 'map')
            ]
        )
    ])
