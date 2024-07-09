import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import xacro

def generate_launch_description():
    # Provide the name of the package and subpath to the xacro file within the package
    pkg_name = 'my_robot_description'
    file_subpath = 'description/my_robot.urdf.xacro'

    # Full path to the xacro file
    xacro_file = os.path.join(get_package_share_directory(pkg_name), file_subpath)
    # Process the xacro file and convert it to an XML string (URDF)
    robot_description_raw = xacro.process_file(xacro_file).toxml()

    # Define the robot_state_publisher node
    node_robot_state_publisher = Node(
        package='robot_state_publisher',      # Name of the package
        executable='robot_state_publisher',   # Executable to be run
        output='screen',                      # Output configuration
        parameters=[{'robot_description': robot_description_raw}]  # Pass the URDF XML to the node
    )

    # Run the robot_state_publisher node
    return LaunchDescription([
        node_robot_state_publisher
    ])
