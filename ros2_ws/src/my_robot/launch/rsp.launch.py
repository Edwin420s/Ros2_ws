import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Path to your URDF file
    urdf_file = os.path.join(get_package_share_directory('my_robot'), 'urdf', 'my_robot.urdf')

    # Read the URDF file
    with open(urdf_file, 'r') as infp:
        robot_description = infp.read()

    # Robot State Publisher Node
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description}]
    )

    return LaunchDescription([
        node_robot_state_publisher
    ])