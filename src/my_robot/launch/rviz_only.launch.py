import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg = 'my_robot'
    pkg_dir = get_package_share_directory(pkg)
    
    # Load URDF
    with open(os.path.join(pkg_dir, 'urdf', 'my_robot.urdf'), 'r') as f:
        robot_description = f.read()
    
    # Core nodes for RViz visualization
    nodes = [
        # Robot state publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description}]
        ),
        
        # Odometry and wheel animation
        Node(
            package='my_robot',
            executable='fake_odom.py',
            output='screen'
        ),
        
        # Motion behavior
        Node(
            package='my_robot',
            executable='walker.py',
            output='screen'
        ),
        
        # Navigation and pick coordination
        Node(
            package='my_robot',
            executable='pick_controller.py',
            output='screen'
        ),
        
        # Arm pick-and-place
        Node(
            package='my_robot',
            executable='arm_controller.py',
            output='screen'
        ),
        
        # Sensors (LiDAR + IMU + Object detection)
        Node(
            package='my_robot',
            executable='sensor_simulator.py',
            output='screen'
        ),
        
        # Services
        Node(
            package='my_robot',
            executable='robot_services.py',
            output='screen'
        ),
        
        # Actions
        Node(
            package='my_robot',
            executable='robot_actions.py',
            output='screen'
        ),
    ]
    
    return LaunchDescription(nodes)
