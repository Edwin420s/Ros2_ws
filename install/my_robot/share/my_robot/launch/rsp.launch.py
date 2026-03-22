import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    pkg_name = 'my_robot'
    urdf_file = os.path.join(get_package_share_directory(pkg_name), 'urdf', 'my_robot.urdf')

    with open(urdf_file, 'r') as infp:
        robot_description = infp.read()

    # Node to publish robot structure [cite: 5]
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description}]
    )

    # Node to provide joint data for visualization
    node_joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        output='screen'
    )

    # Node to provide fake odometry
    node_fake_odom = Node(
        package='my_robot',
        executable='fake_odom.py',
        output='screen'
    )

    # Node for arm control
    node_arm_controller = Node(
        package='my_robot',
        executable='arm_controller.py',
        output='screen'
    )

    # Node for sensor simulation
    node_sensor_simulator = Node(
        package='my_robot',
        executable='sensor_simulator.py',
        output='screen'
    )

    # Node for pick control logic
    node_pick_controller = Node(
        package='my_robot',
        executable='pick_controller.py',
        output='screen'
    )

    return LaunchDescription([
        node_robot_state_publisher,
        node_joint_state_publisher,
        node_fake_odom,
        node_arm_controller,
        node_sensor_simulator,
        node_pick_controller
    ])