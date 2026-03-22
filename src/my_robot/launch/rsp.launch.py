import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg     = 'my_robot'
    pkg_dir = get_package_share_directory(pkg)

    with open(os.path.join(pkg_dir, 'urdf', 'my_robot.urdf'), 'r') as f:
        robot_description = f.read()

    # ── Core ────────────────────────────────────────────────────────────────
    node_rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description}]
    )

    # arm_controller.py publishes the arm joints; fake_odom.py publishes
    # wheel joints — so we do NOT also start joint_state_publisher (it would
    # conflict and publish zeros on top of our custom data).

    # ── Odometry & wheel spin ────────────────────────────────────────────────
    node_fake_odom = Node(
        package='my_robot',
        executable='fake_odom.py',
        output='screen'
    )

    # ── Motion behaviour ─────────────────────────────────────────────────────
    node_walker = Node(
        package='my_robot',
        executable='walker.py',
        output='screen'
    )

    # ── Navigation & pick coordination ───────────────────────────────────────
    node_pick_controller = Node(
        package='my_robot',
        executable='pick_controller.py',
        output='screen'
    )

    # ── Arm pick-and-place ───────────────────────────────────────────────────
    node_arm = Node(
        package='my_robot',
        executable='arm_controller.py',
        output='screen'
    )

    # ── Sensors ──────────────────────────────────────────────────────────────
    node_sensors = Node(
        package='my_robot',
        executable='sensor_simulator.py',
        output='screen'
    )

    return LaunchDescription([
        node_rsp,
        node_fake_odom,
        node_walker,
        node_pick_controller,
        node_arm,
        node_sensors,
    ])