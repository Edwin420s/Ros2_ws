#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Declare arguments
    urdf_file = DeclareLaunchArgument(
        'urdf_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('turtlebot3_simulation'),
            'urdf',
            'turtlebot3_burger.urdf'
        ]),
        description='Path to the robot URDF file'
    )
    
    use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time'
    )
    
    # Get package directory
    pkg_dir = get_package_share_directory('turtlebot3_simulation')
    
    # Read URDF file
    with open(os.path.join(pkg_dir, 'urdf', 'turtlebot3_burger.urdf'), 'r') as f:
        robot_description_content = f.read()
    
    # Robot State Publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[
            {
                'robot_description': robot_description_content,
                'use_sim_time': LaunchConfiguration('use_sim_time')
            }
        ]
    )
    
    # Joint State Publisher
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )
    
    # RViz2
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', PathJoinSubstitution([
            FindPackageShare('turtlebot3_simulation'),
            'rviz',
            'turtlebot3_view.rviz'
        ])],
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        output='screen'
    )
    
    return LaunchDescription([
        urdf_file,
        use_sim_time,
        robot_state_publisher_node,
        joint_state_publisher_node,
        rviz_node
    ])
