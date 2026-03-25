#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource

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
    
    # Include Gazebo launch file
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('gazebo_ros'),
                'launch',
                'gazebo.launch.py'
            ])
        ]),
        launch_arguments={
            'world_name': PathJoinSubstitution([
                FindPackageShare('turtlebot3_simulation'),
                'worlds',
                'empty_world.world'
            ]),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'gui': 'true',
            'verbose': 'true'
        }.items()
    )
    
    return LaunchDescription([
        urdf_file,
        use_sim_time,
        gazebo_launch
    ])
