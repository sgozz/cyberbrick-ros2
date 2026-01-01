#!/usr/bin/env python3
"""
CyberBrick Simulation Launch File
Starts Gazebo, ROS-Ignition bridge, and vision node
"""

from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # World file path
    world_file = '/ros2_ws/src/cyberbrick_description/worlds/cyberbrick_world.sdf'
    
    # Start Gazebo (Ignition Fortress)
    gazebo = ExecuteProcess(
        cmd=['ign', 'gazebo', world_file, '-r'],  # -r starts simulation running
        output='screen',
        additional_env={'DISPLAY': ':1'}
    )
    
    # ROS-Ignition Bridge for camera
    # Bridges /camera/image_raw from Ignition to ROS2
    ros_gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='ros_gz_bridge',
        arguments=[
            # Camera image: Ignition -> ROS2
            '/camera/image_raw@sensor_msgs/msg/Image@ignition.msgs.Image',
            # Cmd_vel: ROS2 -> Ignition (bidirectional for teleop)
            '/cmd_vel@geometry_msgs/msg/Twist@ignition.msgs.Twist',
            # Arm joints: ROS2 -> Ignition
            '/arm/joint1@std_msgs/msg/Float64@ignition.msgs.Double',
            '/arm/joint2@std_msgs/msg/Float64@ignition.msgs.Double',
            '/arm/joint3@std_msgs/msg/Float64@ignition.msgs.Double',
        ],
        output='screen'
    )
    
    # Vision node (delayed to wait for camera)
    vision_node = Node(
        package='cyberbrick_control',
        executable='vision_node.py',
        name='vision_node',
        output='screen'
    )
    
    # Delay vision node start to allow Gazebo to initialize
    delayed_vision = TimerAction(
        period=5.0,
        actions=[vision_node]
    )
    
    return LaunchDescription([
        gazebo,
        ros_gz_bridge,
        delayed_vision
    ])
