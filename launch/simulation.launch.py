#!/usr/bin/env python3
"""Launch file for CyberBrick simulation"""

from launch import LaunchDescription
from launch.actions import ExecuteProcess, DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
import os

def generate_launch_description():
    # Paths
    pkg_path = '/ros2_ws/src/cyberbrick_description'
    urdf_file = os.path.join(pkg_path, 'urdf', 'cyberbrick_robot.urdf.xacro')
    world_file = os.path.join(pkg_path, 'worlds', 'cyberbrick_world.sdf')
    
    return LaunchDescription([
        # Start Gazebo
        ExecuteProcess(
            cmd=['ign', 'gazebo', world_file],
            output='screen'
        ),
        
        # Robot state publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': open('/tmp/robot.urdf').read()}]
        ),
    ])
