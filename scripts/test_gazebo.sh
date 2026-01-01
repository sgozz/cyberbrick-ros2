#!/bin/bash
# Test script to launch Ignition Gazebo with the world

echo "Starting Ignition Gazebo..."
echo "World file: /ros2_ws/src/cyberbrick_description/worlds/cyberbrick_world.sdf"

# Source ROS2
source /opt/ros/humble/setup.bash

# Launch Ignition Gazebo (Fortress)
ign gazebo /ros2_ws/src/cyberbrick_description/worlds/cyberbrick_world.sdf
