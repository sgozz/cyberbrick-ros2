#!/bin/bash
# Start CyberBrick simulation with vision
# Run this inside the Docker container

set -e

echo "=== CyberBrick Simulation Startup ==="

# Source ROS2
source /opt/ros/humble/setup.bash

# Export display for Gazebo
export DISPLAY=:1

# Start Gazebo in background
echo "Starting Gazebo..."
ign gazebo /ros2_ws/src/cyberbrick_description/worlds/cyberbrick_world.sdf &
GAZEBO_PID=$!

# Wait for Gazebo to initialize
sleep 5

# Unpause simulation
echo "Unpausing simulation..."
ign service -s /world/cyberbrick_world/control \
	--reqtype ignition.msgs.WorldControl \
	--reptype ignition.msgs.Boolean \
	--timeout 2000 \
	--req 'pause: false' || true

# Start ROS-Ignition bridge
echo "Starting ROS-Ignition bridge..."
ros2 run ros_gz_bridge parameter_bridge \
	/camera/image_raw@sensor_msgs/msg/Image@ignition.msgs.Image \
	/cmd_vel@geometry_msgs/msg/Twist@ignition.msgs.Twist \
	/arm/joint1@std_msgs/msg/Float64@ignition.msgs.Double \
	/arm/joint2@std_msgs/msg/Float64@ignition.msgs.Double \
	/arm/joint3@std_msgs/msg/Float64@ignition.msgs.Double &
BRIDGE_PID=$!

# Wait for bridge to initialize
sleep 2

# Start vision node
echo "Starting vision node..."
python3 /ros2_ws/src/cyberbrick_control/scripts/vision_node.py &
VISION_PID=$!

echo ""
echo "=== Simulation Running ==="
echo "Gazebo PID: $GAZEBO_PID"
echo "Bridge PID: $BRIDGE_PID"
echo "Vision PID: $VISION_PID"
echo ""
echo "View at: http://localhost:6080/vnc.html"
echo ""
echo "Useful commands:"
echo "  # Move robot forward"
echo "  ros2 topic pub /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.3}}' --once"
echo ""
echo "  # Move arm joint1"
echo "  ros2 topic pub /arm/joint1 std_msgs/msg/Float64 '{data: 0.5}' --once"
echo ""
echo "  # View detected objects"
echo "  ros2 topic echo /detected_objects"
echo ""
echo "Press Ctrl+C to stop all processes"

# Wait for any process to exit
wait
