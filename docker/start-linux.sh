#!/bin/bash

# Setup ROS2
source /opt/ros/humble/setup.bash

# Keep container running
echo "CyberBrick ROS2 container ready!"
echo "Run: docker exec -it cyberbrick-ros2 bash"
exec tail -f /dev/null
