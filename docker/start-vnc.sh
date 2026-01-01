#!/bin/bash

# Setup ROS2
source /opt/ros/humble/setup.bash

# Create log directory
mkdir -p /var/log/supervisor

# Start supervisor (manages VNC + noVNC + XFCE)
exec /usr/bin/supervisord -c /etc/supervisor/conf.d/supervisord.conf
