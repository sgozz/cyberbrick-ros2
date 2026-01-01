#!/bin/bash
# Start CyberBrick simulation with MQTT bridge
# Run this inside the Docker container

set -e

echo "=== CyberBrick Simulation Startup ==="

# Source ROS2
source /opt/ros/humble/setup.bash

# Export display (use :0 for Linux X11, :1 for VNC)
export DISPLAY=${DISPLAY:-:0}

# Start Gazebo in background
echo "Starting Gazebo..."
ign gazebo /ros2_ws/src/cyberbrick_description/worlds/cyberbrick_world.sdf -r &
GAZEBO_PID=$!

# Wait for Gazebo to initialize
echo "Waiting for Gazebo to load..."
sleep 10

# Unpause simulation
echo "Unpausing simulation..."
ign service -s /world/cyberbrick_world/control \
	--reqtype ignition.msgs.WorldControl \
	--reptype ignition.msgs.Boolean \
	--timeout 2000 \
	--req 'pause: false' || true

# Find MQTT broker IP (mosquitto container)
MQTT_HOST=${MQTT_HOST:-$(getent hosts mosquitto | awk '{print $1}' 2>/dev/null || echo "172.22.0.2")}
MQTT_PORT=${MQTT_PORT:-1883}

# Start MQTT bridge
echo "Starting MQTT bridge (connecting to $MQTT_HOST:$MQTT_PORT)..."
python3 /ros2_ws/src/cyberbrick_control/scripts/mqtt_bridge.py --host $MQTT_HOST --port $MQTT_PORT &
MQTT_PID=$!

# Wait for bridge to initialize
sleep 2

# Optionally start vision node
if [ "${START_VISION:-false}" = "true" ]; then
    echo "Starting vision node..."
    python3 /ros2_ws/src/cyberbrick_control/scripts/vision_node.py &
    VISION_PID=$!
fi

echo ""
echo "=== Simulation Running ==="
echo "Gazebo PID: $GAZEBO_PID"
echo "MQTT Bridge PID: $MQTT_PID"
[ -n "$VISION_PID" ] && echo "Vision PID: $VISION_PID"
echo ""
echo "=== MQTT Control ==="
echo "Broker: localhost:1883"
echo ""
echo "Arm topics:"
echo "  cyberbrick/arm/base       {\"angle\": 45}      # degrees"
echo "  cyberbrick/arm/shoulder   {\"angle\": 30}      # degrees"
echo "  cyberbrick/arm/elbow      {\"angle\": 45}      # degrees"
echo "  cyberbrick/arm/gripper    {\"open\": true}     # boolean"
echo ""
echo "Truck topics:"
echo "  cyberbrick/truck/cmd      {\"speed\": 0.3, \"steering\": 0.2}  # m/s, rad"
echo "  cyberbrick/truck/stop     {}                  # immediate stop"
echo ""
echo "=== Direct Gazebo Control ==="
echo "  # Arm shoulder"
echo "  ign topic -t /arm/shoulder -m ignition.msgs.Double -p 'data: 0.8'"
echo ""
echo "  # Truck forward"
echo "  ign topic -t /truck/cmd_vel -m ignition.msgs.Twist -p 'linear: {x: 0.3}'"
echo ""
echo "Press Ctrl+C to stop all processes"

# Wait for any process to exit
wait
