#!/bin/bash
# Start CyberBrick simulation with Vision + AI Agent
# Run this inside the Docker container

set -e

echo "=== CyberBrick Simulation Startup ==="
echo ""

# Parse arguments
START_AI_AGENT=false
SKIP_GAZEBO=false

for arg in "$@"; do
    case $arg in
        --ai-agent)
            START_AI_AGENT=true
            ;;
        --skip-gazebo)
            SKIP_GAZEBO=true
            ;;
        --help)
            echo "Usage: $0 [OPTIONS]"
            echo ""
            echo "Options:"
            echo "  --ai-agent     Start AI agent with interactive CLI"
            echo "  --skip-gazebo  Skip Gazebo startup (assumes already running)"
            echo "  --help         Show this help"
            exit 0
            ;;
    esac
done

# Source ROS2
source /opt/ros/humble/setup.bash

# Export display (use :0 for Linux X11, :1 for VNC)
export DISPLAY=${DISPLAY:-:0}

# Find MQTT broker IP
MQTT_HOST=${MQTT_HOST:-$(getent hosts mosquitto | awk '{print $1}' 2>/dev/null || echo "172.22.0.2")}
MQTT_PORT=${MQTT_PORT:-1883}

# Script paths
SCRIPT_DIR="/ros2_ws/src/cyberbrick_control/scripts"
WORLD_FILE="/ros2_ws/src/cyberbrick_description/worlds/cyberbrick_world.sdf"

# PIDs for cleanup
declare -a PIDS

cleanup() {
    echo ""
    echo "Shutting down all processes..."
    for pid in "${PIDS[@]}"; do
        if kill -0 "$pid" 2>/dev/null; then
            kill "$pid" 2>/dev/null || true
        fi
    done
    exit 0
}
trap cleanup SIGINT SIGTERM

# Start Gazebo
if [ "$SKIP_GAZEBO" = false ]; then
    echo "Starting Gazebo..."
    ign gazebo "$WORLD_FILE" -r &
    GAZEBO_PID=$!
    PIDS+=($GAZEBO_PID)
    
    echo "Waiting for Gazebo to load..."
    sleep 10
    
    # Unpause simulation
    echo "Unpausing simulation..."
    ign service -s /world/cyberbrick_world/control \
        --reqtype ignition.msgs.WorldControl \
        --reptype ignition.msgs.Boolean \
        --timeout 2000 \
        --req 'pause: false' || true
else
    echo "Skipping Gazebo startup (assuming already running)"
fi

# Start MQTT bridge
echo "Starting MQTT bridge (connecting to $MQTT_HOST:$MQTT_PORT)..."
python3 "$SCRIPT_DIR/mqtt_bridge.py" --host "$MQTT_HOST" --port "$MQTT_PORT" &
MQTT_PID=$!
PIDS+=($MQTT_PID)
sleep 2

# Start Vision bridge
echo "Starting Vision bridge..."
python3 "$SCRIPT_DIR/vision_bridge.py" &
VISION_PID=$!
PIDS+=($VISION_PID)
sleep 2

# Start Arm controller
echo "Starting Arm controller..."
python3 "$SCRIPT_DIR/arm_controller.py" &
ARM_PID=$!
PIDS+=($ARM_PID)
sleep 1

echo ""
echo "=== Simulation Running ==="
[ -n "$GAZEBO_PID" ] && echo "Gazebo PID: $GAZEBO_PID"
echo "MQTT Bridge PID: $MQTT_PID"
echo "Vision Bridge PID: $VISION_PID"
echo "Arm Controller PID: $ARM_PID"
echo ""
echo "=== MQTT Topics ==="
echo ""
echo "Vision (output):"
echo "  cyberbrick/vision/state     {\"cubes\": [{\"color\": \"red\", \"x\": 0.4, \"y\": 0.1, \"visible\": true}, ...]}"
echo ""
echo "Arm Control (input):"
echo "  cyberbrick/arm/command      {\"action\": \"pick\", \"x\": 0.4, \"y\": 0.1}"
echo "  cyberbrick/arm/command      {\"action\": \"place\", \"x\": 0.3, \"y\": 0.15}"
echo "  cyberbrick/arm/command      {\"action\": \"home\"}"
echo ""
echo "Arm Status (output):"
echo "  cyberbrick/arm/status       {\"status\": \"ready\", \"busy\": false}"
echo ""
echo "AI Agent (if running):"
echo "  cyberbrick/agent/command    {\"text\": \"move red cube to the left\"}"
echo "  cyberbrick/agent/response   {\"text\": \"Moving red cube...\"}"
echo ""

# Start AI Agent if requested
if [ "$START_AI_AGENT" = true ]; then
    echo "=== Starting AI Agent ==="
    echo "Connecting to Ollama at localhost:11434..."
    echo ""
    # Run AI agent in foreground (interactive)
    python3 "$SCRIPT_DIR/ai_agent.py"
else
    echo "=== Manual Control Mode ==="
    echo "To start the AI agent, run:"
    echo "  python3 $SCRIPT_DIR/ai_agent.py"
    echo ""
    echo "Or restart with:"
    echo "  $0 --ai-agent"
    echo ""
    echo "Press Ctrl+C to stop all processes"
    
    # Wait for any process to exit
    wait
fi
