# CyberBrick ROS2 Simulation

Complete simulation of the CyberBrick system with Gazebo, controllable via MQTT.

This allows you to develop and test your control scripts without real hardware - the same MQTT commands will work with both the simulation and the real CyberBrick robot.

## Simulated Hardware

- **L-ONE Robotic Arm**: 4 DOF (base rotation, shoulder, elbow) + gripper
- **CyberBrick Truck**: Ackermann steering (rear drive + front steering)
- **Camera**: Simulated camera on the truck
- **Environment**: Table with colored cubes (red, blue, green)

## Requirements

### Linux
- Docker and Docker Compose
- X11 (display server)
- 8GB+ RAM recommended
- Python 3 with `paho-mqtt` for controlling from host

### Mac M1/M2
- Docker Desktop or Colima
- Web browser (for VNC)
- 8GB+ RAM
- Python 3 with `paho-mqtt` for controlling from host

## Quick Start

### Linux (Native X11)

```bash
# Clone the repo
git clone https://github.com/sgozz/cyberbrick-ros2.git
cd cyberbrick-ros2

# Allow X11 connections from the container
xhost +local:docker

# Build and start
docker compose -f docker-compose.linux.yml build
docker compose -f docker-compose.linux.yml up -d

# Start Gazebo simulation
docker exec -d cyberbrick-ros2 bash -c "export DISPLAY=:0 && ign gazebo /ros2_ws/src/cyberbrick_description/worlds/cyberbrick_world.sdf -r"

# Wait for Gazebo to load, then unpause simulation
sleep 10
docker exec cyberbrick-ros2 bash -c "ign service -s /world/cyberbrick_world/control --reqtype ignition.msgs.WorldControl --reptype ignition.msgs.Boolean --timeout 2000 --req 'pause: false'"

# Start MQTT bridge (connects simulation to MQTT)
docker exec -d cyberbrick-ros2 python3 /ros2_ws/src/cyberbrick_control/scripts/mqtt_bridge.py --host 172.22.0.2 --port 1883
```

### Mac M1/M2 (VNC)

```bash
# Clone the repo
git clone https://github.com/sgozz/cyberbrick-ros2.git
cd cyberbrick-ros2

# Build and start
docker compose -f docker-compose.vnc.yml build
docker compose -f docker-compose.vnc.yml up -d

# Open in browser
open http://localhost:6080/vnc.html
# VNC Password: password

# Start simulation (in VNC terminal or via docker exec)
docker exec -it cyberbrick-ros2-vnc bash /ros2_ws/src/cyberbrick_control/scripts/start_simulation.sh
```

## Controlling via MQTT

The simulation is controlled via MQTT, using the same protocol that will work with real CyberBrick hardware.

### Install paho-mqtt on your host

```bash
pip install paho-mqtt
```

### MQTT Protocol

#### Arm Control

| Topic | Payload | Description |
|-------|---------|-------------|
| `cyberbrick/arm/base` | `{"angle": 45}` | Base rotation in degrees (-90 to 90) |
| `cyberbrick/arm/shoulder` | `{"angle": 30}` | Shoulder angle in degrees (-30 to 90) |
| `cyberbrick/arm/elbow` | `{"angle": 45}` | Elbow angle in degrees (-90 to 90) |
| `cyberbrick/arm/gripper` | `{"open": true}` | Open or close gripper |
| `cyberbrick/arm/move` | `{"base": 0, "shoulder": 30, "elbow": 45}` | Move multiple joints at once |

#### Truck Control

| Topic | Payload | Description |
|-------|---------|-------------|
| `cyberbrick/truck/cmd` | `{"speed": 0.3, "steering": 0.2}` | Speed (m/s) and steering (radians) |
| `cyberbrick/truck/stop` | `{}` | Immediate stop |

#### Feedback (from simulation)

| Topic | Payload | Description |
|-------|---------|-------------|
| `cyberbrick/arm/state` | `{"base": 0, "shoulder": 30, "elbow": 45, "gripper_open": true}` | Current arm state |
| `cyberbrick/truck/odom` | `{"x": 0.1, "y": 0.2, "heading": 45}` | Truck position |

### Python Example

```python
import paho.mqtt.client as mqtt
import json
import time

# Connect to MQTT broker (running in Docker)
client = mqtt.Client()
client.connect("localhost", 1883)

# Move arm
client.publish("cyberbrick/arm/shoulder", json.dumps({"angle": 45}))
time.sleep(1)

client.publish("cyberbrick/arm/base", json.dumps({"angle": 30}))
time.sleep(1)

client.publish("cyberbrick/arm/gripper", json.dumps({"open": False}))
time.sleep(1)

# Move truck
client.publish("cyberbrick/truck/cmd", json.dumps({"speed": 0.3, "steering": 0}))
time.sleep(2)

client.publish("cyberbrick/truck/stop", json.dumps({}))

client.disconnect()
```

### Interactive Test Script

```bash
# Run the interactive test script
python3 scripts/test_mqtt.py --host localhost --port 1883

# Or run a demo sequence
python3 scripts/test_mqtt.py --host localhost --port 1883 --demo
```

## Architecture

```
Your Python Scripts
       |
       | MQTT (localhost:1883)
       v
+------------------+
|    Mosquitto     |  <-- MQTT Broker (Docker)
+------------------+
       |
       | MQTT
       v
+------------------+
|   MQTT Bridge    |  <-- Translates MQTT to Gazebo commands
+------------------+
       |
       | ign topic
       v
+------------------+
|     Gazebo       |  <-- Physics simulation
|  - L-ONE Arm     |
|  - Truck         |
|  - Camera        |
+------------------+
```

## Direct Gazebo Control (Advanced)

You can also control the simulation directly via Gazebo topics:

```bash
CONTAINER=cyberbrick-ros2  # or cyberbrick-ros2-vnc on Mac

# Arm - base rotation (radians)
docker exec $CONTAINER bash -c "ign topic -t /arm/base -m ignition.msgs.Double -p 'data: 0.785'"

# Arm - shoulder (radians)
docker exec $CONTAINER bash -c "ign topic -t /arm/shoulder -m ignition.msgs.Double -p 'data: 0.5'"

# Arm - elbow (radians)
docker exec $CONTAINER bash -c "ign topic -t /arm/elbow -m ignition.msgs.Double -p 'data: 0.5'"

# Arm - gripper (negative = close, positive = open)
docker exec $CONTAINER bash -c "ign topic -t /arm/gripper -m ignition.msgs.Double -p 'data: -0.008'"

# Truck - forward
docker exec $CONTAINER bash -c "ign topic -t /truck/cmd_vel -m ignition.msgs.Twist -p 'linear: {x: 0.3}'"

# Truck - steer
docker exec $CONTAINER bash -c "ign topic -t /truck/cmd_vel -m ignition.msgs.Twist -p 'linear: {x: 0.2}, angular: {z: 0.3}'"

# Truck - stop
docker exec $CONTAINER bash -c "ign topic -t /truck/cmd_vel -m ignition.msgs.Twist -p 'linear: {x: 0}, angular: {z: 0}'"
```

## Project Structure

```
cyberbrick-ros2/
├── docker/
│   ├── Dockerfile.linux      # ROS2 + Gazebo (Linux, native X11)
│   ├── Dockerfile.vnc        # ROS2 + Gazebo + VNC (Mac)
│   ├── supervisord.conf      # VNC services configuration
│   ├── start-linux.sh        # Linux container startup script
│   └── start-vnc.sh          # Mac container startup script
├── docker-compose.linux.yml  # Container orchestration (Linux)
├── docker-compose.vnc.yml    # Container orchestration (Mac)
├── worlds/
│   └── cyberbrick_world.sdf  # Gazebo world (L-ONE arm, truck, camera, cubes)
├── scripts/
│   ├── mqtt_bridge.py        # MQTT to Gazebo bridge
│   ├── test_mqtt.py          # Interactive MQTT test script
│   ├── vision_node.py        # Colored cube detection (OpenCV)
│   └── start_simulation.sh   # Start Gazebo + bridge + vision
├── launch/
│   └── cyberbrick_sim.launch.py  # ROS2 launch file
├── README.md
└── agent.md                  # Technical context for AI agents
```

## Project Status

### Working
- [x] L-ONE robotic arm simulation (4 DOF + gripper)
- [x] CyberBrick truck simulation (Ackermann steering)
- [x] MQTT control interface
- [x] Docker with native display on Linux (X11)
- [x] Docker with VNC for Mac M1
- [x] World with table and colored cubes
- [x] Simulated camera on the truck
- [x] Vision node with OpenCV

### To Implement
- [ ] Voice node (voice commands with Whisper)
- [ ] Autonomous navigation to detected cubes
- [ ] MicroPython code for real CyberBrick hardware
- [ ] Integration with real CyberBrick via MQTT

## Connecting Real CyberBrick Hardware

When you have real CyberBrick hardware, you'll need to:

1. Write MicroPython code on the CyberBrick ESP32-C3 that:
   - Connects to WiFi
   - Subscribes to the same MQTT topics
   - Translates commands to servo/motor control

2. Your control scripts will work unchanged - just ensure the CyberBrick connects to the same MQTT broker.

```
Your Python Scripts
       |
       | MQTT
       v
   Mosquitto
       |
       +---> Simulation (Gazebo)
       |
       +---> Real CyberBrick (ESP32-C3 via WiFi)
```

## Troubleshooting

### Simulation not moving

Make sure the simulation is not paused:
```bash
docker exec cyberbrick-ros2 bash -c "ign service -s /world/cyberbrick_world/control --reqtype ignition.msgs.WorldControl --reptype ignition.msgs.Boolean --timeout 2000 --req 'pause: false'"
```

### MQTT bridge not receiving messages

Check the MQTT broker IP inside Docker network:
```bash
docker inspect mosquitto --format '{{range .NetworkSettings.Networks}}{{.IPAddress}}{{end}}'
```

Then restart the bridge with the correct IP:
```bash
docker exec -d cyberbrick-ros2 python3 /ros2_ws/src/cyberbrick_control/scripts/mqtt_bridge.py --host <IP> --port 1883
```

### Display not working (Linux)

```bash
xhost +local:docker
echo $DISPLAY  # Should show :0 or :1
```

### Rebuild after changes

```bash
# Linux
docker compose -f docker-compose.linux.yml down
docker compose -f docker-compose.linux.yml build --no-cache
docker compose -f docker-compose.linux.yml up -d

# Mac
docker compose -f docker-compose.vnc.yml down
docker compose -f docker-compose.vnc.yml build --no-cache
docker compose -f docker-compose.vnc.yml up -d
```

## Technical Notes

- **Gazebo Fortress** (Ignition) for ARM64 compatibility
- **Ackermann steering** for realistic truck simulation
- **MQTT** as the communication protocol (same as real CyberBrick will use)
- **paho-mqtt** Python library for MQTT communication

## Development

See [agent.md](agent.md) for technical details and useful commands for AI-assisted development.
