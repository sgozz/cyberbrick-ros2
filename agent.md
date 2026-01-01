# CyberBrick ROS2 - Agent Context

## Project Overview

Simulation of CyberBrick hardware (L-ONE robotic arm + truck) using ROS2 + Gazebo, controllable via MQTT.

The goal is to develop control scripts that work identically with both the simulation and real hardware.

## Current Status

### Working
- [x] L-ONE arm simulation (4 DOF + gripper)
- [x] CyberBrick truck simulation (Ackermann steering)
- [x] MQTT bridge (translates MQTT commands to Gazebo)
- [x] Docker setup for Linux (X11) and Mac (VNC)
- [x] Camera simulation
- [x] Vision node (cube detection)

### To Implement
- [ ] MicroPython code for real CyberBrick ESP32-C3
- [ ] Voice commands (Whisper)
- [ ] Autonomous navigation

## Architecture

```
Python Scripts (your code)
       |
       | MQTT (localhost:1883)
       v
   Mosquitto (Docker)
       |
       | MQTT  
       v
   mqtt_bridge.py
       |
       | ign topic commands
       v
   Gazebo Fortress
   - l_one_arm model
   - cyberbrick_truck model
```

## MQTT Protocol

### Arm Commands

| Topic | Payload | Unit |
|-------|---------|------|
| `cyberbrick/arm/base` | `{"angle": 45}` | degrees |
| `cyberbrick/arm/shoulder` | `{"angle": 30}` | degrees |
| `cyberbrick/arm/elbow` | `{"angle": 45}` | degrees |
| `cyberbrick/arm/gripper` | `{"open": true}` | boolean |
| `cyberbrick/arm/move` | `{"base": 0, "shoulder": 30, "elbow": 45}` | degrees |

### Truck Commands

| Topic | Payload | Unit |
|-------|---------|------|
| `cyberbrick/truck/cmd` | `{"speed": 0.3, "steering": 0.2}` | m/s, rad |
| `cyberbrick/truck/stop` | `{}` | - |

### Feedback Topics

| Topic | Payload |
|-------|---------|
| `cyberbrick/arm/state` | `{"base": 0, "shoulder": 30, "elbow": 45, "gripper_open": true}` |
| `cyberbrick/truck/odom` | `{"x": 0.1, "y": 0.2, "heading": 45}` |

## Gazebo Topics (Internal)

These are the raw Gazebo topics used by the MQTT bridge:

| Gazebo Topic | Type | Description |
|--------------|------|-------------|
| `/arm/base` | `ignition.msgs.Double` | Base rotation (rad) |
| `/arm/shoulder` | `ignition.msgs.Double` | Shoulder angle (rad) |
| `/arm/elbow` | `ignition.msgs.Double` | Elbow angle (rad) |
| `/arm/gripper` | `ignition.msgs.Double` | Gripper position (m) |
| `/truck/cmd_vel` | `ignition.msgs.Twist` | Truck velocity |
| `/truck/odom` | `ignition.msgs.Odometry` | Truck odometry |
| `/camera/image_raw` | `ignition.msgs.Image` | Camera image |

## Key Files

| File | Description |
|------|-------------|
| `worlds/cyberbrick_world.sdf` | Gazebo world with arm, truck, table, cubes |
| `scripts/mqtt_bridge.py` | MQTT to Gazebo bridge |
| `scripts/test_mqtt.py` | Interactive test script |
| `scripts/vision_node.py` | OpenCV cube detection |
| `docker-compose.linux.yml` | Docker setup for Linux |
| `docker-compose.vnc.yml` | Docker setup for Mac |

## Useful Commands

### Start Simulation

```bash
# Linux
xhost +local:docker
docker compose -f docker-compose.linux.yml up -d
docker exec -d cyberbrick-ros2 bash -c "export DISPLAY=:0 && ign gazebo /ros2_ws/src/cyberbrick_description/worlds/cyberbrick_world.sdf -r"
sleep 10
docker exec cyberbrick-ros2 bash -c "ign service -s /world/cyberbrick_world/control --reqtype ignition.msgs.WorldControl --reptype ignition.msgs.Boolean --timeout 2000 --req 'pause: false'"
```

### Start MQTT Bridge

```bash
# Get mosquitto IP
MQTT_IP=$(docker inspect mosquitto --format '{{range .NetworkSettings.Networks}}{{.IPAddress}}{{end}}')

# Start bridge
docker exec -d cyberbrick-ros2 python3 /ros2_ws/src/cyberbrick_control/scripts/mqtt_bridge.py --host $MQTT_IP --port 1883
```

### Test Commands

```bash
# Direct Gazebo commands
docker exec cyberbrick-ros2 bash -c "ign topic -t /arm/shoulder -m ignition.msgs.Double -p 'data: 0.8'"
docker exec cyberbrick-ros2 bash -c "ign topic -t /truck/cmd_vel -m ignition.msgs.Twist -p 'linear: {x: 0.3}'"

# Via MQTT (from host)
python3 -c "
import paho.mqtt.client as mqtt
import json
client = mqtt.Client()
client.connect('localhost', 1883)
client.publish('cyberbrick/arm/shoulder', json.dumps({'angle': 45}))
client.disconnect()
"
```

### Stop Everything

```bash
docker compose -f docker-compose.linux.yml down
```

## Simulated Hardware Specs

### L-ONE Arm

| Joint | Range | Gazebo Topic |
|-------|-------|--------------|
| Base rotation | -90 to 90 deg | `/arm/base` |
| Shoulder | -30 to 90 deg | `/arm/shoulder` |
| Elbow | -90 to 90 deg | `/arm/elbow` |
| Gripper | open/close | `/arm/gripper` |

### CyberBrick Truck

- Type: Ackermann steering (like a car)
- Drive: Rear wheels
- Steering: Front wheels
- Control: `/truck/cmd_vel` (linear.x = speed, angular.z = steering)

## Next Steps for Real Hardware

1. Write MicroPython code for ESP32-C3 that:
   - Connects to WiFi
   - Connects to MQTT broker
   - Subscribes to `cyberbrick/#` topics
   - Controls servos/motors based on received commands

2. The same Python scripts will work with both simulation and real hardware.

## Notes

- Gazebo Fortress (Ignition) is used for ARM64 compatibility
- MQTT broker runs on port 1883 (exposed from Docker)
- Bridge converts degrees to radians for Gazebo
- Shoulder joint needs high PID gains to overcome gravity
