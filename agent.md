# CyberBrick ROS2 - Agent Context

Documentazione tecnica per sviluppo assistito da AI.

## Overview

Simulazione CyberBrick (braccio L-ONE + truck) con Gazebo Fortress, controllabile via MQTT.

## Stato Attuale

### Funzionante
- [x] Braccio L-ONE simulato (4 DOF + gripper)
- [x] Truck CyberBrick simulato (sterzo Ackermann)
- [x] Bridge MQTT (traduce comandi MQTT in comandi Gazebo)
- [x] Docker per Linux (X11 e NVIDIA GPU)
- [x] Docker per Mac (VNC)
- [x] Camera simulata
- [x] Vision node (rilevamento cubi)

### Da Implementare
- [ ] Codice MicroPython per ESP32-C3 reale
- [ ] Comandi vocali (Whisper)
- [ ] Navigazione autonoma

## Architettura

```
Script Python (tuo codice)
       |
       | MQTT (localhost:1883)
       v
   Mosquitto (Docker)
       |
       | MQTT  
       v
   mqtt_bridge.py
       |
       | ign topic
       v
   Gazebo Fortress
   - l_one_arm
   - cyberbrick_truck
```

## Protocollo MQTT

### Comandi Braccio

| Topic | Payload | Unità |
|-------|---------|-------|
| `cyberbrick/arm/base` | `{"angle": 45}` | gradi |
| `cyberbrick/arm/shoulder` | `{"angle": 30}` | gradi |
| `cyberbrick/arm/elbow` | `{"angle": 45}` | gradi |
| `cyberbrick/arm/gripper` | `{"open": true}` | bool |
| `cyberbrick/arm/move` | `{"base": 0, "shoulder": 30, "elbow": 45}` | gradi |

### Comandi Truck

| Topic | Payload | Unità |
|-------|---------|-------|
| `cyberbrick/truck/cmd` | `{"speed": 0.3, "steering": 0.2}` | m/s, rad |
| `cyberbrick/truck/stop` | `{}` | - |

### Feedback

| Topic | Payload |
|-------|---------|
| `cyberbrick/arm/state` | `{"base": 0, "shoulder": 30, "elbow": 45, "gripper_open": true}` |
| `cyberbrick/truck/odom` | `{"x": 0.1, "y": 0.2, "heading": 45}` |

## Topic Gazebo (Interni)

Usati dal bridge MQTT internamente:

| Topic | Tipo | Descrizione |
|-------|------|-------------|
| `/arm/base` | `ignition.msgs.Double` | Rotazione base (rad) |
| `/arm/shoulder` | `ignition.msgs.Double` | Spalla (rad) |
| `/arm/elbow` | `ignition.msgs.Double` | Gomito (rad) |
| `/arm/gripper` | `ignition.msgs.Double` | Gripper (m) |
| `/truck/cmd_vel` | `ignition.msgs.Twist` | Velocità truck |
| `/camera/image_raw` | `ignition.msgs.Image` | Immagine camera |

## File Principali

| File | Descrizione |
|------|-------------|
| `worlds/cyberbrick_world.sdf` | Mondo Gazebo con braccio, truck, tavolo, cubi |
| `scripts/mqtt_bridge.py` | Bridge MQTT → Gazebo |
| `scripts/test_mqtt.py` | Script test interattivo |
| `scripts/start_simulation.sh` | Avvia simulazione completa |
| `scripts/vision_node.py` | Rilevamento cubi OpenCV |

## Comandi Utili

### Avvia Simulazione

```bash
# Avvia container
xhost +local:docker
docker compose -f docker-compose.nvidia.yml up -d

# Avvia tutto
docker exec -it cyberbrick-ros2 bash /ros2_ws/src/cyberbrick_control/scripts/start_simulation.sh
```

### Avvio Manuale

```bash
# Gazebo
docker exec -d cyberbrick-ros2 bash -c "export DISPLAY=:0 && ign gazebo /ros2_ws/src/cyberbrick_description/worlds/cyberbrick_world.sdf -r"

# Unpause (dopo 10 sec)
docker exec cyberbrick-ros2 bash -c "ign service -s /world/cyberbrick_world/control --reqtype ignition.msgs.WorldControl --reptype ignition.msgs.Boolean --timeout 2000 --req 'pause: false'"

# Bridge MQTT
MQTT_IP=$(docker inspect mosquitto --format '{{range .NetworkSettings.Networks}}{{.IPAddress}}{{end}}')
docker exec -d cyberbrick-ros2 python3 /ros2_ws/src/cyberbrick_control/scripts/mqtt_bridge.py --host $MQTT_IP --port 1883
```

### Test

```bash
# Test interattivo
python3 scripts/test_mqtt.py

# Test diretto Gazebo
docker exec cyberbrick-ros2 bash -c "ign topic -t /arm/shoulder -m ignition.msgs.Double -p 'data: 0.8'"
```

### Stop

```bash
docker compose -f docker-compose.nvidia.yml down
```

## Specifiche Hardware Simulato

### Braccio L-ONE

| Giunto | Range | Topic |
|--------|-------|-------|
| Base | -90 to 90 deg | `/arm/base` |
| Spalla | -30 to 90 deg | `/arm/shoulder` |
| Gomito | -90 to 90 deg | `/arm/elbow` |
| Gripper | aperto/chiuso | `/arm/gripper` |

### Truck CyberBrick

- Tipo: Sterzo Ackermann
- Trazione: Ruote posteriori
- Sterzo: Ruote anteriori
- Controllo: `/truck/cmd_vel` (linear.x = velocità, angular.z = sterzo)

## Note

- Gazebo Fortress per compatibilità ARM64
- MQTT broker su porta 1883 (esposta da Docker)
- Bridge converte gradi → radianti per Gazebo
- Spalla richiede PID gains alti per vincere la gravità
