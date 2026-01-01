# CyberBrick ROS2 - Agent Context

## Progetto

Simulazione ROS2 + Gazebo per robot CyberBrick (mobile + braccio 3DOF) su Mac M1.
L'obiettivo finale è controllare il robot con comandi vocali e visione artificiale.

## Stato Attuale

### Funzionante
- [x] Docker con VNC per visualizzare Gazebo su Mac M1
- [x] Robot mobile differential drive controllabile via `/cmd_vel`
- [x] Braccio 3DOF con joint position controller (`/arm/joint1`, `/arm/joint2`, `/arm/joint3`)
- [x] Mondo con tavolo e cubi colorati (rosso, blu, verde)
- [x] Container MQTT Mosquitto attivo

### Da Implementare
- [ ] Bridge MQTT funzionante (ROS2 topics ↔ MQTT)
- [ ] Camera simulata sul robot
- [ ] Vision node (rilevamento cubi colorati con OpenCV)
- [ ] Voice node (comandi vocali con Whisper)
- [ ] Gripper funzionante sul braccio
- [ ] Launch file completo
- [ ] Test con hardware reale CyberBrick

## Architettura

```
Mac M1
├── Docker (Colima)
│   ├── cyberbrick-ros2-vnc (ROS2 Humble + Gazebo Fortress)
│   │   ├── VNC server (:5901)
│   │   ├── noVNC web (:6080)
│   │   └── Gazebo simulation
│   └── mosquitto (MQTT broker :1883)
└── Browser → http://localhost:6080/vnc.html
```

## Comandi Utili

### Avviare simulazione
```bash
cd /path/to/cyberbrick-ros2
docker compose -f docker-compose.vnc.yml up -d
# Apri http://localhost:6080/vnc.html
```

### Controllare robot (da terminale host)
```bash
# Muovi robot avanti
docker exec cyberbrick-ros2-vnc bash -c "ign topic -t /cmd_vel -m ignition.msgs.Twist -p 'linear: {x: 0.3}'"

# Ruota robot
docker exec cyberbrick-ros2-vnc bash -c "ign topic -t /cmd_vel -m ignition.msgs.Twist -p 'angular: {z: 1.0}'"

# Stop robot
docker exec cyberbrick-ros2-vnc bash -c "ign topic -t /cmd_vel -m ignition.msgs.Twist -p 'linear: {x: 0}'"
```

### Controllare braccio
```bash
# Ruota base (joint1): -1.57 a 1.57 rad
docker exec cyberbrick-ros2-vnc bash -c "ign topic -t /arm/joint1 -m ignition.msgs.Double -p 'data: 0.5'"

# Spalla (joint2): -0.78 a 2.35 rad
docker exec cyberbrick-ros2-vnc bash -c "ign topic -t /arm/joint2 -m ignition.msgs.Double -p 'data: 1.0'"

# Gomito (joint3): -2.35 a 0.78 rad
docker exec cyberbrick-ros2-vnc bash -c "ign topic -t /arm/joint3 -m ignition.msgs.Double -p 'data: -0.5'"
```

### Riavviare Gazebo
```bash
docker exec cyberbrick-ros2-vnc pkill -f "ign gazebo"
docker exec cyberbrick-ros2-vnc bash -c "export DISPLAY=:1 && ign gazebo /ros2_ws/src/cyberbrick_description/worlds/cyberbrick_world.sdf &"
sleep 5
docker exec cyberbrick-ros2-vnc bash -c "ign service -s /world/cyberbrick_world/control --reqtype ignition.msgs.WorldControl --reptype ignition.msgs.Boolean --timeout 2000 --req 'pause: false'"
```

### Fermare tutto
```bash
docker compose -f docker-compose.vnc.yml down
```

## File Principali

| File | Descrizione |
|------|-------------|
| `worlds/cyberbrick_world.sdf` | Mondo Gazebo con robot, braccio, tavolo, cubi |
| `docker-compose.vnc.yml` | Setup Docker con VNC |
| `docker/Dockerfile.vnc` | Immagine ROS2 + Gazebo + VNC |
| `scripts/mqtt_bridge.py` | Bridge ROS2 ↔ MQTT (da completare) |

## Obiettivo Finale

Sistema completamente simulato:
1. Robot mobile naviga verso il tavolo
2. Camera simulata rileva cubi colorati
3. Comandi vocali: "prendi il cubo rosso"
4. Braccio afferra il cubo

Nessun hardware reale richiesto - tutto in Gazebo.

## Note

- Gazebo Fortress (Ignition) usato al posto di Gazebo Classic per compatibilità ARM64
- VNC necessario perché OpenGL non funziona via XQuartz su Docker Mac
- I plugin usano il naming `libignition-gazebo6-*` specifico per Fortress
