# CyberBrick ROS2 Simulation

Simulazione completa del sistema CyberBrick con Gazebo su Mac M1.

## Requisiti

- Mac M1/M2 con 8GB+ RAM
- Docker Desktop oppure Colima
- Browser web (per VNC)

## Quick Start

```bash
# Clona il repo
git clone https://github.com/sgozz/cyberbrick-ros2.git
cd cyberbrick-ros2

# Build dell'immagine (prima volta: ~10-15 min)
docker compose -f docker-compose.vnc.yml build

# Avvia i container
docker compose -f docker-compose.vnc.yml up -d

# Apri nel browser
open http://localhost:6080/vnc.html
# Password VNC: password

# Avvia simulazione completa (Gazebo + Vision)
docker exec -it cyberbrick-ros2-vnc bash /ros2_ws/src/cyberbrick_control/scripts/start_simulation.sh
```

## Architettura

```
Mac M1
├── Docker
│   ├── cyberbrick-ros2-vnc (ROS2 Humble + Gazebo Fortress + OpenCV)
│   │   ├── VNC server (:5901)
│   │   ├── noVNC web (:6080) ──► Browser
│   │   ├── Gazebo simulation
│   │   └── Vision node (cube detection)
│   └── mosquitto (MQTT broker :1883)
└── Browser → http://localhost:6080/vnc.html
```

**Tutto simulato - nessun hardware reale richiesto!**

## Comandi

### Controllare il robot
```bash
# Avanti
docker exec cyberbrick-ros2-vnc bash -c "ign topic -t /cmd_vel -m ignition.msgs.Twist -p 'linear: {x: 0.3}'"

# Ruota
docker exec cyberbrick-ros2-vnc bash -c "ign topic -t /cmd_vel -m ignition.msgs.Twist -p 'angular: {z: 1.0}'"

# Stop
docker exec cyberbrick-ros2-vnc bash -c "ign topic -t /cmd_vel -m ignition.msgs.Twist -p 'linear: {x: 0}'"
```

### Controllare il braccio
```bash
# Joint 1 (base): -1.57 a 1.57 rad
docker exec cyberbrick-ros2-vnc bash -c "ign topic -t /arm/joint1 -m ignition.msgs.Double -p 'data: 0.5'"

# Joint 2 (spalla): -0.78 a 2.35 rad  
docker exec cyberbrick-ros2-vnc bash -c "ign topic -t /arm/joint2 -m ignition.msgs.Double -p 'data: 1.0'"

# Joint 3 (gomito): -2.35 a 0.78 rad
docker exec cyberbrick-ros2-vnc bash -c "ign topic -t /arm/joint3 -m ignition.msgs.Double -p 'data: -0.5'"
```

### Testare la visione
```bash
# Visualizza oggetti rilevati dalla camera
docker exec cyberbrick-ros2-vnc bash -c "source /opt/ros/humble/setup.bash && ros2 topic echo /detected_objects"
```

## Struttura Progetto

```
cyberbrick-ros2/
├── docker/
│   ├── Dockerfile.vnc      # ROS2 + Gazebo + VNC + OpenCV
│   ├── supervisord.conf    # Configurazione servizi VNC
│   └── start-vnc.sh        # Script avvio container
├── docker-compose.vnc.yml  # Orchestrazione container
├── worlds/
│   └── cyberbrick_world.sdf  # Mondo Gazebo (robot, braccio, camera, cubi)
├── scripts/
│   ├── vision_node.py        # Rilevamento cubi colorati (OpenCV)
│   └── start_simulation.sh   # Avvia Gazebo + bridge + vision
├── launch/
│   └── cyberbrick_sim.launch.py  # Launch file ROS2
├── README.md
└── agent.md                  # Contesto tecnico per AI agents
```

## Stato del Progetto

### Funzionante
- [x] Docker con VNC per visualizzare Gazebo su Mac M1
- [x] Robot mobile differential drive (`/cmd_vel`)
- [x] Braccio 3DOF con joint controller (`/arm/joint1`, `/arm/joint2`, `/arm/joint3`)
- [x] Mondo con tavolo e cubi colorati (rosso, blu, verde)
- [x] Camera simulata sul robot (`/camera/image_raw`)
- [x] Vision node con OpenCV (`/detected_objects`)
- [x] Script di avvio completo

### Da Implementare
- [ ] Voice node (comandi vocali con Whisper)
- [ ] Gripper funzionante sul braccio
- [ ] Navigazione autonoma verso cubi rilevati
- [ ] Test con hardware reale CyberBrick

## Obiettivo Finale

Sistema completamente simulato dove puoi dire "prendi il cubo rosso" e:
1. Robot naviga verso il tavolo
2. Camera rileva la posizione del cubo rosso
3. Braccio si muove per afferrarlo

## Troubleshooting

### VNC non si connette
```bash
# Verifica che il container sia attivo
docker ps | grep cyberbrick

# Riavvia il container
docker compose -f docker-compose.vnc.yml restart
```

### Gazebo non parte
```bash
# Entra nel container e avvia manualmente
docker exec -it cyberbrick-ros2-vnc bash
export DISPLAY=:1
ign gazebo /ros2_ws/src/cyberbrick_description/worlds/cyberbrick_world.sdf
```

### Rebuild dopo modifiche
```bash
docker compose -f docker-compose.vnc.yml build --no-cache
docker compose -f docker-compose.vnc.yml up -d
```

## Note Tecniche

- **Gazebo Fortress** (Ignition) invece di Gazebo Classic per compatibilità ARM64
- **VNC** necessario perché OpenGL non funziona via XQuartz su Docker Mac
- I plugin usano il naming `libignition-gazebo6-*` specifico per Fortress

## Sviluppo

Vedi [agent.md](agent.md) per dettagli tecnici e comandi utili per continuare lo sviluppo.
