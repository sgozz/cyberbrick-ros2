# CyberBrick ROS2 Simulation

Simulazione completa del sistema CyberBrick con Gazebo su Mac M1.

## Architettura

```
┌─────────────────────────────────────────────────────────────────┐
│                         DOCKER                                   │
│  ┌─────────────────────────────────────────────────────────┐    │
│  │                    ROS2 Humble                           │    │
│  │                                                          │    │
│  │  ┌──────────┐  ┌──────────┐  ┌────────────────────────┐ │    │
│  │  │ Gazebo   │  │  RViz    │  │  Control Nodes         │ │    │
│  │  │          │  │          │  │  - mqtt_bridge.py      │ │    │
│  │  │ 🤖 Robot │  │ Visualize│  │  - vision_node.py      │ │    │
│  │  │ 🦾 Arm   │  │          │  │  - voice_node.py       │ │    │
│  │  │ 📷 Cam   │  │          │  │                        │ │    │
│  │  └──────────┘  └──────────┘  └────────────────────────┘ │    │
│  └─────────────────────────────────────────────────────────┘    │
│                              │                                   │
│                              │ MQTT                              │
│                              ▼                                   │
│                      ┌──────────────┐                           │
│                      │  Mosquitto   │                           │
│                      │  (broker)    │                           │
│                      └──────────────┘                           │
└─────────────────────────────────────────────────────────────────┘
                               │
          ┌────────────────────┼────────────────────┐
          │                    │                    │
          ▼                    ▼                    ▼
   ┌─────────────┐     ┌─────────────┐     ┌─────────────┐
   │ CyberBrick  │     │ CyberBrick  │     │  ESP32-CAM  │
   │   Robot     │     │    Arm      │     │   (reale)   │
   │  (reale)    │     │  (reale)    │     │             │
   └─────────────┘     └─────────────┘     └─────────────┘
```

## Requisiti

- Mac M1 con 16GB RAM
- Docker Desktop per Mac
- XQuartz (per GUI)
- ~10GB spazio disco

## Installazione

### 1. Installa XQuartz (per visualizzare Gazebo/RViz)

```bash
brew install --cask xquartz
```

**Riavvia il Mac** dopo l'installazione.

### 2. Configura XQuartz

```bash
# Apri XQuartz
open -a XQuartz

# In XQuartz: Preferences → Security → ✅ "Allow connections from network clients"
```

### 3. Installa Docker Desktop

Scarica da: https://www.docker.com/products/docker-desktop/

### 4. Clona e avvia

```bash
cd /Users/fabio/dev/cyberbrick-ros2

# Prima volta: build dell'immagine (10-15 min)
docker compose build

# Avvia tutto
docker compose up
```

## Uso

### Avviare la simulazione

```bash
# Terminal 1: Avvia i container
docker compose up

# Terminal 2: Entra nel container ROS2
docker exec -it cyberbrick-ros2 bash

# Dentro il container: lancia Gazebo
ros2 launch cyberbrick_description simulation.launch.py
```

### Controllare il robot simulato

```bash
# Teleop da tastiera
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=/cyberbrick/cmd_vel

# Oppure invia comandi diretti
ros2 topic pub /cyberbrick/cmd_vel geometry_msgs/Twist "{linear: {x: 0.5}, angular: {z: 0.0}}"
```

### Visualizzare in RViz

```bash
ros2 run rviz2 rviz2
```

### Attivare il bridge MQTT (per robot reale)

```bash
ros2 run cyberbrick_control mqtt_bridge.py
```

## Struttura progetto

```
cyberbrick-ros2/
├── docker/
│   ├── Dockerfile           # Immagine ROS2 + Gazebo
│   └── ros_entrypoint.sh    # Script avvio
├── docker-compose.yml       # Orchestrazione container
├── urdf/
│   ├── cyberbrick_robot.urdf.xacro  # Modello robot mobile
│   └── cyberbrick_arm.urdf.xacro    # Modello braccio 3DOF
├── worlds/
│   └── cyberbrick_world.sdf  # Mondo Gazebo con oggetti
├── launch/
│   └── simulation.launch.py  # Launch file principale
├── config/
│   ├── mosquitto.conf        # Config broker MQTT
│   └── arm_controllers.yaml  # Controller braccio
├── scripts/
│   ├── mqtt_bridge.py        # Bridge ROS2 ↔ MQTT
│   ├── vision_node.py        # Nodo visione OpenCV
│   └── voice_node.py         # Nodo comandi vocali
└── README.md
```

## Topics ROS2

| Topic | Tipo | Descrizione |
|:------|:-----|:------------|
| `/cyberbrick/cmd_vel` | Twist | Velocità robot |
| `/cyberbrick/odom` | Odometry | Posizione robot |
| `/cyberbrick/camera/image_raw` | Image | Immagine camera |
| `/arm/joint_states` | JointState | Stato servo braccio |
| `/arm/command` | String | Comandi braccio (JSON) |

## Da simulazione a reale

Il codice funziona **identicamente** su:

1. **Simulazione**: Gazebo riceve comandi via ROS2 topics
2. **Reale**: `mqtt_bridge.py` converte topics → MQTT → CyberBrick

```bash
# Solo simulazione
ros2 launch cyberbrick_description simulation.launch.py

# Simulazione + robot reale (entrambi si muovono!)
ros2 launch cyberbrick_description simulation.launch.py
ros2 run cyberbrick_control mqtt_bridge.py
```

## Troubleshooting

### Gazebo non si apre

```bash
# Verifica XQuartz
xhost +localhost

# Verifica DISPLAY
echo $DISPLAY  # Deve mostrare qualcosa tipo ":0"
```

### Container non parte

```bash
# Ricostruisci l'immagine
docker compose build --no-cache

# Controlla i log
docker compose logs -f
```

### MQTT non connette

```bash
# Verifica che Mosquitto sia attivo
docker compose ps

# Testa connessione
mosquitto_pub -h localhost -t test -m "hello"
```

## Prossimi passi

1. **Aggiungi vision_node.py** - Rilevamento oggetti colorati
2. **Aggiungi voice_node.py** - Comandi vocali con Whisper
3. **Crea launch file completo** - Avvia tutto insieme
4. **Integra con robot reale** - Testa con CyberBrick fisico
