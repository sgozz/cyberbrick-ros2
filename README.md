# CyberBrick ROS2 Simulation

Simulazione del sistema CyberBrick (braccio L-ONE + truck) con Gazebo, controllabile via MQTT.

Sviluppa e testa i tuoi script di controllo senza hardware reale - gli stessi comandi MQTT funzioneranno sia con la simulazione che con il CyberBrick fisico.

## Hardware Simulato

- **Braccio L-ONE**: 4 DOF (rotazione base, spalla, gomito) + gripper
- **CyberBrick Truck**: Sterzo Ackermann (trazione posteriore + sterzo anteriore)
- **Camera Overhead**: Camera fissa sopra il tavolo per rilevamento cubi
- **AI Agent**: Controllo vocale/testuale tramite LLM (Ollama)
- **Ambiente**: Tavolo con cubi colorati (rosso, blu, verde)

## Requisiti

- Docker e Docker Compose
- Linux con X11 oppure Mac con Docker Desktop
- 8GB+ RAM
- (Opzionale) GPU NVIDIA con nvidia-container-toolkit
- Python 3 con `paho-mqtt` per controllare dal PC host
- (Opzionale) Ollama per il controllo AI

## Quick Start

### 1. Clona e avvia i container

```bash
git clone https://github.com/sgozz/cyberbrick-ros2.git
cd cyberbrick-ros2

# Abilita X11 per Docker
xhost +local:docker

# Avvia i container (scegli UNA delle opzioni):

# Opzione A: Linux standard
docker compose -f docker-compose.linux.yml up -d

# Opzione B: Linux con GPU NVIDIA (consigliato se hai una GPU NVIDIA)
docker compose -f docker-compose.nvidia.yml up -d

# Opzione C: Mac M1/M2 (usa VNC)
docker compose -f docker-compose.vnc.yml up -d
```

### 2. Avvia la simulazione

```bash
# Avvia tutto con lo script (metodo semplice)
docker exec -it cyberbrick-ros2 bash /ros2_ws/src/cyberbrick_control/scripts/start_simulation.sh
```

Oppure avvia manualmente passo per passo:

```bash
# Avvia Gazebo
docker exec -d cyberbrick-ros2 bash -c "export DISPLAY=:0 && ign gazebo /ros2_ws/src/cyberbrick_description/worlds/cyberbrick_world.sdf -r"

# Aspetta che Gazebo carichi (10-15 secondi), poi togli la pausa
docker exec cyberbrick-ros2 bash -c "ign service -s /world/cyberbrick_world/control --reqtype ignition.msgs.WorldControl --reptype ignition.msgs.Boolean --timeout 2000 --req 'pause: false'"

# Avvia il bridge MQTT
docker exec -d cyberbrick-ros2 python3 /ros2_ws/src/cyberbrick_control/scripts/mqtt_bridge.py --host 172.22.0.2 --port 1883
```

### 3. Controlla via MQTT

```bash
# Installa paho-mqtt sul tuo PC
pip install paho-mqtt

# Usa lo script di test interattivo
python3 scripts/test_mqtt.py
```

### 4. (Opzionale) Controllo AI con Ollama

```bash
# Installa Ollama (https://ollama.ai)
curl -fsSL https://ollama.ai/install.sh | sh

# Scarica un modello (consigliato: qwen2.5-coder:14b)
ollama pull qwen2.5-coder:14b

# Installa le dipendenze Python
pip install paho-mqtt requests

# Avvia l'AI Agent (dal PC host, non dentro Docker)
python3 scripts/ai_agent.py

# Ora puoi controllare il braccio con comandi naturali:
#   "Sposta il cubo rosso a sinistra"
#   "Move the red cube to the right"
#   "Ordina i cubi per colore"
```

## Controllo via MQTT

### Protocollo

#### Braccio L-ONE

| Topic | Payload | Descrizione |
|-------|---------|-------------|
| `cyberbrick/arm/base` | `{"angle": 45}` | Rotazione base in gradi (-90 a 90) |
| `cyberbrick/arm/shoulder` | `{"angle": 30}` | Spalla in gradi (-30 a 90) |
| `cyberbrick/arm/elbow` | `{"angle": 45}` | Gomito in gradi (-90 a 90) |
| `cyberbrick/arm/gripper` | `{"open": true}` | Apri/chiudi gripper |
| `cyberbrick/arm/move` | `{"base": 0, "shoulder": 30, "elbow": 45}` | Muovi tutti i giunti insieme |

#### Truck

| Topic | Payload | Descrizione |
|-------|---------|-------------|
| `cyberbrick/truck/cmd` | `{"speed": 0.3, "steering": 0.2}` | Velocità (m/s) e sterzo (radianti) |
| `cyberbrick/truck/stop` | `{}` | Stop immediato |

#### Vision (output)

| Topic | Payload | Descrizione |
|-------|---------|-------------|
| `cyberbrick/vision/state` | `{"cubes": [...]}` | Posizione dei cubi rilevati |

#### Arm Controller (input)

| Topic | Payload | Descrizione |
|-------|---------|-------------|
| `cyberbrick/arm/command` | `{"action": "pick", "x": 0.4, "y": 0.1}` | Prendi cubo alla posizione |
| `cyberbrick/arm/command` | `{"action": "place", "x": 0.3, "y": 0.15}` | Rilascia cubo alla posizione |
| `cyberbrick/arm/command` | `{"action": "home"}` | Torna a posizione iniziale |

#### AI Agent (input/output)

| Topic | Payload | Descrizione |
|-------|---------|-------------|
| `cyberbrick/agent/command` | `{"text": "sposta rosso a sinistra"}` | Comando in linguaggio naturale |
| `cyberbrick/agent/response` | `{"text": "Ok, sposto..."}` | Risposta dell'AI |

### Esempio Python

```python
import paho.mqtt.client as mqtt
import json
import time

client = mqtt.Client()
client.connect("localhost", 1883)

# Muovi il braccio
client.publish("cyberbrick/arm/shoulder", json.dumps({"angle": 45}))
time.sleep(1)
client.publish("cyberbrick/arm/base", json.dumps({"angle": 30}))
time.sleep(1)
client.publish("cyberbrick/arm/gripper", json.dumps({"open": False}))

# Muovi il truck
client.publish("cyberbrick/truck/cmd", json.dumps({"speed": 0.3, "steering": 0}))
time.sleep(2)
client.publish("cyberbrick/truck/stop", json.dumps({}))

client.disconnect()
```

### Script di Test Interattivo

```bash
python3 scripts/test_mqtt.py

# Comandi disponibili:
#   base 45        - ruota base a 45 gradi
#   shoulder 30    - spalla a 30 gradi
#   elbow 20       - gomito a 20 gradi
#   grip open      - apri gripper
#   grip close     - chiudi gripper
#   truck 0.3      - vai avanti a 0.3 m/s
#   truck 0.2 0.3  - vai avanti sterzando
#   stop           - ferma truck
#   demo           - esegui sequenza demo
#   quit           - esci
```

## Comandi Docker

### Gestione Container

```bash
# Avvia
docker compose -f docker-compose.nvidia.yml up -d

# Ferma
docker compose -f docker-compose.nvidia.yml down

# Ricostruisci dopo modifiche
docker compose -f docker-compose.nvidia.yml build --no-cache
docker compose -f docker-compose.nvidia.yml up -d

# Entra nel container
docker exec -it cyberbrick-ros2 bash

# Vedi i log
docker logs cyberbrick-ros2
```

### Comandi Gazebo (dentro il container)

```bash
# Avvia Gazebo
ign gazebo /ros2_ws/src/cyberbrick_description/worlds/cyberbrick_world.sdf -r

# Togli pausa
ign service -s /world/cyberbrick_world/control --reqtype ignition.msgs.WorldControl --reptype ignition.msgs.Boolean --timeout 2000 --req 'pause: false'

# Muovi braccio direttamente (radianti)
ign topic -t /arm/base -m ignition.msgs.Double -p 'data: 0.785'
ign topic -t /arm/shoulder -m ignition.msgs.Double -p 'data: 0.5'
ign topic -t /arm/elbow -m ignition.msgs.Double -p 'data: 0.5'
ign topic -t /arm/gripper -m ignition.msgs.Double -p 'data: -0.008'

# Muovi truck direttamente
ign topic -t /truck/cmd_vel -m ignition.msgs.Twist -p 'linear: {x: 0.3}'
ign topic -t /truck/cmd_vel -m ignition.msgs.Twist -p 'linear: {x: 0.3}, angular: {z: 0.2}'
ign topic -t /truck/cmd_vel -m ignition.msgs.Twist -p 'linear: {x: 0}, angular: {z: 0}'
```

## Struttura Progetto

```
cyberbrick-ros2/
├── docker/
│   ├── Dockerfile.linux      # Immagine per Linux (X11)
│   ├── Dockerfile.vnc        # Immagine per Mac (VNC)
│   ├── start-linux.sh        # Script avvio container Linux
│   ├── start-vnc.sh          # Script avvio container Mac
│   └── supervisord.conf      # Configurazione VNC
├── scripts/
│   ├── mqtt_bridge.py        # Bridge MQTT <-> Gazebo
│   ├── vision_bridge.py      # Rilevamento cubi con camera overhead
│   ├── arm_controller.py     # Controller braccio con cinematica inversa
│   ├── ai_agent.py           # AI Agent con Ollama LLM
│   ├── test_mqtt.py          # Script test interattivo
│   └── start_simulation.sh   # Avvia simulazione completa
├── worlds/
│   └── cyberbrick_world.sdf  # Mondo Gazebo (braccio, truck, tavolo, cubi)
├── docker-compose.linux.yml  # Docker Compose per Linux
├── docker-compose.nvidia.yml # Docker Compose per Linux + NVIDIA
├── docker-compose.vnc.yml    # Docker Compose per Mac
├── README.md
└── agent.md                  # Documentazione tecnica
```

## Setup GPU NVIDIA (Linux)

Se hai una GPU NVIDIA, installa il container toolkit per accelerazione hardware:

```bash
# Fedora
curl -s -L https://nvidia.github.io/libnvidia-container/stable/rpm/nvidia-container-toolkit.repo | \
  sudo tee /etc/yum.repos.d/nvidia-container-toolkit.repo
sudo dnf install -y nvidia-container-toolkit
sudo nvidia-ctk runtime configure --runtime=docker
sudo systemctl restart docker

# Ubuntu/Debian
curl -fsSL https://nvidia.github.io/libnvidia-container/gpgkey | \
  sudo gpg --dearmor -o /usr/share/keyrings/nvidia-container-toolkit-keyring.gpg
curl -s -L https://nvidia.github.io/libnvidia-container/stable/deb/nvidia-container-toolkit.list | \
  sed 's#deb https://#deb [signed-by=/usr/share/keyrings/nvidia-container-toolkit-keyring.gpg] https://#g' | \
  sudo tee /etc/apt/sources.list.d/nvidia-container-toolkit.list
sudo apt-get update && sudo apt-get install -y nvidia-container-toolkit
sudo nvidia-ctk runtime configure --runtime=docker
sudo systemctl restart docker

# Poi usa docker-compose.nvidia.yml
docker compose -f docker-compose.nvidia.yml up -d
```

## Troubleshooting

### La simulazione non si muove
```bash
# Verifica che non sia in pausa
docker exec cyberbrick-ros2 bash -c "ign service -s /world/cyberbrick_world/control --reqtype ignition.msgs.WorldControl --reptype ignition.msgs.Boolean --timeout 2000 --req 'pause: false'"
```

### Display non funziona (Linux)
```bash
xhost +local:docker
echo $DISPLAY  # Deve mostrare :0 o :1
```

### MQTT non si connette
```bash
# Verifica IP di mosquitto
docker inspect mosquitto --format '{{range .NetworkSettings.Networks}}{{.IPAddress}}{{end}}'

# Riavvia il bridge con l'IP corretto
docker exec -d cyberbrick-ros2 python3 /ros2_ws/src/cyberbrick_control/scripts/mqtt_bridge.py --host <IP> --port 1883
```

### Errore "runtime nvidia not found"
Installa nvidia-container-toolkit (vedi sezione Setup GPU NVIDIA).

## Prossimi Passi: Hardware Reale

Per collegare il CyberBrick reale, dovrai scrivere codice MicroPython per ESP32-C3 che:

1. Si connette al WiFi
2. Si connette al broker MQTT
3. Si sottoscrive ai topic `cyberbrick/#`
4. Traduce i comandi in controllo servo/motori

I tuoi script Python funzioneranno senza modifiche - basta che il CyberBrick si connetta allo stesso broker MQTT.

## Note Tecniche

- **Gazebo Fortress** (Ignition) per compatibilità ARM64
- **Sterzo Ackermann** per simulazione realistica del truck
- **MQTT** come protocollo di comunicazione (stesso del CyberBrick reale)
- **paho-mqtt** per la comunicazione Python
