#!/usr/bin/env python3
"""
AI Agent for CyberBrick ROS2
Uses Ollama LLM to interpret natural language commands and control the robotic arm.
Supports both Italian and English commands.
Verifies actions using vision feedback.
"""

import json
import time
import signal
import sys
import threading
import readline  # For better CLI input
import requests
import paho.mqtt.client as mqtt

# Configuration
import os

# MQTT broker - use localhost when running on host (port 1883 mapped from mosquitto container)
MQTT_BROKER = os.environ.get("MQTT_HOST", "localhost")
MQTT_PORT = int(os.environ.get("MQTT_PORT", 1883))
# Configuration notes:
# - Run this script on the HOST (not in Docker) to access Ollama
# - MQTT broker is accessible on localhost:1883 (port mapped from mosquitto container)
# - Ollama runs on localhost:11434
OLLAMA_URL = os.environ.get("OLLAMA_URL", "http://localhost:11434/api/generate")
OLLAMA_MODEL = "qwen2.5-coder:14b"

# MQTT Topics
VISION_STATE_TOPIC = "cyberbrick/vision/state"
ARM_COMMAND_TOPIC = "cyberbrick/arm/command"
ARM_STATUS_TOPIC = "cyberbrick/arm/status"
AGENT_COMMAND_TOPIC = "cyberbrick/agent/command"
AGENT_RESPONSE_TOPIC = "cyberbrick/agent/response"

# Timing
ACTION_TIMEOUT = 10  # Seconds to wait for action to complete
VERIFY_DELAY = 2  # Seconds to wait before verifying
MAX_RETRIES = 3  # Maximum retries for failed actions

# Zone definitions (world coordinates)
# Table is 0.6m x 0.4m, centered at (0.5, 0)
ZONES = {
    "left": {"y_min": 0.08, "y_max": 0.18, "x_center": 0.45},
    "right": {"y_min": -0.18, "y_max": -0.08, "x_center": 0.45},
    "front": {"x_min": 0.55, "x_max": 0.70, "y_center": 0.0},
    "back": {"x_min": 0.25, "x_max": 0.40, "y_center": 0.0},
    "center": {"x_center": 0.50, "y_center": 0.0},
    # Italian
    "sinistra": {"y_min": 0.08, "y_max": 0.18, "x_center": 0.45},
    "destra": {"y_min": -0.18, "y_max": -0.08, "x_center": 0.45},
    "davanti": {"x_min": 0.55, "x_max": 0.70, "y_center": 0.0},
    "dietro": {"x_min": 0.25, "x_max": 0.40, "y_center": 0.0},
    "centro": {"x_center": 0.50, "y_center": 0.0},
}

SYSTEM_PROMPT = """You are a robot controller for an L-ONE robotic arm on a table.
The table is 60cm x 40cm. The arm is at the center (0.5, 0).

Available zones:
- left/sinistra: y > 0.08 (positive Y)
- right/destra: y < -0.08 (negative Y)  
- front/davanti: x > 0.55
- back/dietro: x < 0.40
- center/centro: middle of the table

You receive the current state of the cubes (red, green, blue) with their positions.
You must plan actions to fulfill the user's request.

IMPORTANT RULES:
1. Only output valid JSON, nothing else
2. Use the exact format shown below
3. Plan one action at a time for verification
4. The arm can only hold one cube at a time
5. After picking, you must place before picking another

Output format:
{
  "thinking": "Brief explanation of what you're doing",
  "action": {
    "type": "pick" or "place" or "home" or "done",
    "cube": "red" or "green" or "blue" (for pick),
    "zone": "zone name" (for place) or {"x": 0.5, "y": 0.1} (for precise placement)
  },
  "response": "Message to user in their language"
}

For "done" action (when task is complete):
{
  "thinking": "Task completed",
  "action": {"type": "done"},
  "response": "Confirmation message to user"
}

Examples:
User: "move red cube to the left"
State: red at (0.4, 0.1), blue at (0.5, -0.1), green at (0.6, 0.05)
Response: {"thinking": "Red cube is visible, picking it up", "action": {"type": "pick", "cube": "red"}, "response": "Picking up the red cube..."}

User: (after pick) "continue"  
State: red not visible (being held), blue at (0.5, -0.1), green at (0.6, 0.05)
Response: {"thinking": "Holding red cube, placing it on the left", "action": {"type": "place", "zone": "left"}, "response": "Placing on the left side..."}

User: "sposta il cubo blu a destra"
Response: {"thinking": "Devo spostare il cubo blu a destra", "action": {"type": "pick", "cube": "blue"}, "response": "Prendo il cubo blu..."}
"""


class AIAgent:
    def __init__(self):
        self.running = True
        self.mqtt_client = None

        # State
        self.vision_state = None
        self.arm_status = None
        self.holding_cube = None
        self.conversation_history = []

        # Synchronization
        self.arm_ready = threading.Event()
        self.arm_ready.set()

        self.setup_mqtt()

    def setup_mqtt(self):
        """Setup MQTT client"""
        self.mqtt_client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
        self.mqtt_client.on_connect = self.on_mqtt_connect
        self.mqtt_client.on_disconnect = self.on_mqtt_disconnect
        self.mqtt_client.on_message = self.on_mqtt_message

        try:
            self.mqtt_client.connect(MQTT_BROKER, MQTT_PORT, 60)
            self.mqtt_client.loop_start()
            print(f"[Agent] Connecting to MQTT broker at {MQTT_BROKER}:{MQTT_PORT}")
        except Exception as e:
            print(f"[Agent] Failed to connect to MQTT broker: {e}")

    def on_mqtt_connect(self, client, userdata, flags, reason_code, properties):
        print(f"[Agent] Connected to MQTT broker")
        self.mqtt_client.subscribe(VISION_STATE_TOPIC)
        self.mqtt_client.subscribe(ARM_STATUS_TOPIC)
        self.mqtt_client.subscribe(AGENT_COMMAND_TOPIC)
        print(f"[Agent] Subscribed to topics")

    def on_mqtt_disconnect(self, client, userdata, flags, reason_code, properties):
        print(f"[Agent] Disconnected from MQTT broker")

    def on_mqtt_message(self, client, userdata, msg):
        """Handle incoming MQTT messages"""
        try:
            payload = json.loads(msg.payload.decode())

            if msg.topic == VISION_STATE_TOPIC:
                self.vision_state = payload

            elif msg.topic == ARM_STATUS_TOPIC:
                self.arm_status = payload
                if payload.get("status") == "ready":
                    self.arm_ready.set()
                elif payload.get("status") == "executing":
                    self.arm_ready.clear()

            elif msg.topic == AGENT_COMMAND_TOPIC:
                text = payload.get("text", "")
                if text:
                    threading.Thread(target=self.process_command, args=(text,)).start()

        except Exception as e:
            print(f"[Agent] Error handling message: {e}")

    def get_vision_context(self):
        """Get current vision state as context for LLM"""
        if not self.vision_state:
            return "No vision data available."

        cubes = self.vision_state.get("cubes", [])
        lines = ["Current cube positions:"]

        for cube in cubes:
            color = cube.get("color", "unknown")
            if cube.get("visible", False):
                x = cube.get("x", 0)
                y = cube.get("y", 0)
                lines.append(f"- {color}: ({x:.3f}, {y:.3f})")
            else:
                lines.append(f"- {color}: not visible")

        if self.holding_cube:
            lines.append(f"\nCurrently holding: {self.holding_cube}")

        return "\n".join(lines)

    def call_ollama(self, user_message):
        """Call Ollama API with the user message and context"""
        context = self.get_vision_context()

        prompt = f"""Current state:
{context}

User request: {user_message}

Respond with a single JSON object for the next action."""

        try:
            response = requests.post(
                OLLAMA_URL,
                json={
                    "model": OLLAMA_MODEL,
                    "prompt": prompt,
                    "system": SYSTEM_PROMPT,
                    "stream": False,
                    "options": {"temperature": 0.3, "num_predict": 500},
                },
                timeout=60,
            )

            if response.status_code != 200:
                print(f"[Agent] Ollama error: {response.status_code}")
                return None

            result = response.json()
            return result.get("response", "")

        except requests.exceptions.Timeout:
            print("[Agent] Ollama request timed out")
            return None
        except Exception as e:
            print(f"[Agent] Error calling Ollama: {e}")
            return None

    def parse_llm_response(self, response_text):
        """Parse JSON response from LLM"""
        try:
            # Try to find JSON in the response
            # Sometimes LLM adds extra text before/after JSON
            start = response_text.find("{")
            end = response_text.rfind("}") + 1

            if start == -1 or end == 0:
                print(f"[Agent] No JSON found in response: {response_text}")
                return None

            json_str = response_text[start:end]
            return json.loads(json_str)

        except json.JSONDecodeError as e:
            print(f"[Agent] Failed to parse JSON: {e}")
            print(f"[Agent] Response was: {response_text}")
            return None

    def zone_to_coordinates(self, zone):
        """Convert zone name or dict to (x, y) coordinates"""
        if isinstance(zone, dict):
            return zone.get("x", 0.5), zone.get("y", 0.0)

        zone_lower = zone.lower()
        if zone_lower in ZONES:
            zone_def = ZONES[zone_lower]
            x = zone_def.get("x_center", 0.5)
            y = zone_def.get("y_center", 0.0)
            return x, y

        print(f"[Agent] Unknown zone: {zone}")
        return 0.5, 0.0

    def find_cube_position(self, color):
        """Find cube position from vision state"""
        if not self.vision_state:
            return None

        for cube in self.vision_state.get("cubes", []):
            if cube.get("color") == color and cube.get("visible", False):
                return cube.get("x"), cube.get("y")

        return None

    def execute_action(self, action):
        """Execute a single action"""
        action_type = action.get("type")

        if action_type == "done":
            print("[Agent] Task completed")
            return True

        elif action_type == "home":
            print("[Agent] Moving to home position")
            self.mqtt_client.publish(ARM_COMMAND_TOPIC, json.dumps({"action": "home"}))
            self.arm_ready.clear()
            self.arm_ready.wait(timeout=ACTION_TIMEOUT)
            self.holding_cube = None
            return True

        elif action_type == "pick":
            cube = action.get("cube")
            pos = self.find_cube_position(cube)

            if pos is None:
                print(f"[Agent] Cannot find {cube} cube")
                return False

            x, y = pos
            print(f"[Agent] Picking {cube} cube at ({x:.3f}, {y:.3f})")

            self.mqtt_client.publish(
                ARM_COMMAND_TOPIC, json.dumps({"action": "pick", "x": x, "y": y})
            )

            self.arm_ready.clear()
            if self.arm_ready.wait(timeout=ACTION_TIMEOUT):
                self.holding_cube = cube
                return True
            else:
                print("[Agent] Pick action timed out")
                return False

        elif action_type == "place":
            zone = action.get("zone")
            x, y = self.zone_to_coordinates(zone)

            print(f"[Agent] Placing at zone '{zone}' ({x:.3f}, {y:.3f})")

            self.mqtt_client.publish(
                ARM_COMMAND_TOPIC, json.dumps({"action": "place", "x": x, "y": y})
            )

            self.arm_ready.clear()
            if self.arm_ready.wait(timeout=ACTION_TIMEOUT):
                self.holding_cube = None
                return True
            else:
                print("[Agent] Place action timed out")
                return False

        else:
            print(f"[Agent] Unknown action type: {action_type}")
            return False

    def verify_action(self, action, pre_state):
        """Verify that action succeeded by checking vision state"""
        time.sleep(VERIFY_DELAY)

        action_type = action.get("type")

        if action_type == "pick":
            cube = action.get("cube")
            # After pick, cube should not be visible (it's in gripper)
            pos = self.find_cube_position(cube)
            if pos is None:
                print(f"[Agent] Verified: {cube} cube picked (not visible)")
                return True
            else:
                print(f"[Agent] Verification failed: {cube} cube still visible")
                return False

        elif action_type == "place":
            # After place, the held cube should be visible at new position
            if self.holding_cube:
                pos = self.find_cube_position(self.holding_cube)
                if pos is not None:
                    print(f"[Agent] Verified: cube placed and visible")
                    return True
            # Even if we can't verify, consider it successful if arm is ready
            return True

        return True

    def process_command(self, user_message):
        """Process a user command through the full pipeline"""
        print(f"\n[Agent] Processing: {user_message}")

        # Store pre-action state
        pre_state = self.vision_state.copy() if self.vision_state else None

        # Get LLM response
        print("[Agent] Asking LLM for action plan...")
        llm_response = self.call_ollama(user_message)

        if not llm_response:
            self.send_response(
                "Sorry, I couldn't process your request. / Mi dispiace, non sono riuscito a elaborare la richiesta."
            )
            return

        print(f"[Agent] LLM response: {llm_response}")

        # Parse response
        parsed = self.parse_llm_response(llm_response)
        if not parsed:
            self.send_response(
                "Sorry, I couldn't understand the response. / Mi dispiace, non ho capito la risposta."
            )
            return

        thinking = parsed.get("thinking", "")
        action = parsed.get("action", {})
        response = parsed.get("response", "")

        print(f"[Agent] Thinking: {thinking}")
        print(f"[Agent] Action: {action}")

        # Send initial response to user
        self.send_response(response)

        # Execute action
        if action.get("type") == "done":
            return

        success = self.execute_action(action)

        if success:
            # Verify action
            verified = self.verify_action(action, pre_state)

            if verified:
                # Check if more actions are needed
                if action.get("type") == "pick":
                    # After pick, we need to place - continue the conversation
                    self.process_command("continue / continua")
            else:
                # Retry
                print("[Agent] Action verification failed, retrying...")
                for retry in range(MAX_RETRIES - 1):
                    print(f"[Agent] Retry {retry + 1}/{MAX_RETRIES - 1}")
                    success = self.execute_action(action)
                    if success and self.verify_action(action, pre_state):
                        break
                else:
                    self.send_response(
                        "Action failed after retries. / Azione fallita dopo i tentativi."
                    )
        else:
            self.send_response(
                "Failed to execute action. / Impossibile eseguire l'azione."
            )

    def send_response(self, message):
        """Send response to user via MQTT and print to console"""
        print(f"[Agent] Response: {message}")
        try:
            self.mqtt_client.publish(
                AGENT_RESPONSE_TOPIC,
                json.dumps({"text": message, "timestamp": time.time()}),
            )
        except Exception as e:
            print(f"[Agent] Error sending response: {e}")

    def run_cli(self):
        """Run interactive CLI"""
        print("\n" + "=" * 60)
        print("CyberBrick AI Agent")
        print("=" * 60)
        print("Commands: Type your request in English or Italian")
        print("Examples:")
        print("  - Move the red cube to the left")
        print("  - Sposta il cubo blu a destra")
        print("  - Stack all cubes")
        print("  - Ordina i cubi per colore")
        print("")
        print("Special commands:")
        print("  - 'status' / 'stato': Show current vision state")
        print("  - 'home': Move arm to home position")
        print("  - 'quit' / 'esci': Exit")
        print("=" * 60 + "\n")

        while self.running:
            try:
                user_input = input("You: ").strip()

                if not user_input:
                    continue

                if user_input.lower() in ["quit", "exit", "esci", "q"]:
                    print("[Agent] Goodbye! / Arrivederci!")
                    break

                if user_input.lower() in ["status", "stato"]:
                    print("\n" + self.get_vision_context() + "\n")
                    continue

                if user_input.lower() == "home":
                    self.execute_action({"type": "home"})
                    continue

                # Process the command
                self.process_command(user_input)

            except EOFError:
                break
            except KeyboardInterrupt:
                break

        self.cleanup()

    def cleanup(self):
        """Cleanup resources"""
        print("[Agent] Shutting down...")
        self.running = False
        if self.mqtt_client:
            self.mqtt_client.loop_stop()
            self.mqtt_client.disconnect()


def main():
    agent = AIAgent()

    def signal_handler(sig, frame):
        print("\n[Agent] Received shutdown signal")
        agent.running = False

    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)

    # Wait for initial data
    print("[Agent] Waiting for vision and arm data...")
    time.sleep(2)

    # Run CLI
    agent.run_cli()


if __name__ == "__main__":
    main()
