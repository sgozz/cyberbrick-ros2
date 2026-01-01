#!/usr/bin/env python3
"""
MQTT Bridge for CyberBrick Simulation

Translates MQTT messages to Gazebo Ignition commands.
This allows controlling the simulation with the same MQTT protocol
that will be used with the real CyberBrick hardware.

MQTT Topics (input):
  - cyberbrick/truck/cmd      {"speed": 0.5, "steering": 0.2}  (m/s, rad)
  - cyberbrick/truck/stop     {}
  - cyberbrick/arm/base       {"angle": 45}    (degrees)
  - cyberbrick/arm/shoulder   {"angle": 30}    (degrees)
  - cyberbrick/arm/elbow      {"angle": 45}    (degrees)
  - cyberbrick/arm/gripper    {"open": true}
  - cyberbrick/arm/move       {"base": 0, "shoulder": 30, "elbow": 45}  (degrees)

MQTT Topics (output):
  - cyberbrick/truck/odom     {"x": 0.1, "y": 0.2, "heading": 45}
  - cyberbrick/arm/state      {"base": 0, "shoulder": 30, "elbow": 45, "gripper_open": true}
"""

import json
import math
import subprocess
import threading
import time
import paho.mqtt.client as mqtt


class MQTTBridge:
    def __init__(self, mqtt_host="localhost", mqtt_port=1883):
        self.mqtt_host = mqtt_host
        self.mqtt_port = mqtt_port

        # Current state
        self.arm_state = {
            "base": 0.0,
            "shoulder": 0.0,
            "elbow": 0.0,
            "gripper_open": True,
        }
        self.truck_odom = {"x": 0.0, "y": 0.0, "heading": 0.0}

        # MQTT client
        self.client = mqtt.Client(client_id="cyberbrick_bridge")
        self.client.on_connect = self.on_connect
        self.client.on_message = self.on_message

        # State publishing thread
        self.running = False
        self.state_thread = None

    def on_connect(self, client, userdata, flags, rc):
        if rc == 0:
            print("[MQTT Bridge] Connected to MQTT broker")
            # Subscribe to all cyberbrick topics
            client.subscribe("cyberbrick/#")
            print("[MQTT Bridge] Subscribed to cyberbrick/#")
        else:
            print(f"[MQTT Bridge] Connection failed with code {rc}")

    def on_message(self, client, userdata, msg):
        try:
            topic = msg.topic

            # Ignore messages we produce or that are handled by other components
            ignored_topics = [
                "cyberbrick/arm/state",
                "cyberbrick/truck/odom",
                "cyberbrick/vision/state",  # Produced by vision_bridge
                "cyberbrick/arm/command",  # Handled by arm_controller
                "cyberbrick/arm/status",  # Produced by arm_controller
                "cyberbrick/agent/command",  # Handled by ai_agent
                "cyberbrick/agent/response",  # Produced by ai_agent
            ]
            if topic in ignored_topics:
                return

            payload = json.loads(msg.payload.decode()) if msg.payload else {}

            print(f"[MQTT Bridge] Received: {topic} -> {payload}")

            # Route message to handler
            if topic == "cyberbrick/truck/cmd":
                self.handle_truck_cmd(payload)
            elif topic == "cyberbrick/truck/stop":
                self.handle_truck_stop()
            elif topic == "cyberbrick/arm/base":
                self.handle_arm_base(payload)
            elif topic == "cyberbrick/arm/shoulder":
                self.handle_arm_shoulder(payload)
            elif topic == "cyberbrick/arm/elbow":
                self.handle_arm_elbow(payload)
            elif topic == "cyberbrick/arm/gripper":
                self.handle_arm_gripper(payload)
            elif topic == "cyberbrick/arm/move":
                self.handle_arm_move(payload)
            # Silently ignore unknown topics (may be for other components)
            # else:
            #     print(f"[MQTT Bridge] Unknown topic: {topic}")

        except json.JSONDecodeError as e:
            print(f"[MQTT Bridge] JSON decode error: {e}")
        except Exception as e:
            print(f"[MQTT Bridge] Error handling message: {e}")

    def send_ign_cmd(self, topic, msg_type, data):
        """Send command to Ignition Gazebo via ign topic"""
        cmd = ["ign", "topic", "-t", topic, "-m", msg_type, "-p", data]
        print(f"[MQTT Bridge] Executing: {' '.join(cmd)}")
        try:
            result = subprocess.run(cmd, capture_output=True, timeout=2, text=True)
            if result.returncode != 0:
                print(f"[MQTT Bridge] Command failed: {result.stderr}")
            else:
                print(f"[MQTT Bridge] Command sent successfully to {topic}")
        except subprocess.TimeoutExpired:
            print(f"[MQTT Bridge] Timeout sending to {topic}")
        except Exception as e:
            print(f"[MQTT Bridge] Error sending to {topic}: {e}")

    def degrees_to_radians(self, degrees):
        """Convert degrees to radians"""
        return degrees * math.pi / 180.0

    def radians_to_degrees(self, radians):
        """Convert radians to degrees"""
        return radians * 180.0 / math.pi

    # --- Truck handlers ---

    def handle_truck_cmd(self, payload):
        """Handle truck movement command"""
        speed = payload.get("speed", 0.0)  # m/s
        steering = payload.get(
            "steering", 0.0
        )  # rad or could be degrees, let's use rad

        # Send to Ackermann steering plugin
        # linear.x = speed, angular.z = steering angle
        data = f"linear: {{x: {speed}}}, angular: {{z: {steering}}}"
        self.send_ign_cmd("/truck/cmd_vel", "ignition.msgs.Twist", data)

    def handle_truck_stop(self):
        """Stop the truck immediately"""
        data = "linear: {x: 0}, angular: {z: 0}"
        self.send_ign_cmd("/truck/cmd_vel", "ignition.msgs.Twist", data)

    # --- Arm handlers ---

    def handle_arm_base(self, payload):
        """Handle arm base rotation"""
        angle_deg = payload.get("angle", 0.0)
        angle_rad = self.degrees_to_radians(angle_deg)

        # Clamp to limits (-90 to 90 degrees)
        angle_rad = max(-1.57, min(1.57, angle_rad))

        self.send_ign_cmd("/arm/base", "ignition.msgs.Double", f"data: {angle_rad}")
        self.arm_state["base"] = angle_deg

    def handle_arm_shoulder(self, payload):
        """Handle arm shoulder movement"""
        angle_deg = payload.get("angle", 0.0)
        angle_rad = self.degrees_to_radians(angle_deg)

        # Clamp to limits (-30 to 90 degrees approximately)
        angle_rad = max(-0.5, min(1.57, angle_rad))

        self.send_ign_cmd("/arm/shoulder", "ignition.msgs.Double", f"data: {angle_rad}")
        self.arm_state["shoulder"] = angle_deg

    def handle_arm_elbow(self, payload):
        """Handle arm elbow movement"""
        angle_deg = payload.get("angle", 0.0)
        angle_rad = self.degrees_to_radians(angle_deg)

        # Clamp to limits (-90 to 90 degrees)
        angle_rad = max(-1.57, min(1.57, angle_rad))

        self.send_ign_cmd("/arm/elbow", "ignition.msgs.Double", f"data: {angle_rad}")
        self.arm_state["elbow"] = angle_deg

    def handle_arm_gripper(self, payload):
        """Handle gripper open/close - single servo controls both fingers"""
        is_open = payload.get("open", True)

        # Both fingers use same value:
        # Positive = fingers move apart (open)
        # Negative = fingers move together (close)
        if is_open:
            pos = 0.005  # open
        else:
            pos = -0.01  # close

        self.send_ign_cmd("/arm/gripper_left", "ignition.msgs.Double", f"data: {pos}")
        self.send_ign_cmd("/arm/gripper_right", "ignition.msgs.Double", f"data: {pos}")
        self.arm_state["gripper_open"] = is_open

    def handle_arm_move(self, payload):
        """Handle moving all arm joints at once"""
        if "base" in payload:
            self.handle_arm_base({"angle": payload["base"]})
        if "shoulder" in payload:
            self.handle_arm_shoulder({"angle": payload["shoulder"]})
        if "elbow" in payload:
            self.handle_arm_elbow({"angle": payload["elbow"]})
        if "gripper_open" in payload:
            self.handle_arm_gripper({"open": payload["gripper_open"]})

    # --- State publishing ---

    def publish_state(self):
        """Publish current state to MQTT"""
        while self.running:
            try:
                # Publish arm state
                self.client.publish("cyberbrick/arm/state", json.dumps(self.arm_state))

                # Publish truck odometry (TODO: read from Gazebo)
                self.client.publish(
                    "cyberbrick/truck/odom", json.dumps(self.truck_odom)
                )

            except Exception as e:
                print(f"[MQTT Bridge] Error publishing state: {e}")

            time.sleep(0.1)  # 10 Hz

    def start(self):
        """Start the bridge"""
        print(f"[MQTT Bridge] Connecting to {self.mqtt_host}:{self.mqtt_port}")

        try:
            self.client.connect(self.mqtt_host, self.mqtt_port, 60)
        except Exception as e:
            print(f"[MQTT Bridge] Failed to connect: {e}")
            return False

        # Start state publishing thread
        self.running = True
        self.state_thread = threading.Thread(target=self.publish_state, daemon=True)
        self.state_thread.start()

        # Start MQTT loop (blocking)
        print("[MQTT Bridge] Starting MQTT loop...")
        try:
            self.client.loop_forever()
        except KeyboardInterrupt:
            print("\n[MQTT Bridge] Shutting down...")
            self.stop()

        return True

    def stop(self):
        """Stop the bridge"""
        self.running = False
        self.client.disconnect()
        if self.state_thread:
            self.state_thread.join(timeout=1)


def main():
    import argparse

    parser = argparse.ArgumentParser(
        description="MQTT Bridge for CyberBrick Simulation"
    )
    parser.add_argument("--host", default="localhost", help="MQTT broker host")
    parser.add_argument("--port", type=int, default=1883, help="MQTT broker port")
    args = parser.parse_args()

    bridge = MQTTBridge(mqtt_host=args.host, mqtt_port=args.port)
    bridge.start()


if __name__ == "__main__":
    main()
