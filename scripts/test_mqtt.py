#!/usr/bin/env python3
"""
Test script for MQTT Bridge

Run this script to test controlling the simulation via MQTT.
Make sure the MQTT bridge is running first!

Usage:
    python3 test_mqtt.py --host localhost --port 1883
"""

import json
import time
import paho.mqtt.client as mqtt


class CyberBrickController:
    """Simple controller for CyberBrick via MQTT"""

    def __init__(self, host="localhost", port=1883):
        self.client = mqtt.Client(client_id="cyberbrick_test")
        self.client.on_connect = self._on_connect
        self.client.on_message = self._on_message
        self.client.connect(host, port, 60)
        self.client.loop_start()
        time.sleep(0.5)  # Wait for connection

    def _on_connect(self, client, userdata, flags, rc):
        if rc == 0:
            print("[Test] Connected to MQTT broker")
            client.subscribe("cyberbrick/+/state")
            client.subscribe("cyberbrick/+/odom")
        else:
            print(f"[Test] Connection failed: {rc}")

    def _on_message(self, client, userdata, msg):
        print(f"[Test] State update: {msg.topic} -> {msg.payload.decode()}")

    def _publish(self, topic, payload):
        self.client.publish(topic, json.dumps(payload))
        print(f"[Test] Sent: {topic} -> {payload}")

    # --- Truck commands ---

    def truck_move(self, speed, steering=0):
        """Move truck: speed in m/s, steering in radians"""
        self._publish("cyberbrick/truck/cmd", {"speed": speed, "steering": steering})

    def truck_stop(self):
        """Stop the truck"""
        self._publish("cyberbrick/truck/stop", {})

    # --- Arm commands ---

    def arm_base(self, angle):
        """Rotate arm base: angle in degrees (-90 to 90)"""
        self._publish("cyberbrick/arm/base", {"angle": angle})

    def arm_shoulder(self, angle):
        """Move arm shoulder: angle in degrees"""
        self._publish("cyberbrick/arm/shoulder", {"angle": angle})

    def arm_elbow(self, angle):
        """Move arm elbow: angle in degrees"""
        self._publish("cyberbrick/arm/elbow", {"angle": angle})

    def arm_gripper(self, open=True):
        """Open or close gripper"""
        self._publish("cyberbrick/arm/gripper", {"open": open})

    def arm_move(self, base=None, shoulder=None, elbow=None, gripper_open=None):
        """Move multiple arm joints at once"""
        payload = {}
        if base is not None:
            payload["base"] = base
        if shoulder is not None:
            payload["shoulder"] = shoulder
        if elbow is not None:
            payload["elbow"] = elbow
        if gripper_open is not None:
            payload["gripper_open"] = gripper_open
        self._publish("cyberbrick/arm/move", payload)

    def close(self):
        self.client.loop_stop()
        self.client.disconnect()


def demo_sequence(controller):
    """Run a demo sequence"""
    print("\n=== Demo: Arm movements ===")

    print("\n1. Moving arm to home position...")
    controller.arm_move(base=0, shoulder=0, elbow=0)
    time.sleep(2)

    print("\n2. Rotating base...")
    controller.arm_base(45)
    time.sleep(2)

    print("\n3. Moving shoulder up...")
    controller.arm_shoulder(45)
    time.sleep(2)

    print("\n4. Moving elbow...")
    controller.arm_elbow(30)
    time.sleep(2)

    print("\n5. Closing gripper...")
    controller.arm_gripper(open=False)
    time.sleep(1)

    print("\n6. Opening gripper...")
    controller.arm_gripper(open=True)
    time.sleep(1)

    print("\n=== Demo: Truck movements ===")

    print("\n7. Moving truck forward...")
    controller.truck_move(speed=0.3)
    time.sleep(2)

    print("\n8. Steering left...")
    controller.truck_move(speed=0.2, steering=0.3)
    time.sleep(2)

    print("\n9. Stopping truck...")
    controller.truck_stop()
    time.sleep(1)

    print("\n=== Demo complete! ===")


def interactive_mode(controller):
    """Interactive command mode"""
    print("\n=== Interactive Mode ===")
    print("Commands:")
    print("  truck <speed> [steering]  - Move truck (m/s, rad)")
    print("  stop                      - Stop truck")
    print("  base <angle>              - Rotate arm base (degrees)")
    print("  shoulder <angle>          - Move shoulder (degrees)")
    print("  elbow <angle>             - Move elbow (degrees)")
    print("  grip open|close           - Control gripper")
    print("  demo                      - Run demo sequence")
    print("  quit                      - Exit")
    print()

    while True:
        try:
            cmd = input("> ").strip().lower().split()
            if not cmd:
                continue

            if cmd[0] == "quit" or cmd[0] == "q":
                break
            elif cmd[0] == "demo":
                demo_sequence(controller)
            elif cmd[0] == "truck" and len(cmd) >= 2:
                speed = float(cmd[1])
                steering = float(cmd[2]) if len(cmd) > 2 else 0
                controller.truck_move(speed, steering)
            elif cmd[0] == "stop":
                controller.truck_stop()
            elif cmd[0] == "base" and len(cmd) >= 2:
                controller.arm_base(float(cmd[1]))
            elif cmd[0] == "shoulder" and len(cmd) >= 2:
                controller.arm_shoulder(float(cmd[1]))
            elif cmd[0] == "elbow" and len(cmd) >= 2:
                controller.arm_elbow(float(cmd[1]))
            elif cmd[0] == "grip" and len(cmd) >= 2:
                controller.arm_gripper(open=(cmd[1] == "open"))
            else:
                print("Unknown command. Type 'quit' to exit.")

        except ValueError as e:
            print(f"Invalid value: {e}")
        except KeyboardInterrupt:
            break

    print("\nExiting...")


def main():
    import argparse

    parser = argparse.ArgumentParser(description="Test CyberBrick MQTT control")
    parser.add_argument("--host", default="localhost", help="MQTT broker host")
    parser.add_argument("--port", type=int, default=1883, help="MQTT broker port")
    parser.add_argument(
        "--demo", action="store_true", help="Run demo sequence and exit"
    )
    args = parser.parse_args()

    controller = CyberBrickController(host=args.host, port=args.port)

    try:
        if args.demo:
            demo_sequence(controller)
        else:
            interactive_mode(controller)
    finally:
        controller.close()


if __name__ == "__main__":
    main()
