#!/usr/bin/env python3
"""
Arm Controller for CyberBrick ROS2
Provides inverse kinematics and pick/place sequences for the L-ONE arm.
Receives commands via MQTT and executes them by publishing to the MQTT bridge.
"""

import math
import time
import json
import signal
import sys
import paho.mqtt.client as mqtt

# Configuration
import os

MQTT_BROKER = os.environ.get(
    "MQTT_HOST", "mosquitto"
)  # Use 'mosquitto' as default for Docker
MQTT_PORT = int(os.environ.get("MQTT_PORT", 1883))

# MQTT Topics
ARM_COMMAND_TOPIC = "cyberbrick/arm/command"
ARM_STATUS_TOPIC = "cyberbrick/arm/status"
ARM_BASE_TOPIC = "cyberbrick/arm/base"
ARM_SHOULDER_TOPIC = "cyberbrick/arm/shoulder"
ARM_ELBOW_TOPIC = "cyberbrick/arm/elbow"
ARM_GRIPPER_TOPIC = "cyberbrick/arm/gripper"

# L-ONE Arm Geometry (in meters)
# Arm is mounted at (0.5, 0, 0.26) on the table
ARM_BASE_X = 0.5
ARM_BASE_Y = 0.0
ARM_BASE_Z = 0.26

# Link lengths (from SDF: lower_arm visual 0.08m, upper_arm visual 0.07m + gripper offset 0.07m)
BASE_HEIGHT = 0.045  # Height of base + turntable
LOWER_ARM_LENGTH = 0.10  # Shoulder to elbow (including joint offsets)
UPPER_ARM_LENGTH = 0.12  # Elbow to gripper tip (upper_arm + gripper)

# Joint limits (in degrees) - relaxed for testing
BASE_MIN, BASE_MAX = -90, 90
SHOULDER_MIN, SHOULDER_MAX = -30, 100  # Relaxed from 90
ELBOW_MIN, ELBOW_MAX = -90, 135  # Relaxed from 90

# Heights for pick/place operations
APPROACH_HEIGHT = 0.06  # Height above table for approach (reduced for reach)
PICK_HEIGHT = 0.02  # Height for grabbing cube (cube is 4cm)
SAFE_HEIGHT = 0.10  # Safe height for moving

# Timing
MOVE_DELAY = 0.8  # Delay between joint movements
GRIPPER_DELAY = 0.5  # Delay for gripper open/close
SETTLE_DELAY = 0.3  # Delay for arm to settle


class ArmController:
    def __init__(self):
        self.running = True
        self.mqtt_client = None
        self.busy = False
        self.current_action = None

        # Current joint positions (in degrees)
        self.current_base = 0
        self.current_shoulder = 0
        self.current_elbow = 0
        self.gripper_open = True

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
            print(f"[Arm] Connecting to MQTT broker at {MQTT_BROKER}:{MQTT_PORT}")
        except Exception as e:
            print(f"[Arm] Failed to connect to MQTT broker: {e}")

    def on_mqtt_connect(self, client, userdata, flags, reason_code, properties):
        print(f"[Arm] Connected to MQTT broker")
        # Subscribe to command topic
        self.mqtt_client.subscribe(ARM_COMMAND_TOPIC)
        print(f"[Arm] Subscribed to {ARM_COMMAND_TOPIC}")
        # Publish initial status
        self.publish_status("ready")

    def on_mqtt_disconnect(self, client, userdata, flags, reason_code, properties):
        print(f"[Arm] Disconnected from MQTT broker")

    def on_mqtt_message(self, client, userdata, msg):
        """Handle incoming MQTT commands"""
        try:
            payload = json.loads(msg.payload.decode())
            action = payload.get("action", "")

            print(f"[Arm] Received command: {payload}")

            if self.busy:
                print("[Arm] Busy, ignoring command")
                self.publish_status("busy", error="Arm is busy")
                return

            if action == "pick":
                x = payload.get("x")
                y = payload.get("y")
                if x is not None and y is not None:
                    self.execute_pick(x, y)
                else:
                    self.publish_status("error", error="Missing x or y for pick")

            elif action == "place":
                x = payload.get("x")
                y = payload.get("y")
                if x is not None and y is not None:
                    self.execute_place(x, y)
                else:
                    self.publish_status("error", error="Missing x or y for place")

            elif action == "home":
                self.execute_home()

            elif action == "move":
                # Direct joint control
                base = payload.get("base")
                shoulder = payload.get("shoulder")
                elbow = payload.get("elbow")
                self.move_joints(base, shoulder, elbow)

            elif action == "gripper":
                open_gripper = payload.get("open", True)
                self.control_gripper(open_gripper)

            else:
                print(f"[Arm] Unknown action: {action}")
                self.publish_status("error", error=f"Unknown action: {action}")

        except json.JSONDecodeError as e:
            print(f"[Arm] Invalid JSON: {e}")
        except Exception as e:
            print(f"[Arm] Error handling message: {e}")
            self.publish_status("error", error=str(e))

    def publish_status(self, status, error=None, action=None):
        """Publish arm status"""
        msg = {
            "status": status,
            "busy": self.busy,
            "action": action or self.current_action,
            "position": {
                "base": self.current_base,
                "shoulder": self.current_shoulder,
                "elbow": self.current_elbow,
                "gripper_open": self.gripper_open,
            },
            "timestamp": time.time(),
        }
        if error:
            msg["error"] = error

        try:
            self.mqtt_client.publish(ARM_STATUS_TOPIC, json.dumps(msg))
        except Exception as e:
            print(f"[Arm] Error publishing status: {e}")

    def inverse_kinematics(self, x, y, z):
        """
        Calculate joint angles for target position (x, y, z) in world coordinates.
        Returns (base_angle, shoulder_angle, elbow_angle) in degrees, or None if unreachable.
        """
        # Convert to arm-relative coordinates
        rel_x = x - ARM_BASE_X
        rel_y = y - ARM_BASE_Y
        rel_z = z - ARM_BASE_Z - BASE_HEIGHT

        # Calculate base rotation (around Z axis)
        base_angle = math.degrees(math.atan2(rel_y, rel_x))

        # Check base limits
        if base_angle < BASE_MIN or base_angle > BASE_MAX:
            print(f"[Arm] Base angle {base_angle:.1f} out of range")
            return None

        # Distance in XY plane from base to target
        r = math.sqrt(rel_x**2 + rel_y**2)

        # Distance from shoulder to target (in the arm plane)
        d = math.sqrt(r**2 + rel_z**2)

        # Check if reachable
        max_reach = LOWER_ARM_LENGTH + UPPER_ARM_LENGTH
        min_reach = abs(LOWER_ARM_LENGTH - UPPER_ARM_LENGTH)

        if d > max_reach:
            print(f"[Arm] Target too far: {d:.3f}m > {max_reach:.3f}m")
            return None
        if d < min_reach:
            print(f"[Arm] Target too close: {d:.3f}m < {min_reach:.3f}m")
            return None

        # Use law of cosines to find elbow angle
        # cos(elbow) = (L1^2 + L2^2 - d^2) / (2 * L1 * L2)
        cos_elbow = (LOWER_ARM_LENGTH**2 + UPPER_ARM_LENGTH**2 - d**2) / (
            2 * LOWER_ARM_LENGTH * UPPER_ARM_LENGTH
        )
        cos_elbow = max(-1, min(1, cos_elbow))  # Clamp for numerical stability

        elbow_angle = math.degrees(math.acos(cos_elbow))
        # We want elbow angle relative to lower arm, so subtract 180
        elbow_angle = 180 - elbow_angle

        # Find shoulder angle
        # Angle from horizontal to target
        alpha = math.atan2(rel_z, r)

        # Angle from lower arm to line connecting shoulder to target
        cos_beta = (LOWER_ARM_LENGTH**2 + d**2 - UPPER_ARM_LENGTH**2) / (
            2 * LOWER_ARM_LENGTH * d
        )
        cos_beta = max(-1, min(1, cos_beta))
        beta = math.acos(cos_beta)

        shoulder_angle = math.degrees(alpha + beta)

        # Check limits
        if shoulder_angle < SHOULDER_MIN or shoulder_angle > SHOULDER_MAX:
            print(f"[Arm] Shoulder angle {shoulder_angle:.1f} out of range")
            return None
        if elbow_angle < ELBOW_MIN or elbow_angle > ELBOW_MAX:
            print(f"[Arm] Elbow angle {elbow_angle:.1f} out of range")
            return None

        return (base_angle, shoulder_angle, elbow_angle)

    def move_to_joint(self, topic, angle):
        """Send joint position command"""
        try:
            self.mqtt_client.publish(topic, json.dumps({"angle": angle}))
        except Exception as e:
            print(f"[Arm] Error sending joint command: {e}")

    def control_gripper(self, open_gripper):
        """Open or close the gripper"""
        try:
            self.mqtt_client.publish(
                ARM_GRIPPER_TOPIC, json.dumps({"open": open_gripper})
            )
            self.gripper_open = open_gripper
            time.sleep(GRIPPER_DELAY)
        except Exception as e:
            print(f"[Arm] Error controlling gripper: {e}")

    def move_joints(self, base=None, shoulder=None, elbow=None, wait=True):
        """Move joints to specified angles (in degrees)"""
        if base is not None:
            self.move_to_joint(ARM_BASE_TOPIC, base)
            self.current_base = base
        if shoulder is not None:
            self.move_to_joint(ARM_SHOULDER_TOPIC, shoulder)
            self.current_shoulder = shoulder
        if elbow is not None:
            self.move_to_joint(ARM_ELBOW_TOPIC, elbow)
            self.current_elbow = elbow

        if wait:
            time.sleep(MOVE_DELAY)

    def move_to_position(self, x, y, z):
        """Move arm to world position using inverse kinematics"""
        angles = self.inverse_kinematics(x, y, z)
        if angles is None:
            print(f"[Arm] Cannot reach position ({x:.3f}, {y:.3f}, {z:.3f})")
            return False

        base, shoulder, elbow = angles
        print(
            f"[Arm] Moving to ({x:.3f}, {y:.3f}, {z:.3f}) -> base={base:.1f}, shoulder={shoulder:.1f}, elbow={elbow:.1f}"
        )

        self.move_joints(base, shoulder, elbow)
        return True

    def execute_home(self):
        """Move arm to home position"""
        self.busy = True
        self.current_action = "home"
        self.publish_status("executing", action="home")

        print("[Arm] Moving to home position")

        # Open gripper first
        self.control_gripper(True)

        # Move to safe position
        self.move_joints(base=0, shoulder=45, elbow=0)
        time.sleep(SETTLE_DELAY)

        # Move to home
        self.move_joints(base=0, shoulder=0, elbow=0)
        time.sleep(SETTLE_DELAY)

        self.busy = False
        self.current_action = None
        self.publish_status("ready", action="home")
        print("[Arm] Home position reached")

    def execute_pick(self, x, y):
        """Execute pick sequence at position (x, y)"""
        self.busy = True
        self.current_action = "pick"
        self.publish_status("executing", action="pick")

        print(f"[Arm] Picking at ({x:.3f}, {y:.3f})")

        # Calculate table height for the cube
        table_z = 0.26 + 0.02  # Table surface

        # Step 1: Open gripper
        print("[Arm] Step 1: Opening gripper")
        self.control_gripper(True)

        # Step 2: Move to approach position (above the cube)
        print("[Arm] Step 2: Moving to approach position")
        approach_z = table_z + APPROACH_HEIGHT
        if not self.move_to_position(x, y, approach_z):
            self.busy = False
            self.publish_status("error", error="Cannot reach approach position")
            return False
        time.sleep(SETTLE_DELAY)

        # Step 3: Move down to pick height
        print("[Arm] Step 3: Moving down to pick height")
        pick_z = table_z + PICK_HEIGHT
        if not self.move_to_position(x, y, pick_z):
            self.busy = False
            self.publish_status("error", error="Cannot reach pick position")
            return False
        time.sleep(SETTLE_DELAY)

        # Step 4: Close gripper
        print("[Arm] Step 4: Closing gripper")
        self.control_gripper(False)

        # Step 5: Lift up
        print("[Arm] Step 5: Lifting up")
        safe_z = table_z + SAFE_HEIGHT
        if not self.move_to_position(x, y, safe_z):
            # Try to at least move up
            self.move_joints(shoulder=60, elbow=-30)
        time.sleep(SETTLE_DELAY)

        self.busy = False
        self.current_action = None
        self.publish_status("ready", action="pick")
        print(f"[Arm] Pick completed at ({x:.3f}, {y:.3f})")
        return True

    def execute_place(self, x, y):
        """Execute place sequence at position (x, y)"""
        self.busy = True
        self.current_action = "place"
        self.publish_status("executing", action="place")

        print(f"[Arm] Placing at ({x:.3f}, {y:.3f})")

        # Calculate table height
        table_z = 0.26 + 0.02

        # Step 1: Move to approach position
        print("[Arm] Step 1: Moving to approach position")
        approach_z = table_z + APPROACH_HEIGHT
        if not self.move_to_position(x, y, approach_z):
            self.busy = False
            self.publish_status("error", error="Cannot reach approach position")
            return False
        time.sleep(SETTLE_DELAY)

        # Step 2: Move down
        print("[Arm] Step 2: Moving down")
        place_z = table_z + PICK_HEIGHT
        if not self.move_to_position(x, y, place_z):
            self.busy = False
            self.publish_status("error", error="Cannot reach place position")
            return False
        time.sleep(SETTLE_DELAY)

        # Step 3: Open gripper
        print("[Arm] Step 3: Opening gripper")
        self.control_gripper(True)

        # Step 4: Lift up
        print("[Arm] Step 4: Lifting up")
        safe_z = table_z + SAFE_HEIGHT
        if not self.move_to_position(x, y, safe_z):
            self.move_joints(shoulder=60, elbow=-30)
        time.sleep(SETTLE_DELAY)

        self.busy = False
        self.current_action = None
        self.publish_status("ready", action="place")
        print(f"[Arm] Place completed at ({x:.3f}, {y:.3f})")
        return True

    def run(self):
        """Main loop"""
        print("[Arm] Starting arm controller...")
        print(f"[Arm] Listening for commands on {ARM_COMMAND_TOPIC}")

        # Move to home on startup
        time.sleep(1)
        self.execute_home()

        while self.running:
            try:
                time.sleep(0.1)
            except KeyboardInterrupt:
                break

        self.cleanup()

    def cleanup(self):
        """Cleanup resources"""
        print("[Arm] Shutting down...")
        self.running = False
        if self.mqtt_client:
            self.mqtt_client.loop_stop()
            self.mqtt_client.disconnect()


def main():
    controller = ArmController()

    def signal_handler(sig, frame):
        print("\n[Arm] Received shutdown signal")
        controller.running = False

    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)

    controller.run()


if __name__ == "__main__":
    main()
