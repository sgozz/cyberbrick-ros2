#!/usr/bin/env python3
"""
Vision Bridge for CyberBrick ROS2
Captures images from Gazebo overhead camera, detects colored cubes using OpenCV,
and publishes their positions via MQTT.
"""

import subprocess
import re
import time
import json
import threading
import signal
import sys
import numpy as np
import cv2
import paho.mqtt.client as mqtt

# Configuration
import os

MQTT_BROKER = os.environ.get(
    "MQTT_HOST", "mosquitto"
)  # Use 'mosquitto' as default for Docker
MQTT_PORT = int(os.environ.get("MQTT_PORT", 1883))
VISION_TOPIC = "cyberbrick/vision/state"
CAMERA_TOPIC = "/overhead_camera/image_raw"

# Camera calibration (pixel to world coordinates)
# Camera is at (0.5, 0, 0.8) looking down at table at z=0.26
# Table is 0.6m x 0.4m centered at (0.5, 0)
# With FOV ~1.2 rad (~69°), at 0.54m height above table:
# visible area approx 0.7m x 0.52m
IMAGE_WIDTH = 640
IMAGE_HEIGHT = 480

# Calibration: map pixel coordinates to world coordinates
# These values need tuning based on actual camera view
WORLD_X_MIN = 0.15  # World X at image top
WORLD_X_MAX = 0.85  # World X at image bottom
WORLD_Y_MIN = -0.35  # World Y at image right
WORLD_Y_MAX = 0.35  # World Y at image left

# HSV color ranges for cube detection
COLOR_RANGES = {
    "red": {
        "lower1": np.array([0, 120, 100]),
        "upper1": np.array([10, 255, 255]),
        "lower2": np.array([160, 120, 100]),
        "upper2": np.array([180, 255, 255]),
    },
    "green": {
        "lower": np.array([35, 100, 100]),
        "upper": np.array([85, 255, 255]),
    },
    "blue": {
        "lower": np.array([100, 120, 100]),
        "upper": np.array([140, 255, 255]),
    },
}

# Minimum contour area to consider as a cube
MIN_CUBE_AREA = 200


class VisionBridge:
    def __init__(self):
        self.running = True
        self.mqtt_client = None
        self.last_state = None

        # Setup MQTT
        self.setup_mqtt()

    def setup_mqtt(self):
        """Setup MQTT client"""
        self.mqtt_client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
        self.mqtt_client.on_connect = self.on_mqtt_connect
        self.mqtt_client.on_disconnect = self.on_mqtt_disconnect

        try:
            self.mqtt_client.connect(MQTT_BROKER, MQTT_PORT, 60)
            self.mqtt_client.loop_start()
            print(f"[Vision] Connecting to MQTT broker at {MQTT_BROKER}:{MQTT_PORT}")
        except Exception as e:
            print(f"[Vision] Failed to connect to MQTT broker: {e}")

    def on_mqtt_connect(self, client, userdata, flags, reason_code, properties):
        print(f"[Vision] Connected to MQTT broker")

    def on_mqtt_disconnect(self, client, userdata, flags, reason_code, properties):
        print(f"[Vision] Disconnected from MQTT broker")

    def capture_image(self):
        """Capture one image from Gazebo camera topic"""
        try:
            # Use ign topic to get one message
            result = subprocess.run(
                ["ign", "topic", "-e", "-t", CAMERA_TOPIC, "-n", "1"],
                capture_output=True,
                timeout=5,
            )

            if result.returncode != 0:
                print(f"[Vision] Failed to capture image: {result.stderr.decode()}")
                return None

            return result.stdout

        except subprocess.TimeoutExpired:
            print("[Vision] Timeout capturing image")
            return None
        except Exception as e:
            print(f"[Vision] Error capturing image: {e}")
            return None

    def parse_image(self, raw_data):
        """Parse protobuf text format image data to numpy array"""
        try:
            text = raw_data.decode("latin-1")

            # Extract dimensions
            width_match = re.search(r"width:\s*(\d+)", text)
            height_match = re.search(r"height:\s*(\d+)", text)

            if not width_match or not height_match:
                print("[Vision] Could not find image dimensions")
                return None

            width = int(width_match.group(1))
            height = int(height_match.group(1))

            # Find the data field - it starts after 'data: "' and ends at the last '"'
            data_start = text.find('data: "')
            if data_start == -1:
                print("[Vision] Could not find image data")
                return None

            data_start += 7  # Skip 'data: "'

            # Find the end of data (last quote before any trailing content)
            data_end = text.rfind('"')
            if data_end <= data_start:
                print("[Vision] Could not find end of image data")
                return None

            # Extract the escaped string data
            escaped_data = text[data_start:data_end]

            # Decode escape sequences
            # Handle octal escapes like \332
            def decode_escape(s):
                result = bytearray()
                i = 0
                while i < len(s):
                    if s[i] == "\\" and i + 1 < len(s):
                        next_char = s[i + 1]
                        if next_char.isdigit():
                            # Octal escape - read up to 3 digits
                            octal = ""
                            j = i + 1
                            while j < len(s) and j < i + 4 and s[j].isdigit():
                                octal += s[j]
                                j += 1
                            result.append(int(octal, 8))
                            i = j
                        elif next_char == "n":
                            result.append(ord("\n"))
                            i += 2
                        elif next_char == "r":
                            result.append(ord("\r"))
                            i += 2
                        elif next_char == "t":
                            result.append(ord("\t"))
                            i += 2
                        elif next_char == "\\":
                            result.append(ord("\\"))
                            i += 2
                        elif next_char == '"':
                            result.append(ord('"'))
                            i += 2
                        else:
                            result.append(ord(s[i]))
                            i += 1
                    else:
                        result.append(ord(s[i]))
                        i += 1
                return bytes(result)

            pixel_data = decode_escape(escaped_data)

            expected_size = width * height * 3
            if len(pixel_data) < expected_size:
                print(
                    f"[Vision] Image data too small: {len(pixel_data)} < {expected_size}"
                )
                return None

            # Convert to numpy array
            img_array = np.frombuffer(pixel_data[:expected_size], dtype=np.uint8)
            img_array = img_array.reshape((height, width, 3))

            # Convert RGB to BGR for OpenCV
            img_bgr = cv2.cvtColor(img_array, cv2.COLOR_RGB2BGR)

            return img_bgr

        except Exception as e:
            print(f"[Vision] Error parsing image: {e}")
            return None

    def detect_cubes(self, image):
        """Detect colored cubes in the image using HSV color filtering"""
        detections = []

        # Convert to HSV
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        for color_name, ranges in COLOR_RANGES.items():
            if color_name == "red":
                # Red requires two ranges (wraps around in HSV)
                mask1 = cv2.inRange(hsv, ranges["lower1"], ranges["upper1"])
                mask2 = cv2.inRange(hsv, ranges["lower2"], ranges["upper2"])
                mask = cv2.bitwise_or(mask1, mask2)
            else:
                mask = cv2.inRange(hsv, ranges["lower"], ranges["upper"])

            # Morphological operations to clean up
            kernel = np.ones((5, 5), np.uint8)
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

            # Find contours
            contours, _ = cv2.findContours(
                mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
            )

            for contour in contours:
                area = cv2.contourArea(contour)

                if area > MIN_CUBE_AREA:
                    # Get bounding box
                    x, y, w, h = cv2.boundingRect(contour)

                    # Calculate center in pixels
                    center_px = x + w // 2
                    center_py = y + h // 2

                    # Convert to world coordinates
                    world_x, world_y = self.pixel_to_world(center_px, center_py)

                    detections.append(
                        {
                            "color": color_name,
                            "x": round(world_x, 3),
                            "y": round(world_y, 3),
                            "pixel_x": center_px,
                            "pixel_y": center_py,
                            "area": area,
                            "visible": True,
                        }
                    )

        return detections

    def pixel_to_world(self, px, py):
        """Convert pixel coordinates to world coordinates"""
        # Linear interpolation
        # Note: camera is looking down, so image Y maps to world X (forward/back)
        # and image X maps to world Y (left/right, inverted)

        world_x = WORLD_X_MIN + (py / IMAGE_HEIGHT) * (WORLD_X_MAX - WORLD_X_MIN)
        world_y = WORLD_Y_MAX - (px / IMAGE_WIDTH) * (WORLD_Y_MAX - WORLD_Y_MIN)

        return world_x, world_y

    def publish_state(self, detections):
        """Publish vision state to MQTT"""
        state = {"cubes": detections, "timestamp": time.time()}

        # Add cubes that were expected but not detected
        detected_colors = {d["color"] for d in detections}
        for color in ["red", "green", "blue"]:
            if color not in detected_colors:
                state["cubes"].append(
                    {"color": color, "x": None, "y": None, "visible": False}
                )

        # Sort by color for consistency
        state["cubes"].sort(key=lambda c: c["color"])

        try:
            self.mqtt_client.publish(VISION_TOPIC, json.dumps(state))
            self.last_state = state
        except Exception as e:
            print(f"[Vision] Error publishing state: {e}")

    def run(self):
        """Main loop"""
        print("[Vision] Starting vision bridge...")
        print(f"[Vision] Listening to camera topic: {CAMERA_TOPIC}")
        print(f"[Vision] Publishing to MQTT topic: {VISION_TOPIC}")

        # Wait for Gazebo to start
        time.sleep(1)

        while self.running:
            try:
                # Capture image
                raw_data = self.capture_image()
                if raw_data is None:
                    time.sleep(1)
                    continue

                # Parse image
                image = self.parse_image(raw_data)
                if image is None:
                    time.sleep(1)
                    continue

                # Detect cubes
                detections = self.detect_cubes(image)

                # Publish state
                self.publish_state(detections)

                # Print status
                visible_cubes = [d for d in detections if d.get("visible", False)]
                if visible_cubes:
                    cubes_str = ", ".join(
                        [
                            f"{d['color']}({d['x']:.2f},{d['y']:.2f})"
                            for d in visible_cubes
                        ]
                    )
                    print(f"[Vision] Detected: {cubes_str}")

                # Rate limit
                time.sleep(0.5)

            except KeyboardInterrupt:
                break
            except Exception as e:
                print(f"[Vision] Error in main loop: {e}")
                time.sleep(1)

        self.cleanup()

    def cleanup(self):
        """Cleanup resources"""
        print("[Vision] Shutting down...")
        self.running = False
        if self.mqtt_client:
            self.mqtt_client.loop_stop()
            self.mqtt_client.disconnect()


def main():
    bridge = VisionBridge()

    # Setup signal handler
    def signal_handler(sig, frame):
        print("\n[Vision] Received shutdown signal")
        bridge.running = False

    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)

    bridge.run()


if __name__ == "__main__":
    main()
