#!/usr/bin/env python3
"""
Vision Node for CyberBrick ROS2
Detects colored cubes (red, blue, green) using OpenCV
Publishes detected objects with their positions
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from geometry_msgs.msg import Point
import cv2
from cv_bridge import CvBridge
import numpy as np
import json


class VisionNode(Node):
    def __init__(self):
        super().__init__('vision_node')
        
        # CV Bridge to convert ROS images to OpenCV
        self.bridge = CvBridge()
        
        # Subscriber for camera images
        self.image_sub = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )
        
        # Publisher for detected objects
        self.detection_pub = self.create_publisher(
            String,
            '/detected_objects',
            10
        )
        
        # Publisher for debug/visualization image
        self.debug_image_pub = self.create_publisher(
            Image,
            '/camera/debug_image',
            10
        )
        
        # Color ranges in HSV
        # These values may need tuning based on simulation lighting
        self.color_ranges = {
            'red': {
                'lower1': np.array([0, 100, 100]),
                'upper1': np.array([10, 255, 255]),
                'lower2': np.array([160, 100, 100]),  # Red wraps around in HSV
                'upper2': np.array([180, 255, 255]),
            },
            'green': {
                'lower': np.array([35, 100, 100]),
                'upper': np.array([85, 255, 255]),
            },
            'blue': {
                'lower': np.array([100, 100, 100]),
                'upper': np.array([130, 255, 255]),
            }
        }
        
        self.get_logger().info('Vision node initialized')

    def image_callback(self, msg):
        try:
            # Convert ROS Image to OpenCV format
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'Failed to convert image: {e}')
            return
        
        # Detect colored cubes
        detections = self.detect_cubes(cv_image)
        
        # Publish detections as JSON
        if detections:
            detection_msg = String()
            detection_msg.data = json.dumps(detections)
            self.detection_pub.publish(detection_msg)
            self.get_logger().debug(f'Detected: {detections}')
        
        # Create and publish debug image
        debug_image = self.draw_detections(cv_image, detections)
        try:
            debug_msg = self.bridge.cv2_to_imgmsg(debug_image, encoding='bgr8')
            self.debug_image_pub.publish(debug_msg)
        except Exception as e:
            self.get_logger().error(f'Failed to publish debug image: {e}')

    def detect_cubes(self, image):
        """Detect colored cubes in the image"""
        detections = []
        
        # Convert to HSV
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        
        for color_name, ranges in self.color_ranges.items():
            if color_name == 'red':
                # Red requires two ranges because it wraps around in HSV
                mask1 = cv2.inRange(hsv, ranges['lower1'], ranges['upper1'])
                mask2 = cv2.inRange(hsv, ranges['lower2'], ranges['upper2'])
                mask = cv2.bitwise_or(mask1, mask2)
            else:
                mask = cv2.inRange(hsv, ranges['lower'], ranges['upper'])
            
            # Clean up mask
            kernel = np.ones((5, 5), np.uint8)
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
            
            # Find contours
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            
            for contour in contours:
                area = cv2.contourArea(contour)
                
                # Filter by area (adjust these values based on cube size in image)
                if area > 100:  # Minimum area threshold
                    # Get bounding box
                    x, y, w, h = cv2.boundingRect(contour)
                    
                    # Calculate center
                    center_x = x + w // 2
                    center_y = y + h // 2
                    
                    # Estimate relative position (simple heuristic)
                    # Left/right based on x position in image
                    img_width = image.shape[1]
                    img_height = image.shape[0]
                    
                    rel_x = (center_x - img_width / 2) / (img_width / 2)  # -1 to 1
                    rel_y = (center_y - img_height / 2) / (img_height / 2)  # -1 to 1
                    
                    # Estimate distance based on apparent size (larger = closer)
                    # This is a rough estimate
                    rel_distance = 1000 / max(area, 1)  # Arbitrary scaling
                    
                    detections.append({
                        'color': color_name,
                        'center_x': center_x,
                        'center_y': center_y,
                        'width': w,
                        'height': h,
                        'area': area,
                        'rel_x': round(rel_x, 3),
                        'rel_y': round(rel_y, 3),
                        'rel_distance': round(rel_distance, 3)
                    })
        
        return detections

    def draw_detections(self, image, detections):
        """Draw bounding boxes and labels on image"""
        debug_image = image.copy()
        
        colors_bgr = {
            'red': (0, 0, 255),
            'green': (0, 255, 0),
            'blue': (255, 0, 0)
        }
        
        for det in detections:
            color = colors_bgr.get(det['color'], (255, 255, 255))
            x = det['center_x'] - det['width'] // 2
            y = det['center_y'] - det['height'] // 2
            
            # Draw bounding box
            cv2.rectangle(debug_image, (x, y), (x + det['width'], y + det['height']), color, 2)
            
            # Draw label
            label = f"{det['color']} ({det['area']})"
            cv2.putText(debug_image, label, (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
            
            # Draw center point
            cv2.circle(debug_image, (det['center_x'], det['center_y']), 5, color, -1)
        
        return debug_image


def main(args=None):
    rclpy.init(args=args)
    node = VisionNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
