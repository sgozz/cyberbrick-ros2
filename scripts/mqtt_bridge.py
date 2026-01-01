#!/usr/bin/env python3
"""
MQTT Bridge for CyberBrick
Converts ROS2 topics to MQTT and vice versa.

This allows the same code to work with:
- Simulated robot in Gazebo
- Real CyberBrick robot via MQTT
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import paho.mqtt.client as mqtt
import json
import threading


class MQTTBridge(Node):
    def __init__(self):
        super().__init__('mqtt_bridge')
        
        # Parameters
        self.declare_parameter('mqtt_broker', 'localhost')
        self.declare_parameter('mqtt_port', 1883)
        self.declare_parameter('robot_client_id', 'cyberbrick-robot')
        self.declare_parameter('arm_client_id', 'cyberbrick-arm')
        
        broker = self.get_parameter('mqtt_broker').value
        port = self.get_parameter('mqtt_port').value
        self.robot_id = self.get_parameter('robot_client_id').value
        self.arm_id = self.get_parameter('arm_client_id').value
        
        # MQTT topics
        self.robot_cmd_topic = f"cyberbrick/{self.robot_id}/cmd"
        self.arm_cmd_topic = f"cyberbrick/{self.arm_id}/cmd"
        
        # ROS2 subscribers
        self.robot_vel_sub = self.create_subscription(
            Twist,
            '/cyberbrick/cmd_vel',
            self.robot_vel_callback,
            10
        )
        
        self.arm_cmd_sub = self.create_subscription(
            String,
            '/arm/command',
            self.arm_cmd_callback,
            10
        )
        
        # Setup MQTT
        self.mqtt_client = mqtt.Client(
            mqtt.CallbackAPIVersion.VERSION1,
            client_id='ros2-mqtt-bridge'
        )
        self.mqtt_client.on_connect = self.on_mqtt_connect
        self.mqtt_client.on_disconnect = self.on_mqtt_disconnect
        
        try:
            self.mqtt_client.connect(broker, port)
            self.mqtt_client.loop_start()
            self.get_logger().info(f'MQTT connected to {broker}:{port}')
        except Exception as e:
            self.get_logger().error(f'MQTT connection failed: {e}')
        
        self.get_logger().info('MQTT Bridge started')
        self.get_logger().info(f'  Robot topic: {self.robot_cmd_topic}')
        self.get_logger().info(f'  Arm topic: {self.arm_cmd_topic}')
    
    def on_mqtt_connect(self, client, userdata, flags, rc):
        if rc == 0:
            self.get_logger().info('MQTT connected successfully')
        else:
            self.get_logger().error(f'MQTT connection failed: {rc}')
    
    def on_mqtt_disconnect(self, client, userdata, rc):
        self.get_logger().warn('MQTT disconnected')
    
    def robot_vel_callback(self, msg: Twist):
        """Convert Twist to CyberBrick RC data."""
        # Map linear.x and angular.z to differential drive
        linear = msg.linear.x  # -1 to 1
        angular = msg.angular.z  # -1 to 1
        
        # Convert to RC values (0-4096, center=2048)
        # L2 = throttle, L3 = steering
        l2 = int(2048 + linear * 1500)  # Forward/back
        l3 = int(2048 + angular * 1500)  # Left/right
        
        # Clamp values
        l2 = max(0, min(4096, l2))
        l3 = max(0, min(4096, l3))
        
        # Create RC data array
        rc_data = {
            "rc": [2048, l2, l3, 2048, 2048, 2048, 1, 1, 1, 1]
        }
        
        # Send via MQTT
        self.mqtt_client.publish(
            self.robot_cmd_topic,
            json.dumps(rc_data)
        )
        
        self.get_logger().debug(f'Robot cmd: linear={linear:.2f}, angular={angular:.2f}')
    
    def arm_cmd_callback(self, msg: String):
        """Send arm command via MQTT."""
        try:
            # Expect JSON string with servo positions
            # Example: {"servo1": 90, "servo2": 45, "servo3": 0}
            cmd = json.loads(msg.data)
            
            # Convert to RC format
            # PWM1 = servo1 (base rotation)
            # PWM2 = servo2 (elbow)
            # PWM3 = servo3 (gripper)
            
            s1 = cmd.get('servo1', 90)
            s2 = cmd.get('servo2', 90)
            s3 = cmd.get('servo3', 90)
            
            # Servo angles 0-180 → RC values 0-4096
            r1 = int(s1 * 4096 / 180)
            r2 = int(s2 * 4096 / 180)
            r3 = int(s3 * 4096 / 180)
            
            rc_data = {
                "rc": [r1, r2, r3, 2048, 2048, 2048, 1, 1, 1, 1]
            }
            
            self.mqtt_client.publish(
                self.arm_cmd_topic,
                json.dumps(rc_data)
            )
            
            self.get_logger().debug(f'Arm cmd: s1={s1}, s2={s2}, s3={s3}')
            
        except json.JSONDecodeError as e:
            self.get_logger().error(f'Invalid arm command JSON: {e}')
    
    def destroy_node(self):
        self.mqtt_client.loop_stop()
        self.mqtt_client.disconnect()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = MQTTBridge()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
