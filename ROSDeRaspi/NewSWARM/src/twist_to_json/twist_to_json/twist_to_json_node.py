#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import json

class TwistToJson(Node):
    def __init__(self):
        super().__init__('twist_to_json')
        self.sub = self.create_subscription(Twist, '/cmd_vel', self.cb, 10)
        self.pub = self.create_publisher(String, '/movement', 10)
        self.get_logger().info("Twist → JSON bridge ready (listening on /cmd_vel)")

    def cb(self, msg: Twist):
        direction = "STOP"
        speed = 0
        angle = 0

        # --- Forward / Backward ---
        if msg.linear.x > 0:
            direction = "FORWARD"
            speed = int(abs(msg.linear.x) * 255)  # map to 0–255
        elif msg.linear.x < 0:
            direction = "BACKWARD"
            speed = int(abs(msg.linear.x) * 255)

        # --- Turning ---
        elif msg.angular.z > 0:
            direction = "LEFT"
            # IMPORTANT: fallback fixed duty when only turning
            speed = 200
            angle = int(abs(msg.angular.z) * 90)
        elif msg.angular.z < 0:
            direction = "RIGHT"
            # IMPORTANT: fallback fixed duty when only turning
            speed = 200
            angle = int(abs(msg.angular.z) * 90)

        else:
            direction = "STOP"
            speed = 0
            angle = 0

        json_msg = {
            "direction": direction,
            "speed": speed,
            "angle": angle
        }

        self.pub.publish(String(data=json.dumps(json_msg)))
        self.get_logger().info(f"Published {json_msg}")

def main(args=None):
    rclpy.init(args=args)
    node = TwistToJson()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

