#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import serial, threading
from .utils.consts import Consts, Movement

class SerialBridge(Node):
    def __init__(self):
        super().__init__('serial_bridge')

        # Open serial port
        try:
            self.port = serial.Serial(
                Consts.serial_port,
                Consts.serial_baudrate,
                timeout=Consts.serial_timeout
            )
            self.get_logger().info(f"Opened serial port {Consts.serial_port}")
        except serial.SerialException as e:
            self.get_logger().error(f"Could not open serial port: {e}")
            raise

        # ROS interfaces
        self.sub = self.create_subscription(
            String, Consts.movement_topic, self.cmd_cb, 10)
        self.pub = self.create_publisher(String, Consts.feedback_topic, 10)

        # Thread for reading serial
        threading.Thread(target=self.read_serial, daemon=True).start()

    def cmd_cb(self, msg: String):
        cmd = msg.data.strip()
        self.get_logger().info(f'Received command: {cmd}')

        # Parse incoming JSON string
        try:
            json = Movement.to_json(cmd)
        except Exception as e:
            self.get_logger().error(f'JSON parse error: {e}')
            return

        # Normalize direction (uppercase for consistency)
        direction = str(json.get('direction', '')).upper()
        speed = int(json.get('speed', 0))
        angle = int(json.get('angle', 0))

        if direction not in Movement.valid_directions:
            self.get_logger().error(f'Invalid direction: {direction}')
            return

        # Build command string for ESP
        command = Movement.format_command(direction, speed, angle)
        self.get_logger().info(f'Sending command to serial: {command.strip()}')

        try:
            self.port.write(command.encode())
        except serial.SerialException as e:
            self.get_logger().error(f'Serial write failed: {e}')

    def read_serial(self):
        """ Continuously read feedback from ESP and republish on ROS topic """
        while rclpy.ok():
            try:
                if (line := self.port.readline()):
                    decoded = line.decode(errors='ignore').strip()
                    if decoded:
                        self.pub.publish(String(data=decoded))
                        self.get_logger().debug(f"Feedback: {decoded}")
            except serial.SerialException as e:
                self.get_logger().error(f"Serial read failed: {e}")
                break

def main(args=None):
    rclpy.init(args=args)
    node = SerialBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
