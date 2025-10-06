import json
import os

class Consts:
    # Try to auto-detect ESP32 port
    default_ports = ["/dev/ttyACM0", "/dev/ttyUSB0", "/dev/ttyAMA0"]
    serial_port = None

    for p in default_ports:
        if os.path.exists(p):
            serial_port = p
            break
    if serial_port is None:
        # fallback if nothing found, so at least node doesn't crash
        serial_port = "/dev/ttyACM0"

    serial_baudrate = 115200
    serial_timeout = 0.1

    movement_topic = "/movement"
    scan_topic = "/scan"
    feedback_topic = "/esp_feedback"


class Movement:
    @staticmethod
    def to_json(string: str) -> dict:
        return json.loads(string)

    @staticmethod
    def format_command(direction: str, speed: int, angle: int) -> str:
        return json.dumps({
            "direction": direction,
            "speed": speed,
            "angle": angle
        }) + "\n"

    valid_directions = ["FORWARD", "BACKWARD", "LEFT", "RIGHT", "TURN", "STOP"]
