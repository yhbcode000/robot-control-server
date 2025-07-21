import json
import threading
import time
from typing import Any, Dict, List, Optional

import serial
import serial.tools.list_ports

from src.schemas import MotorCommand, RobotMode, RobotStatus


class USBController:
    def __init__(self, port: Optional[str] = None, baudrate: int = 9600):
        self.port = port
        self.baudrate = baudrate
        self.serial_connection: Optional[serial.Serial] = None
        self.is_connected = False
        self.motor_commands = [0.0, 0.0, 0.0]
        self.current_position = [0.0, 0.0, 0.0]
        self.lock = threading.Lock()

        if port is None:
            self.port = self._auto_detect_port()

    def _auto_detect_port(self) -> Optional[str]:
        """Auto-detect robot USB port"""
        ports = serial.tools.list_ports.comports()
        print("Available USB ports:")
        for port in ports:
            print(f"  {port.device}: {port.description}")

        # For now, return None since we're using dummy implementation
        return None

    def connect(self) -> bool:
        """Connect to the real robot via USB"""
        try:
            if self.port is None:
                print("No USB port specified, using dummy mode")
                self.is_connected = True  # Dummy connection
                return True

            # Real USB connection (commented out for now)
            # self.serial_connection = serial.Serial(self.port, self.baudrate, timeout=1)
            # self.is_connected = self.serial_connection.is_open

            # Dummy implementation
            print(f"[DUMMY] Connected to robot on port {self.port}")
            self.is_connected = True
            return True

        except Exception as e:
            print(f"Failed to connect to robot: {e}")
            self.is_connected = False
            return False

    def disconnect(self):
        """Disconnect from the robot"""
        if self.serial_connection:
            self.serial_connection.close()

        self.is_connected = False
        print("[DUMMY] Disconnected from robot")

    def send_motor_command(self, command: MotorCommand) -> bool:
        """Send motor command to real robot"""
        if not self.is_connected:
            return False

        try:
            with self.lock:
                self.motor_commands[0] = command.motor_1_force
                self.motor_commands[1] = command.motor_2_force
                self.motor_commands[2] = command.motor_3_force

            # Dummy implementation - just print to terminal
            print(f"[DUMMY USB] Motor Command Sent:")
            print(f"  Motor 1 Force: {command.motor_1_force:.3f}")
            print(f"  Motor 2 Force: {command.motor_2_force:.3f}")
            print(f"  Motor 3 Force: {command.motor_3_force:.3f}")
            if command.duration:
                print(f"  Duration: {command.duration:.3f}s")

            # Real implementation would send via serial:
            # command_data = json.dumps(command.to_dict())
            # self.serial_connection.write(command_data.encode())

            return True

        except Exception as e:
            print(f"Failed to send motor command: {e}")
            return False

    def read_feedback(self) -> Optional[Dict[str, Any]]:
        """Read feedback from real robot"""
        if not self.is_connected:
            return None

        try:
            # Dummy implementation - simulate feedback
            import random

            feedback = {
                "position": [
                    random.uniform(-1, 1),
                    random.uniform(-1, 1),
                    random.uniform(-1, 1),
                ],
                "forces": self.motor_commands.copy(),
                "timestamp": time.time(),
            }

            print(f"[DUMMY USB] Feedback received: {feedback}")

            # Real implementation would read via serial:
            # if self.serial_connection.in_waiting > 0:
            #     data = self.serial_connection.readline().decode().strip()
            #     feedback = json.loads(data)

            return feedback

        except Exception as e:
            print(f"Failed to read feedback: {e}")
            return None

    def get_status(self) -> RobotStatus:
        """Get current robot status"""
        feedback = self.read_feedback()
        if feedback:
            self.current_position = feedback.get("position", self.current_position)

        return RobotStatus(
            mode=RobotMode.REAL,
            is_connected=self.is_connected,
            current_position=self.current_position.copy(),
            motor_forces=self.motor_commands.copy(),
        )

    def emergency_stop(self) -> bool:
        """Emergency stop - zero all forces"""
        try:
            emergency_command = MotorCommand(0.0, 0.0, 0.0)
            result = self.send_motor_command(emergency_command)
            print("[DUMMY USB] Emergency stop executed")
            return result
        except Exception as e:
            print(f"Emergency stop failed: {e}")
            return False
