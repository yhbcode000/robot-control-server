import json
import threading
import time
from typing import Any, Dict, List, Optional

import serial
import serial.tools.list_ports

from src.schemas import RobotMode, RobotStatus


class USBController:
    def __init__(
        self,
        port: Optional[str] = None,
        baudrate: int = 9600,
        mujoco_fallback: Optional["MuJoCoController"] = None,
    ):
        self.port = port
        self.baudrate = baudrate
        self.serial_connection: Optional[serial.Serial] = None
        self.is_connected = False
        self.motor_commands = [0.0, 0.0, 0.0]
        self.current_position = [0.0, 0.0, 0.0]
        self.lock = threading.Lock()

        # MuJoCo fallback for testing/simulation when real hardware unavailable
        self.mujoco_fallback = mujoco_fallback
        self.use_fallback = False

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
        """Connect to the real robot via USB with MuJoCo fallback"""
        try:
            if self.port is None:
                print("⚠️  No USB port specified, using dummy mode")
                if self.mujoco_fallback:
                    print("🔄 MuJoCo fallback available for testing")
                    self.use_fallback = True
                self.is_connected = True  # Dummy connection
                return True

            # Real USB connection (commented out for now)
            # self.serial_connection = serial.Serial(self.port, self.baudrate, timeout=1)
            # self.is_connected = self.serial_connection.is_open

            # Dummy implementation with MuJoCo integration awareness
            print(f"[DUMMY USB] Connected to robot on port {self.port}")
            if self.mujoco_fallback:
                print("🔄 MuJoCo fallback controller available for comparison")
            self.is_connected = True
            self.use_fallback = False
            return True

        except Exception as e:
            print(f"❌ Failed to connect to robot: {e}")
            if self.mujoco_fallback:
                print("🔄 Attempting to use MuJoCo fallback...")
                self.use_fallback = True
                self.is_connected = True
                return True
            self.is_connected = False
            return False

    def disconnect(self):
        """Disconnect from the robot"""
        if self.serial_connection:
            self.serial_connection.close()

        self.is_connected = False
        print("[DUMMY] Disconnected from robot")

    def send_force_command(self, forces: List[float]) -> bool:
        """Send force command to real robot with MuJoCo fallback support"""
        if not self.is_connected:
            return False

        try:
            # Validate forces
            if len(forces) < 3:
                forces.extend([0.0] * (3 - len(forces)))

            for i, force in enumerate(forces[:3]):
                if not -10.0 <= force <= 10.0:
                    print(f"❌ Force {force} out of range [-10.0, 10.0]")
                    return False

            # If using MuJoCo fallback, delegate to it
            if self.use_fallback and self.mujoco_fallback:
                print("[USB->MuJoCo FALLBACK] Delegating force command to MuJoCo")
                return self.mujoco_fallback.send_force_command(forces)

            with self.lock:
                self.motor_commands = forces[:3]

            # Enhanced dummy implementation
            print(f"🔌 [USB] Force Command Sent: {forces[:3]}")
            print(
                f"  Mode: {'Fallback (MuJoCo)' if self.use_fallback else 'Real Hardware'}"
            )

            return True

        except Exception as e:
            print(f"❌ Failed to send force command: {e}")
            return False

    def read_feedback(self) -> Optional[Dict[str, Any]]:
        """Read feedback from real robot with MuJoCo fallback support"""
        if not self.is_connected:
            return None

        try:
            # If using MuJoCo fallback, get feedback from it
            if self.use_fallback and self.mujoco_fallback:
                mujoco_status = self.mujoco_fallback.get_status()
                return {
                    "position": mujoco_status.current_position,
                    "forces": mujoco_status.motor_forces,
                    "timestamp": time.time(),
                    "source": "mujoco_fallback",
                }

            # Dummy implementation - simulate realistic feedback
            import random

            # Add some noise to make it more realistic
            noise_level = 0.05
            feedback = {
                "position": [
                    self.current_position[0]
                    + random.uniform(-noise_level, noise_level),
                    self.current_position[1]
                    + random.uniform(-noise_level, noise_level),
                    self.current_position[2]
                    + random.uniform(-noise_level, noise_level),
                ],
                "forces": [f + random.uniform(-0.1, 0.1) for f in self.motor_commands],
                "timestamp": time.time(),
                "source": "dummy_hardware",
            }

            # Simulate position changes based on forces
            for i in range(3):
                self.current_position[i] += self.motor_commands[i] * 0.001

            # Real implementation would read via serial:
            # if self.serial_connection.in_waiting > 0:
            #     data = self.serial_connection.readline().decode().strip()
            #     feedback = json.loads(data)

            return feedback

        except Exception as e:
            print(f"❌ Failed to read feedback: {e}")
            return None

    def get_status(self) -> RobotStatus:
        """Get current robot status with MuJoCo integration awareness"""
        feedback = self.read_feedback()
        if feedback:
            self.current_position = feedback.get("position", self.current_position)

        # Enhanced status with fallback information
        status = RobotStatus(
            mode=RobotMode.REAL,
            is_connected=self.is_connected,
            current_position=self.current_position.copy(),
            motor_forces=self.motor_commands.copy(),
        )

        # Add fallback information if applicable
        if self.use_fallback:
            status.error_message = "Using MuJoCo fallback (real hardware not available)"

        return status

    def emergency_stop(self) -> bool:
        """Emergency stop with MuJoCo fallback support"""
        try:
            # If using MuJoCo fallback, delegate to it
            if self.use_fallback and self.mujoco_fallback:
                print("🚨 [USB->MuJoCo FALLBACK] Emergency stop via MuJoCo")
                return self.mujoco_fallback.emergency_stop()

            result = self.send_force_command([0.0, 0.0, 0.0])
            print("🚨 [USB] Emergency stop executed")
            return result
        except Exception as e:
            print(f"❌ Emergency stop failed: {e}")
            return False

    def set_mujoco_fallback(self, mujoco_controller):
        """Set MuJoCo controller as fallback"""
        self.mujoco_fallback = mujoco_controller
        print("🔄 MuJoCo fallback controller set")
