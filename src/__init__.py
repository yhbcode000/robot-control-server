"""
Robot Control System Package
Provides Flask server for MuJoCo simulation and real robot control
"""

__version__ = "1.0.0"
__author__ = "Robot Control Team"

from src.flask_server import RobotFlaskServer

# Import main controllers and schemas for easy access
from src.mujoco_controller import MuJoCoController
from src.schemas import RobotMode, RobotStatus
from src.usb_controller import USBController

__all__ = [
    "MuJoCoController",
    "USBController",
    "MotorCommand",
    "RobotStatus",
    "RobotMode",
    "RobotFlaskServer",
]
