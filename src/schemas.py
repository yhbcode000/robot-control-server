import json
from dataclasses import dataclass
from enum import Enum
from typing import Any, Dict, List, Optional


class RobotMode(Enum):
    SIMULATION = "simulation"
    REAL = "real"


class CommandType(Enum):
    MOTOR_CONTROL = "motor_control"
    POSITION_CONTROL = "position_control"
    EMERGENCY_STOP = "emergency_stop"
    STATUS_REQUEST = "status_request"


@dataclass
class MotorCommand:
    motor_1_force: float
    motor_2_force: float
    motor_3_force: float
    duration: Optional[float] = None

    def __post_init__(self):
        # Validate force values
        for force in [self.motor_1_force, self.motor_2_force, self.motor_3_force]:
            if not -10.0 <= force <= 10.0:
                raise ValueError(f"Force {force} out of range [-10.0, 10.0]")

    def to_dict(self) -> Dict[str, Any]:
        return {
            "motor_1_force": self.motor_1_force,
            "motor_2_force": self.motor_2_force,
            "motor_3_force": self.motor_3_force,
            "duration": self.duration,
        }

    @classmethod
    def from_dict(cls, data: Dict[str, Any]) -> "MotorCommand":
        return cls(
            motor_1_force=data["motor_1_force"],
            motor_2_force=data["motor_2_force"],
            motor_3_force=data["motor_3_force"],
            duration=data.get("duration"),
        )


@dataclass
class RobotStatus:
    mode: RobotMode
    is_connected: bool
    current_position: List[float]
    motor_forces: List[float]
    error_message: Optional[str] = None

    def to_dict(self) -> Dict[str, Any]:
        return {
            "mode": self.mode.value,
            "is_connected": self.is_connected,
            "current_position": self.current_position,
            "motor_forces": self.motor_forces,
            "error_message": self.error_message,
        }


@dataclass
class CommandRequest:
    command_type: CommandType
    payload: Dict[str, Any]
    timestamp: Optional[str] = None

    def to_dict(self) -> Dict[str, Any]:
        return {
            "command_type": self.command_type.value,
            "payload": self.payload,
            "timestamp": self.timestamp,
        }

    @classmethod
    def from_dict(cls, data: Dict[str, Any]) -> "CommandRequest":
        return cls(
            command_type=CommandType(data["command_type"]),
            payload=data["payload"],
            timestamp=data.get("timestamp"),
        )


@dataclass
class CommandResponse:
    success: bool
    message: str
    data: Optional[Dict[str, Any]] = None

    def to_dict(self) -> Dict[str, Any]:
        return {"success": self.success, "message": self.message, "data": self.data}
