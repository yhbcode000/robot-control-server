import json
from dataclasses import dataclass
from enum import Enum
from typing import Any, Dict, List, Optional


class RobotMode(Enum):
    SIMULATION = "simulation"
    REAL = "real"


@dataclass
class RobotStatus:
    mode: RobotMode
    is_connected: bool
    current_position: List[float]
    motor_forces: List[float]
    error_message: Optional[str] = None
    # Enhanced fields for MuJoCo integration
    simulation_time: Optional[float] = None
    is_simulation_running: Optional[bool] = None
    controller_type: Optional[str] = None

    def to_dict(self) -> Dict[str, Any]:
        result = {
            "mode": self.mode.value,
            "is_connected": self.is_connected,
            "current_position": self.current_position,
            "motor_forces": self.motor_forces,
            "error_message": self.error_message,
        }

        # Add MuJoCo specific fields if available
        if self.simulation_time is not None:
            result["simulation_time"] = self.simulation_time
        if self.is_simulation_running is not None:
            result["is_simulation_running"] = self.is_simulation_running
        if self.controller_type is not None:
            result["controller_type"] = self.controller_type

        return result


@dataclass
class CommandResponse:
    success: bool
    message: str
    data: Optional[Dict[str, Any]] = None

    def to_dict(self) -> Dict[str, Any]:
        return {"success": self.success, "message": self.message, "data": self.data}


@dataclass
class SimulationInfo:
    """Information about MuJoCo simulation state"""

    model_path: str
    is_running: bool
    simulation_time: float
    timestep: float
    viewer_enabled: bool
    model_bodies: int
    model_joints: int
    model_actuators: int

    def to_dict(self) -> Dict[str, Any]:
        return {
            "model_path": self.model_path,
            "is_running": self.is_running,
            "simulation_time": self.simulation_time,
            "timestep": self.timestep,
            "viewer_enabled": self.viewer_enabled,
            "model_bodies": self.model_bodies,
            "model_joints": self.model_joints,
            "model_actuators": self.model_actuators,
        }
