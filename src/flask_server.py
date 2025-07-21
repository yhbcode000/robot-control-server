import json
import os
import time
from datetime import datetime
from typing import Any, Dict, Optional

from flask import Flask, request
from flask_cors import CORS
from flask_restx import Api, Namespace, Resource, fields

from src.mujoco_controller import MuJoCoController
from src.schemas import (
    CommandRequest,
    CommandResponse,
    CommandType,
    MotorCommand,
    RobotMode,
    RobotStatus,
)
from src.usb_controller import USBController


class RobotFlaskServer:
    def __init__(self, host: str = "localhost", port: int = 5000):
        self.app = Flask(__name__)
        CORS(self.app)

        # Initialize Flask-RESTX for Swagger documentation
        self.api = Api(
            self.app,
            version="1.0.0",
            title="Robot Control API",
            description="A comprehensive API for controlling soft robots via MuJoCo simulation or real hardware",
            doc="/docs/",
            authorizations={
                "apikey": {"type": "apiKey", "in": "header", "name": "X-API-Key"}
            },
        )

        self.host = host
        self.port = port

        # Controllers
        self.mujoco_controller: Optional[MuJoCoController] = None
        self.usb_controller: Optional[USBController] = None
        self.current_mode = RobotMode.SIMULATION

        # Initialize controllers
        self._initialize_controllers()

        # Register API models and namespaces
        self._register_api_models()
        self._register_namespaces()

    def _initialize_controllers(self):
        """Initialize robot controllers"""
        try:
            # Check if we're in testing mode to disable viewer
            enable_viewer = not (
                hasattr(self.app, "config") and self.app.config.get("TESTING", False)
            )

            # Initialize MuJoCo controller
            try:
                from ament_index_python.packages import get_package_share_directory

                resource_path = os.path.join(
                    get_package_share_directory("sim"), "resource"
                )
                model_path = os.path.join(
                    resource_path, "physics_model", "spiral_3D.xml"
                )
            except ImportError:
                model_path = None

            # Fallback model path if package not found
            if model_path is None or not os.path.exists(model_path):
                # Try the existing model file in the repository
                model_path = os.path.join(
                    os.path.dirname(__file__), "..", "model", "spiral_3D.xml"
                )

            if os.path.exists(model_path):
                self.mujoco_controller = MuJoCoController(model_path, enable_viewer)
                self.mujoco_controller.start_simulation()
                viewer_status = "with viewer" if enable_viewer else "(headless)"
                print(f"MuJoCo controller initialized {viewer_status}: {model_path}")
            else:
                print(f"Warning: Model file not found at {model_path}")

            # Initialize USB controller
            self.usb_controller = USBController()
            self.usb_controller.connect()
            print("USB controller initialized")

        except Exception as e:
            print(f"Failed to initialize controllers: {e}")

    def _register_api_models(self):
        """Register API models for Swagger documentation"""

        # Motor Command Model
        self.motor_command_model = self.api.model(
            "MotorCommand",
            {
                "motor_1_force": fields.Float(
                    required=True,
                    description="Force for motor 1 (-10.0 to 10.0)",
                    min=-10.0,
                    max=10.0,
                    example=1.5,
                ),
                "motor_2_force": fields.Float(
                    required=True,
                    description="Force for motor 2 (-10.0 to 10.0)",
                    min=-10.0,
                    max=10.0,
                    example=-2.0,
                ),
                "motor_3_force": fields.Float(
                    required=True,
                    description="Force for motor 3 (-10.0 to 10.0)",
                    min=-10.0,
                    max=10.0,
                    example=0.5,
                ),
                "duration": fields.Float(
                    required=False,
                    description="Command duration in seconds",
                    example=2.0,
                ),
            },
        )

        # Robot Status Model
        self.robot_status_model = self.api.model(
            "RobotStatus",
            {
                "mode": fields.String(
                    description="Current robot mode",
                    enum=["simulation", "real"],
                    example="simulation",
                ),
                "is_connected": fields.Boolean(
                    description="Connection status", example=True
                ),
                "current_position": fields.List(
                    fields.Float(),
                    description="Current robot position [x, y, z]",
                    example=[0.1, 0.2, 0.3],
                ),
                "motor_forces": fields.List(
                    fields.Float(),
                    description="Current motor forces",
                    example=[1.0, -0.5, 2.0],
                ),
                "error_message": fields.String(
                    description="Error message if any", required=False
                ),
            },
        )

        # Command Request Model
        self.command_request_model = self.api.model(
            "CommandRequest",
            {
                "command_type": fields.String(
                    required=True,
                    description="Type of command",
                    enum=[
                        "motor_control",
                        "position_control",
                        "emergency_stop",
                        "status_request",
                    ],
                    example="motor_control",
                ),
                "payload": fields.Raw(
                    required=True,
                    description="Command payload (varies by command type)",
                    example={
                        "motor_1_force": 1.5,
                        "motor_2_force": -2.0,
                        "motor_3_force": 0.5,
                    },
                ),
                "timestamp": fields.String(
                    required=False,
                    description="Command timestamp",
                    example="2023-12-01T10:30:00Z",
                ),
            },
        )

        # Command Response Model
        self.command_response_model = self.api.model(
            "CommandResponse",
            {
                "success": fields.Boolean(
                    description="Operation success status", example=True
                ),
                "message": fields.String(
                    description="Response message",
                    example="Motor command sent successfully",
                ),
                "data": fields.Raw(
                    description="Additional response data", required=False
                ),
            },
        )

        # Mode Change Model
        self.mode_change_model = self.api.model(
            "ModeChange",
            {
                "mode": fields.String(
                    required=True,
                    description="New robot mode",
                    enum=["simulation", "real"],
                    example="simulation",
                )
            },
        )

        # Error Model
        self.error_model = self.api.model(
            "Error",
            {
                "error": fields.String(
                    description="Error message", example="Invalid motor command"
                ),
                "message": fields.String(
                    description="Detailed error description", required=False
                ),
            },
        )

    def _register_namespaces(self):
        """Register API namespaces"""

        # Status Namespace
        status_ns = Namespace("status", description="Robot status operations")

        @status_ns.route("/")
        class StatusResource(Resource):
            @self.api.doc("get_robot_status")
            @self.api.marshal_with(self.robot_status_model)
            @self.api.response(200, "Success")
            @self.api.response(500, "Internal server error", self.error_model)
            def get(self):
                """Get current robot status"""
                try:
                    if (
                        self.current_mode == RobotMode.SIMULATION
                        and self.mujoco_controller
                    ):
                        status = self.mujoco_controller.get_status()
                    elif self.current_mode == RobotMode.REAL and self.usb_controller:
                        status = self.usb_controller.get_status()
                    else:
                        status = RobotStatus(
                            mode=self.current_mode,
                            is_connected=False,
                            current_position=[0.0, 0.0, 0.0],
                            motor_forces=[0.0, 0.0, 0.0],
                            error_message="No controller available",
                        )

                    return status.to_dict()

                except Exception as e:
                    self.api.abort(500, f"Failed to get robot status: {str(e)}")

        # Mode Namespace
        mode_ns = Namespace("mode", description="Robot mode operations")

        @mode_ns.route("/")
        class ModeResource(Resource):
            @self.api.doc("get_robot_mode")
            @self.api.response(200, "Success")
            def get(self):
                """Get current robot mode"""
                return {"mode": self.current_mode.value}

            @self.api.doc("set_robot_mode")
            @self.api.expect(self.mode_change_model)
            @self.api.response(200, "Success")
            @self.api.response(400, "Invalid mode", self.error_model)
            @self.api.response(500, "Internal server error", self.error_model)
            def post(self):
                """Set robot mode"""
                try:
                    data = request.get_json()
                    if not data or "mode" not in data:
                        self.api.abort(400, "Missing 'mode' field")

                    new_mode = RobotMode(data["mode"])
                    self.current_mode = new_mode

                    return {
                        "message": f"Mode changed to {new_mode.value}",
                        "mode": new_mode.value,
                    }

                except ValueError:
                    self.api.abort(
                        400,
                        f"Invalid mode. Must be one of: {[m.value for m in RobotMode]}",
                    )
                except Exception as e:
                    self.api.abort(500, str(e))

        # Command Namespace
        command_ns = Namespace("command", description="Robot command operations")

        @command_ns.route("/")
        class CommandResource(Resource):
            @self.api.doc("send_robot_command")
            @self.api.expect(self.command_request_model)
            @self.api.marshal_with(self.command_response_model)
            @self.api.response(200, "Success")
            @self.api.response(400, "Invalid command", self.error_model)
            @self.api.response(500, "Internal server error", self.error_model)
            def post(self):
                """Send command to robot"""
                try:
                    data = request.get_json()
                    if not data:
                        self.api.abort(400, "No JSON data provided")

                    # Parse command request
                    command_request = CommandRequest.from_dict(data)

                    # Handle different command types
                    if command_request.command_type == CommandType.MOTOR_CONTROL:
                        return self._handle_motor_command(command_request.payload)
                    elif command_request.command_type == CommandType.EMERGENCY_STOP:
                        return self._handle_emergency_stop()
                    elif command_request.command_type == CommandType.STATUS_REQUEST:
                        status_resource = StatusResource()
                        return status_resource.get()
                    else:
                        self.api.abort(
                            400,
                            f"Unsupported command type: {command_request.command_type.value}",
                        )

                except Exception as e:
                    self.api.abort(500, str(e))

        # Motor Control Namespace
        motor_ns = Namespace("motor", description="Direct motor control operations")

        @motor_ns.route("/command")
        class MotorCommandResource(Resource):
            @self.api.doc("send_motor_command")
            @self.api.expect(self.motor_command_model)
            @self.api.marshal_with(self.command_response_model)
            @self.api.response(200, "Success")
            @self.api.response(400, "Invalid motor command", self.error_model)
            @self.api.response(500, "Internal server error", self.error_model)
            def post(self):
                """Send direct motor command"""
                try:
                    data = request.get_json()
                    if not data:
                        self.api.abort(400, "No JSON data provided")

                    return self._handle_motor_command(data)

                except Exception as e:
                    self.api.abort(500, str(e))

        @motor_ns.route("/emergency_stop")
        class EmergencyStopResource(Resource):
            @self.api.doc("emergency_stop")
            @self.api.marshal_with(self.command_response_model)
            @self.api.response(200, "Success")
            @self.api.response(500, "Internal server error", self.error_model)
            def post(self):
                """Emergency stop - immediately halt all motors"""
                return self._handle_emergency_stop()

        # Register namespaces with API
        self.api.add_namespace(status_ns, path="/api/v1/status")
        self.api.add_namespace(mode_ns, path="/api/v1/mode")
        self.api.add_namespace(command_ns, path="/api/v1/command")
        self.api.add_namespace(motor_ns, path="/api/v1/motor")

        # Root endpoint
        @self.api.route("/")
        class RootResource(Resource):
            @self.api.doc("get_api_info")
            def get(self):
                """Get API information"""
                return {
                    "name": "Robot Control Server",
                    "version": "1.0.0",
                    "description": "A comprehensive API for controlling soft robots",
                    "documentation": "/docs/",
                    "endpoints": {
                        "status": "/api/v1/status/",
                        "mode": "/api/v1/mode/",
                        "command": "/api/v1/command/",
                        "motor": "/api/v1/motor/",
                        "emergency_stop": "/api/v1/motor/emergency_stop",
                    },
                }

    def _handle_motor_command(self, payload: Dict[str, Any]):
        """Handle motor control commands"""
        try:
            motor_command = MotorCommand.from_dict(payload)

            success = False
            controller_name = ""

            if self.current_mode == RobotMode.SIMULATION and self.mujoco_controller:
                success = self.mujoco_controller.send_motor_command(motor_command)
                controller_name = "MuJoCo Simulation"
            elif self.current_mode == RobotMode.REAL and self.usb_controller:
                success = self.usb_controller.send_motor_command(motor_command)
                controller_name = "Real Robot (USB)"
            else:
                self.api.abort(
                    500, f"No controller available for mode: {self.current_mode.value}"
                )

            if success:
                response = CommandResponse(
                    success=True,
                    message=f"Motor command sent successfully via {controller_name}",
                    data=motor_command.to_dict(),
                )
                return response.to_dict()
            else:
                self.api.abort(
                    500, f"Failed to send motor command via {controller_name}"
                )

        except ValueError as e:
            self.api.abort(400, f"Invalid motor command: {e}")
        except Exception as e:
            self.api.abort(500, str(e))

    def _handle_emergency_stop(self):
        """Handle emergency stop"""
        try:
            success = False
            controller_name = ""

            if self.current_mode == RobotMode.SIMULATION and self.mujoco_controller:
                success = self.mujoco_controller.emergency_stop()
                controller_name = "MuJoCo Simulation"
            elif self.current_mode == RobotMode.REAL and self.usb_controller:
                success = self.usb_controller.emergency_stop()
                controller_name = "Real Robot (USB)"
            else:
                self.api.abort(
                    500, f"No controller available for mode: {self.current_mode.value}"
                )

            if success:
                response = CommandResponse(
                    success=True,
                    message=f"Emergency stop executed via {controller_name}",
                )
                return response.to_dict()
            else:
                self.api.abort(500, f"Emergency stop failed via {controller_name}")

        except Exception as e:
            self.api.abort(500, str(e))

    def run(self, debug: bool = True):
        """Run the Flask server"""
        print(f"Starting Robot Control Server on {self.host}:{self.port}")
        print(f"Current mode: {self.current_mode.value}")
        print(
            f"Swagger documentation available at: http://{self.host}:{self.port}/docs/"
        )
        self.app.run(host=self.host, port=self.port, debug=debug)

    def shutdown(self):
        """Shutdown server and cleanup controllers"""
        print("Shutting down Robot Control Server...")

        if self.mujoco_controller:
            self.mujoco_controller.stop_simulation()

        if self.usb_controller:
            self.usb_controller.disconnect()


def create_app() -> RobotFlaskServer:
    """Factory function to create Flask server"""
    return RobotFlaskServer()


if __name__ == "__main__":
    server = create_app()
    try:
        server.run(debug=True)
    except KeyboardInterrupt:
        server.shutdown()
