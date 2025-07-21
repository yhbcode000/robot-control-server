import json
import os
import time
from datetime import datetime
from typing import Any, Dict, Optional

from flask import Flask, request
from flask_cors import CORS
from flask_restx import Api, Namespace, Resource, fields

from src.mujoco_controller import MuJoCoController
from src.schemas import CommandResponse, RobotMode, RobotStatus
from src.usb_controller import USBController


class RobotFlaskServer:
    def __init__(
        self,
        host: str = "localhost",
        port: int = 5000,
        model_path: str = None,
        enable_viewer: bool = True,
    ):
        self.app = Flask(__name__)
        CORS(self.app)

        # Initialize Flask-RESTX for Swagger documentation
        self.api = Api(
            self.app,
            version="1.0.0",
            title="Robot Control API with MuJoCo",
            description="A comprehensive API for controlling soft robots via MuJoCo simulation and real hardware",
            doc="/docs/",
            authorizations={
                "apikey": {"type": "apiKey", "in": "header", "name": "X-API-Key"}
            },
        )

        self.host = host
        self.port = port
        self.model_path = model_path
        self.enable_viewer = enable_viewer

        # Controllers - MuJoCo is primary
        self.mujoco_controller: Optional[MuJoCoController] = None
        self.usb_controller: Optional[USBController] = None
        self.current_mode = RobotMode.SIMULATION  # Default to simulation

        # Initialize controllers
        self._initialize_controllers()

        # Register API models and namespaces
        self._register_api_models()
        self._register_namespaces()

    def _initialize_controllers(self):
        """Initialize robot controllers with MuJoCo as primary"""
        print("🔧 Initializing controllers...")

        # Initialize MuJoCo controller first (primary)
        try:
            if self.model_path and os.path.exists(self.model_path):
                self.mujoco_controller = MuJoCoController(
                    self.model_path, self.enable_viewer
                )
                self.mujoco_controller.start_simulation()
                viewer_status = "with viewer" if self.enable_viewer else "(headless)"
                print(
                    f"✅ MuJoCo controller initialized {viewer_status}: {self.model_path}"
                )

                # Verify simulation is running
                if not self.mujoco_controller.is_running:
                    print("⚠️  MuJoCo simulation not running, attempting restart...")
                    self.mujoco_controller.start_simulation()

                print(
                    f"🎯 MuJoCo simulation status: {'✅ RUNNING' if self.mujoco_controller.is_running else '❌ STOPPED'}"
                )
            else:
                print(f"❌ Model file not found: {self.model_path}")
                raise FileNotFoundError(
                    f"MuJoCo model file not found: {self.model_path}"
                )

        except Exception as e:
            print(f"💥 Failed to initialize MuJoCo controller: {e}")
            self.mujoco_controller = None

        # Initialize USB controller as secondary
        try:
            self.usb_controller = USBController()
            self.usb_controller.connect()
            print("✅ USB controller initialized (secondary/backup)")
        except Exception as e:
            print(f"⚠️  USB controller initialization failed: {e}")
            self.usb_controller = None

        # Set default mode based on available controllers
        if self.mujoco_controller:
            self.current_mode = RobotMode.SIMULATION
            print("🎮 Default mode: SIMULATION (MuJoCo)")
        elif self.usb_controller:
            self.current_mode = RobotMode.REAL
            print("🔌 Default mode: REAL (USB)")
        else:
            print("❌ No controllers available!")

    def _register_api_models(self):
        """Register API models for Swagger documentation"""

        # Force Command Model
        self.force_command_model = self.api.model(
            "ForceCommand",
            {
                "forces": fields.List(
                    fields.Float(min=-10.0, max=10.0),
                    required=True,
                    description="Force values for 3 motors [-10.0 to 10.0]",
                    example=[1.5, -2.0, 0.5],
                    min_items=3,
                    max_items=3,
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
                "simulation_time": fields.Float(
                    description="MuJoCo simulation time (simulation mode only)",
                    required=False,
                    example=15.234,
                ),
                "is_simulation_running": fields.Boolean(
                    description="MuJoCo simulation running status",
                    required=False,
                    example=True,
                ),
                "error_message": fields.String(
                    description="Error message if any", required=False
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
        """Register API namespaces with MuJoCo integration"""

        # Store reference to parent server for nested classes
        server_instance = self

        # Status Namespace
        status_ns = Namespace("status", description="Robot status operations")

        @status_ns.route("/")
        class StatusResource(Resource):
            @server_instance.api.doc("get_robot_status")
            @server_instance.api.marshal_with(server_instance.robot_status_model)
            @server_instance.api.response(200, "Success")
            @server_instance.api.response(
                500, "Internal server error", server_instance.error_model
            )
            def get(self):
                """Get current robot status with MuJoCo integration"""
                try:
                    status = None

                    if (
                        server_instance.current_mode == RobotMode.SIMULATION
                        and server_instance.mujoco_controller
                    ):
                        status = server_instance.mujoco_controller.get_status()

                        # Add MuJoCo specific information
                        status_dict = status.to_dict()
                        if server_instance.mujoco_controller.data:
                            status_dict["simulation_time"] = float(
                                server_instance.mujoco_controller.data.time
                            )
                        status_dict["is_simulation_running"] = (
                            server_instance.mujoco_controller.is_running
                        )

                        return status_dict

                    elif (
                        server_instance.current_mode == RobotMode.REAL
                        and server_instance.usb_controller
                    ):
                        status = server_instance.usb_controller.get_status()
                        return status.to_dict()
                    else:
                        # Return error status
                        status = RobotStatus(
                            mode=server_instance.current_mode,
                            is_connected=False,
                            current_position=[0.0, 0.0, 0.0],
                            motor_forces=[0.0, 0.0, 0.0],
                            error_message=f"No controller available for mode: {server_instance.current_mode.value}",
                        )
                        return status.to_dict()

                except Exception as e:
                    server_instance.api.abort(
                        500, f"Failed to get robot status: {str(e)}"
                    )

        # Enhanced Mode Namespace
        mode_ns = Namespace("mode", description="Robot mode operations")

        @mode_ns.route("/")
        class ModeResource(Resource):
            @server_instance.api.doc("get_robot_mode")
            @server_instance.api.response(200, "Success")
            def get(self):
                """Get current robot mode and controller status"""
                controller_status = {
                    "mujoco_available": server_instance.mujoco_controller is not None,
                    "mujoco_running": (
                        server_instance.mujoco_controller.is_running
                        if server_instance.mujoco_controller
                        else False
                    ),
                    "usb_available": server_instance.usb_controller is not None,
                    "usb_connected": (
                        server_instance.usb_controller.is_connected
                        if server_instance.usb_controller
                        else False
                    ),
                }

                return {
                    "mode": server_instance.current_mode.value,
                    "controllers": controller_status,
                }

            @server_instance.api.doc("set_robot_mode")
            @server_instance.api.expect(server_instance.mode_change_model)
            @server_instance.api.response(200, "Success")
            @server_instance.api.response(
                400,
                "Invalid mode or controller unavailable",
                server_instance.error_model,
            )
            @server_instance.api.response(
                500, "Internal server error", server_instance.error_model
            )
            def post(self):
                """Set robot mode with controller validation"""
                try:
                    data = request.get_json()
                    if not data or "mode" not in data:
                        server_instance.api.abort(400, "Missing 'mode' field")

                    new_mode = RobotMode(data["mode"])

                    # Validate controller availability
                    if new_mode == RobotMode.SIMULATION:
                        if not server_instance.mujoco_controller:
                            server_instance.api.abort(
                                400, "MuJoCo controller not available"
                            )
                        if not server_instance.mujoco_controller.is_running:
                            print("🔄 Restarting MuJoCo simulation...")
                            server_instance.mujoco_controller.start_simulation()

                    elif new_mode == RobotMode.REAL:
                        if (
                            not server_instance.usb_controller
                            or not server_instance.usb_controller.is_connected
                        ):
                            server_instance.api.abort(
                                400, "USB controller not available or connected"
                            )

                    server_instance.current_mode = new_mode
                    print(f"🎯 Mode changed to: {new_mode.value}")

                    return {
                        "message": f"Mode changed to {new_mode.value}",
                        "mode": new_mode.value,
                    }

                except ValueError:
                    server_instance.api.abort(
                        400,
                        f"Invalid mode. Must be one of: {[m.value for m in RobotMode]}",
                    )
                except Exception as e:
                    server_instance.api.abort(500, str(e))

        # Motor Control Namespace
        motor_ns = Namespace("motor", description="Direct motor control operations")

        @motor_ns.route("/forces")
        class ForceCommandResource(Resource):
            @server_instance.api.doc("send_force_command")
            @server_instance.api.expect(server_instance.force_command_model)
            @server_instance.api.marshal_with(server_instance.command_response_model)
            @server_instance.api.response(200, "Success")
            @server_instance.api.response(
                400, "Invalid force command", server_instance.error_model
            )
            @server_instance.api.response(
                500, "Internal server error", server_instance.error_model
            )
            def post(self):
                """Send direct force command to motors"""
                try:
                    data = request.get_json()
                    if not data:
                        server_instance.api.abort(400, "No JSON data provided")

                    return server_instance._handle_force_command(data)

                except Exception as e:
                    server_instance.api.abort(500, str(e))

        @motor_ns.route("/emergency_stop")
        class EmergencyStopResource(Resource):
            @server_instance.api.doc("emergency_stop")
            @server_instance.api.marshal_with(server_instance.command_response_model)
            @server_instance.api.response(200, "Success")
            @server_instance.api.response(
                500, "Internal server error", server_instance.error_model
            )
            def post(self):
                """Emergency stop - immediately halt all motors"""
                return server_instance._handle_emergency_stop()

        # Add MuJoCo specific namespace
        mujoco_ns = Namespace(
            "mujoco", description="MuJoCo simulation specific operations"
        )

        @mujoco_ns.route("/restart")
        class MuJoCoRestartResource(Resource):
            @server_instance.api.doc("restart_mujoco_simulation")
            @server_instance.api.response(200, "Success")
            @server_instance.api.response(
                500, "Failed to restart simulation", server_instance.error_model
            )
            def post(self):
                """Restart MuJoCo simulation"""
                if not server_instance.mujoco_controller:
                    server_instance.api.abort(500, "MuJoCo controller not available")

                try:
                    server_instance.mujoco_controller.stop_simulation()
                    server_instance.mujoco_controller.start_simulation()

                    return {
                        "message": "MuJoCo simulation restarted successfully",
                        "is_running": server_instance.mujoco_controller.is_running,
                    }
                except Exception as e:
                    server_instance.api.abort(
                        500, f"Failed to restart MuJoCo simulation: {str(e)}"
                    )

        @mujoco_ns.route("/info")
        class MuJoCoInfoResource(Resource):
            @server_instance.api.doc("get_mujoco_info")
            @server_instance.api.response(200, "Success")
            @server_instance.api.response(
                500, "MuJoCo not available", server_instance.error_model
            )
            def get(self):
                """Get detailed MuJoCo simulation information"""
                if not server_instance.mujoco_controller:
                    server_instance.api.abort(500, "MuJoCo controller not available")

                try:
                    info = {
                        "model_path": server_instance.mujoco_controller.model_path,
                        "is_running": server_instance.mujoco_controller.is_running,
                        "viewer_enabled": server_instance.mujoco_controller.enable_viewer,
                    }

                    if server_instance.mujoco_controller.model:
                        info.update(
                            {
                                "model_info": {
                                    "nbody": server_instance.mujoco_controller.model.nbody,
                                    "njnt": server_instance.mujoco_controller.model.njnt,
                                    "nq": server_instance.mujoco_controller.model.nq,
                                    "nu": server_instance.mujoco_controller.model.nu,
                                    "timestep": server_instance.mujoco_controller.model.opt.timestep,
                                }
                            }
                        )

                    if server_instance.mujoco_controller.data:
                        info["simulation_time"] = float(
                            server_instance.mujoco_controller.data.time
                        )

                    return info

                except Exception as e:
                    server_instance.api.abort(500, str(e))

        # Register all namespaces
        self.api.add_namespace(status_ns, path="/api/v1/status")
        self.api.add_namespace(mode_ns, path="/api/v1/mode")
        self.api.add_namespace(motor_ns, path="/api/v1/motor")
        self.api.add_namespace(mujoco_ns, path="/api/v1/mujoco")

    def _handle_force_command(self, payload: Dict[str, Any]):
        """Handle force control commands"""
        try:
            forces = payload.get("forces", [])
            if not forces or len(forces) != 3:
                self.api.abort(400, "forces must be a list of 3 float values")

            # Validate force ranges
            for i, force in enumerate(forces):
                if not isinstance(force, (int, float)) or not -10.0 <= force <= 10.0:
                    self.api.abort(
                        400, f"Force {i+1} ({force}) out of range [-10.0, 10.0]"
                    )

            success = False
            controller_name = ""

            # Use MuJoCo controller for simulation mode
            if self.current_mode == RobotMode.SIMULATION and self.mujoco_controller:
                success = self.mujoco_controller.send_force_command(forces)
                controller_name = "MuJoCo Simulation"

                if not self.mujoco_controller.is_running:
                    print("⚠️  MuJoCo simulation stopped, attempting restart...")
                    self.mujoco_controller.start_simulation()

            # Use USB controller for real mode
            elif self.current_mode == RobotMode.REAL and self.usb_controller:
                success = self.usb_controller.send_force_command(forces)
                controller_name = "Real Robot (USB)"
            else:
                self.api.abort(
                    500, f"No controller available for mode: {self.current_mode.value}"
                )

            if success:
                response = CommandResponse(
                    success=True,
                    message=f"Force command sent successfully via {controller_name}",
                    data={"forces": forces},
                )
                return response.to_dict()
            else:
                self.api.abort(
                    500, f"Failed to send force command via {controller_name}"
                )

        except ValueError as e:
            self.api.abort(400, f"Invalid force command: {e}")
        except Exception as e:
            self.api.abort(500, str(e))

    def _handle_emergency_stop(self):
        """Handle emergency stop - prioritize MuJoCo controller"""
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
        print(f"🚀 Starting Robot Control Server on {self.host}:{self.port}")
        print(f"🎯 Current mode: {self.current_mode.value}")
        print(
            f"📊 MuJoCo controller: {'✅ Available' if self.mujoco_controller else '❌ Not Available'}"
        )
        print(
            f"🔌 USB controller: {'✅ Available' if self.usb_controller else '❌ Not Available'}"
        )
        print(f"📖 Swagger documentation: http://{self.host}:{self.port}/docs/")

        self.app.run(host=self.host, port=self.port, debug=debug)

    def shutdown(self):
        """Shutdown server and cleanup controllers"""
        print("🧹 Shutting down Robot Control Server...")

        # Shutdown MuJoCo controller first (primary)
        if self.mujoco_controller:
            print("🛑 Stopping MuJoCo simulation...")
            self.mujoco_controller.emergency_stop()
            self.mujoco_controller.stop_simulation()
            print("✅ MuJoCo controller shutdown complete")

        # Shutdown USB controller
        if self.usb_controller:
            print("🔌 Disconnecting USB controller...")
            self.usb_controller.disconnect()
            print("✅ USB controller shutdown complete")

        print("✅ All controllers shutdown complete")


def create_app(model_path: str = None, enable_viewer: bool = True) -> RobotFlaskServer:
    """Factory function to create Flask server with MuJoCo integration"""
    # Use default model path if not provided
    if not model_path:
        model_path = "e:\\桌面\\SoftRob\\Mujoco_0610\\robot\\model\\spiral_3D.xml"

    return RobotFlaskServer(model_path=model_path, enable_viewer=enable_viewer)


if __name__ == "__main__":
    server = create_app()
    try:
        server.run(debug=True)
    except KeyboardInterrupt:
        server.shutdown()
