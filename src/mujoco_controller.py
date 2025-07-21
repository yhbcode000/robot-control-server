import os
import sys
import threading
import time
from typing import Any, Dict, List, Optional

import numpy as np
from mujoco import MjData, MjModel, mj_step, viewer

from src.schemas import MotorCommand, RobotMode, RobotStatus


class MuJoCoController:
    def __init__(self, model_path: str, enable_viewer: bool = True):
        self.model_path = model_path
        self.model: Optional[MjModel] = None
        self.data: Optional[MjData] = None
        self.viewer_handle = None
        self.enable_viewer = enable_viewer

        self.motor_commands = [0.0, 0.0, 0.0]
        self.current_position = [0.0, 0.0, 0.0]
        self.is_running = False
        self.simulation_thread: Optional[threading.Thread] = None
        self.lock = threading.Lock()

        self._initialize_simulation()

    def _initialize_simulation(self):
        """Initialize MuJoCo model and data"""
        try:
            if not os.path.exists(self.model_path):
                raise FileNotFoundError(f"Model file not found: {self.model_path}")

            print(f"Loading MuJoCo model from: {self.model_path}")
            self.model = MjModel.from_xml_path(self.model_path)
            print(f"Model loaded successfully: {self.model.nbody} bodies, {self.model.nu} actuators")
            
            self.data = MjData(self.model)
            print("MuJoCo data structure initialized")

            # Only create viewer if explicitly enabled
            if self.enable_viewer:
                try:
                    self.viewer_handle = viewer.launch_passive(self.model, self.data)
                    print(f"MuJoCo simulation initialized with viewer: {self.model_path}")
                except Exception as viewer_error:
                    print(f"Warning: Failed to create viewer: {viewer_error}")
                    print("Continuing without viewer...")
                    self.enable_viewer = False
            else:
                print(f"MuJoCo simulation initialized (headless): {self.model_path}")

        except Exception as e:
            print(f"Failed to initialize MuJoCo simulation: {e}")
            print(f"Error type: {type(e).__name__}")
            import traceback
            traceback.print_exc()
            raise

    def start_simulation(self):
        """Start the simulation loop"""
        if self.is_running:
            return

        self.is_running = True
        self.simulation_thread = threading.Thread(
            target=self._simulation_loop, daemon=True
        )
        self.simulation_thread.start()
        print("MuJoCo simulation started")

    def stop_simulation(self):
        """Stop the simulation loop"""
        self.is_running = False
        if self.simulation_thread and self.simulation_thread.is_alive():
            self.simulation_thread.join(timeout=5.0)
        print("MuJoCo simulation stopped")

    def _simulation_loop(self):
        """Main simulation loop"""
        while self.is_running:
            with self.lock:
                if self.data and self.model:
                    # Apply motor commands
                    self.data.ctrl[: len(self.motor_commands)] = self.motor_commands

                    # Step simulation
                    mj_step(self.model, self.data)

                    # Update position (example: get end-effector position)
                    if hasattr(self.data, "qpos") and len(self.data.qpos) >= 3:
                        self.current_position = self.data.qpos[:3].tolist()

                    # Sync with viewer
                    if self.viewer_handle:
                        self.viewer_handle.sync()

            # time.sleep(0.01)  # 100 Hz simulation

    def send_motor_command(self, command: MotorCommand) -> bool:
        """Send motor command to simulation"""
        try:
            with self.lock:
                # Check if forces actually changed
                old_forces = self.motor_commands.copy()
                self.motor_commands[0] = command.motor_1_force
                self.motor_commands[1] = command.motor_2_force
                self.motor_commands[2] = command.motor_3_force
                
                # Only print if forces changed
                if old_forces != self.motor_commands:
                    print(f"🎛️  Motor update: {command.to_dict()}")

            return True
        except Exception as e:
            print(f"Failed to send motor command: {e}")
            return False

    def get_status(self) -> RobotStatus:
        """Get current robot status"""
        with self.lock:
            return RobotStatus(
                mode=RobotMode.SIMULATION,
                is_connected=self.model is not None and self.data is not None,
                current_position=self.current_position.copy(),
                motor_forces=self.motor_commands.copy(),
            )

    def emergency_stop(self) -> bool:
        """Emergency stop - zero all forces"""
        try:
            with self.lock:
                self.motor_commands = [0.0, 0.0, 0.0]
            print("Emergency stop executed in simulation")
            return True
        except Exception as e:
            print(f"Emergency stop failed: {e}")
            return False

    def __del__(self):
        """Cleanup when object is destroyed"""
        self.stop_simulation()


def get_realtime_key():
    """Get a single keypress without waiting for Enter"""
    try:
        import msvcrt
        if msvcrt.kbhit():
            key = msvcrt.getch().decode('utf-8').lower()
            return key
        return None
    except ImportError:
        try:
            import keyboard
            # Check for various key presses
            key_mappings = {
                'q': 'q', 'a': 'a', 'w': 'w', 's': 's', 'e': 'e', 'd': 'd',
                'r': 'r', 'space': ' ', 't': 't', 'h': 'h', 'x': 'x',
                'c': 'c', 'p': 'p', 'f': 'f', 'i': 'i',
                '1': '1', '2': '2', '3': '3', 'esc': 'esc'
            }
            
            for kb_key, return_key in key_mappings.items():
                if keyboard.is_pressed(kb_key):
                    return return_key
            return None
        except ImportError:
            # Fallback - still use input but warn user
            return "fallback"


def print_controls():
    """Print control instructions"""
    print("\n" + "=" * 60)
    print("🎮 MuJoCo Robot Keyboard Control")
    print("=" * 60)
    print("🔧 Motor Controls:")
    print("  Q/A: Motor 1 +/- force    |  W/S: Motor 2 +/- force")
    print("  E/D: Motor 3 +/- force    |  1/2/3: Select motor focus")
    print("🎛️  Control Commands:")
    print("  R: Reset all motors       |  SPACE: Emergency stop")
    print("  T: Toggle status display  |  I: Show detailed info")
    print("  P: Print current position |  F: Print motor forces")
    print("  C: Connection status      |  H: Show this help")
    print("🚪 Exit Commands:")
    print("  X/ESC: Exit program       |  Ctrl+C: Force quit")
    print("=" * 60)


def status_display_thread(
    controller: MuJoCoController,
    stop_event: threading.Event,
    show_status: threading.Event,
):
    """Thread function to display robot status only when changes occur"""
    last_status = None
    last_forces = None
    last_position = None
    last_connection = None
    last_running = None
    
    print("📊 Status monitor started - will show updates only when values change")

    while not stop_event.is_set():
        if show_status.is_set():
            try:
                status = controller.get_status()
                current_time = time.strftime("%H:%M:%S")
                
                # Check if anything significant changed
                forces_changed = last_forces != status.motor_forces
                position_changed = (last_position is None or 
                                  any(abs(status.current_position[i] - last_position[i]) > 0.001 
                                      for i in range(len(status.current_position))))
                connection_changed = last_connection != status.is_connected
                running_changed = last_running != controller.is_running
                
                # Only print if something changed
                if (forces_changed or position_changed or connection_changed or 
                    running_changed or last_status is None):
                    
                    print(f"\n🔄 Status Update [{current_time}]:")
                    
                    # Connection status (only if changed)
                    if connection_changed or last_status is None:
                        connection_icon = "🟢" if status.is_connected else "🔴"
                        conn_text = "ACTIVE" if status.is_connected else "DISCONNECTED"
                        print(f"📡 Connection: {connection_icon} {conn_text}")
                    
                    # Running status (only if changed)
                    if running_changed or last_status is None:
                        running_text = "RUNNING" if controller.is_running else "STOPPED"
                        print(f"🏃 Simulation: {running_text}")
                    
                    # Position (only if changed significantly)
                    if position_changed or last_status is None:
                        pos = status.current_position
                        print(f"📍 Position: [{pos[0]:8.4f}, {pos[1]:8.4f}, {pos[2]:8.4f}]")
                    
                    # Motor forces (only if changed)
                    if forces_changed or last_status is None:
                        print("⚡ Motor Forces:")
                        for i, force in enumerate(status.motor_forces):
                            bar_length = int(abs(force) * 2)  # Scale for visualization
                            bar = "█" * min(bar_length, 20)
                            direction = "→" if force >= 0 else "←"
                            print(f"   Motor {i+1}: {force:6.2f} N {direction} {bar}")
                    
                    print("-" * 50)
                
                # Update last values
                last_status = status
                last_forces = status.motor_forces.copy()
                last_position = status.current_position.copy()
                last_connection = status.is_connected
                last_running = controller.is_running

            except Exception as e:
                print(f"❌ Error in status display: {e}")

        time.sleep(0.01)  # Check more frequently but only print when needed


def print_detailed_info(controller: MuJoCoController):
    """Print detailed controller and simulation information"""
    try:
        status = controller.get_status()
        print("\n" + "=" * 50)
        print("🔍 DETAILED SYSTEM INFORMATION")
        print("=" * 50)

        # Controller state
        print("🎛️  Controller State:")
        print(f"   Running: {controller.is_running}")
        print(f"   Viewer Enabled: {controller.enable_viewer}")
        print(
            f"   Thread Active: {controller.simulation_thread and controller.simulation_thread.is_alive()}"
        )

        # Model information
        if controller.model:
            print("\n📋 Model Information:")
            print(f"   Path: {controller.model_path}")
            print(f"   Bodies: {controller.model.nbody}")
            print(f"   Joints: {controller.model.njnt}")
            print(f"   DOF: {controller.model.nq}")
            print(f"   Actuators: {controller.model.nu}")
            print(f"   Sensors: {controller.model.nsensor}")

        # Data information
        if controller.data:
            print("\n📊 Simulation Data:")
            print(f"   Time: {controller.data.time:.4f} s")
            print(f"   Timestep: {controller.model.opt.timestep:.6f} s")
            print(f"   Position dim: {len(controller.data.qpos)}")
            print(f"   Velocity dim: {len(controller.data.qvel)}")
            print(f"   Control dim: {len(controller.data.ctrl)}")

        # Status information
        print("\n📈 Current Status:")
        print(f"   Mode: {status.mode.value}")
        print(f"   Connected: {status.is_connected}")
        print(f"   Position: {status.current_position}")
        print(f"   Motor Forces: {status.motor_forces}")

        print("=" * 50)

    except Exception as e:
        print(f"❌ Error getting detailed info: {e}")


def main():
    """Main function for keyboard control"""
    # Default model path - update to the correct location
    model_path = "e:\\桌面\\SoftRob\\Mujoco_0610\\robot\\model\\spiral_3D.xml"  # Correct model path

    # Initialize variables that will be used in finally block
    controller = None
    stop_event = None
    show_status_event = None

    # Check if model file exists
    if not os.path.exists(model_path):
        print(f"❌ Model file not found at: {model_path}")
        print("📝 Trying alternative paths...")

        # Try to find alternative model files - include more potential locations
        possible_paths = [
            "model\\spiral_3D.xml",
            "..\\model\\spiral_3D.xml",
            "model/spiral_3D.xml",
            "../model/spiral_3D.xml",
            "robot\\model\\spiral_3D.xml",
            "e:\\桌面\\SoftRob\\Mujoco_0610\\robot\\model\\spiral_3D.xml",
            "models\\soft_robot.xml",
            "robot.xml",
            "model.xml",
        ]

        for path in possible_paths:
            if os.path.exists(path):
                print(f"✅ Found model at: {path}")
                model_path = path
                break
        else:
            # Ask user for direct input as last resort
            print("❌ No model files found automatically.")
            user_path = input(
                "Please enter the full path to the model file (or press Enter to exit): "
            )
            if not user_path.strip():
                print("Exiting due to missing model file.")
                return
            if os.path.exists(user_path):
                model_path = user_path
            else:
                print(f"❌ Entered path not found: {user_path}")
                print("Exiting due to missing model file.")
                return

    try:
        print(f"🚀 Initializing MuJoCo controller with model: {model_path}")
        # Initialize controller
        controller = MuJoCoController(model_path, enable_viewer=True)

        # Verify initialization
        initial_status = controller.get_status()
        if not initial_status.is_connected:
            print("❌ Failed to connect to simulation")
            print(f"Debug info - Model: {controller.model is not None}, Data: {controller.data is not None}")
            return

        print("✅ Controller initialized successfully")
        controller.start_simulation()

        # Verify simulation started
        time.sleep(0.1)  # Give simulation time to start
        status = controller.get_status()
        if not controller.is_running:
            print("❌ Failed to start simulation")
            return

        print("✅ Simulation started successfully")
        print_controls()

        # Control parameters
        force_increment = 0.5
        max_force = 10.0
        min_force = -10.0
        selected_motor = 0  # Currently selected motor for focused control

        current_forces = [0.0, 0.0, 0.0]

        # Status display control
        stop_event = threading.Event()
        show_status_event = threading.Event()
        show_status_event.set()  # Start with status display enabled

        # Start status display thread
        status_thread = threading.Thread(
            target=status_display_thread,
            args=(controller, stop_event, show_status_event),
            daemon=True,
        )
        status_thread.start()

        # Check if we have real-time input capability
        realtime_input = True
        test_key = get_realtime_key()
        if test_key == "fallback":
            realtime_input = False
            print("⚠️  Real-time input not available, using fallback mode")
            print("💡 Install 'keyboard' package for better experience: pip install keyboard")

        print(f"🎯 Initial forces: {current_forces}")
        print(f"🎮 Status display: ON | Selected motor: {selected_motor + 1}")
        if realtime_input:
            print("� Real-time mode: Press keys directly (no Enter needed)")
        else:
            print("�📝 Fallback mode: Type command + Enter")
        print("📝 Ready for commands...")

        # Command history for debugging
        command_history = []
        last_key_time = 0
        key_repeat_delay = 0.1  # Prevent key repeat spam

        # Main control loop
        while True:
            try:
                # Check simulation health
                if not controller.is_running:
                    print("⚠️  Simulation stopped unexpectedly. Attempting restart...")
                    controller.start_simulation()
                    time.sleep(0.5)
                    if not controller.is_running:
                        print("❌ Failed to restart simulation")
                        break

                # Get keyboard input
                key = None
                current_time = time.time()
                
                if realtime_input:
                    # Real-time input mode
                    key = get_realtime_key()
                    if key and (current_time - last_key_time) > key_repeat_delay:
                        last_key_time = current_time
                    else:
                        key = None  # Ignore key repeat
                else:
                    # Fallback input mode
                    try:
                        key = input().strip().lower()
                    except (EOFError, KeyboardInterrupt):
                        break

                # Log command
                if key:
                    command_history.append((time.time(), key))
                    if len(command_history) > 100:  # Keep last 100 commands
                        command_history.pop(0)

                # Process commands (rest of the existing command processing code...)
                if key == "h":
                    print_controls()
                    continue
                elif key == "x" or key == "esc":
                    print("👋 Exiting...")
                    break
                elif key == "i":
                    print_detailed_info(controller)
                    continue
                elif key == "c":
                    # Connection status check
                    status = controller.get_status()
                    print(
                        f"📡 Connection: {'✅ CONNECTED' if status.is_connected else '❌ DISCONNECTED'}"
                    )
                    print(
                        f"🏃 Running: {'✅ YES' if controller.is_running else '❌ NO'}"
                    )
                    continue
                elif key == "p":
                    # Print current position
                    status = controller.get_status()
                    print(f"📍 Current position: {status.current_position}")
                    continue
                elif key == "f":
                    # Print motor forces
                    status = controller.get_status()
                    print(f"⚡ Motor forces: {status.motor_forces}")
                    continue
                elif key == "t":
                    # Toggle status display
                    if show_status_event.is_set():
                        show_status_event.clear()
                        print("📺 Status display: OFF")
                    else:
                        show_status_event.set()
                        print("📺 Status display: ON")
                    continue
                elif key in ["1", "2", "3"]:
                    # Select motor focus
                    selected_motor = int(key) - 1
                    print(f"🎯 Selected motor: {selected_motor + 1}")
                    continue
                elif key == "r":
                    # Reset all forces
                    current_forces = [0.0, 0.0, 0.0]
                    print("🔄 All forces reset to zero")
                elif key == " " or key == "space":
                    # Emergency stop
                    success = controller.emergency_stop()
                    current_forces = [0.0, 0.0, 0.0]
                    if success:
                        print("🚨 Emergency stop activated!")
                    else:
                        print("❌ Emergency stop failed!")
                    continue

                # Motor controls
                elif key == "q":
                    current_forces[0] = min(
                        max_force, current_forces[0] + force_increment
                    )
                    print(f"⬆️  Motor 1 force: {current_forces[0]:.2f} N")
                elif key == "a":
                    current_forces[0] = max(
                        min_force, current_forces[0] - force_increment
                    )
                    print(f"⬇️  Motor 1 force: {current_forces[0]:.2f} N")
                elif key == "w":
                    current_forces[1] = min(
                        max_force, current_forces[1] + force_increment
                    )
                    print(f"⬆️  Motor 2 force: {current_forces[1]:.2f} N")
                elif key == "s":
                    current_forces[1] = max(
                        min_force, current_forces[1] - force_increment
                    )
                    print(f"⬇️  Motor 2 force: {current_forces[1]:.2f} N")
                elif key == "e":
                    current_forces[2] = min(
                        max_force, current_forces[2] + force_increment
                    )
                    print(f"⬆️  Motor 3 force: {current_forces[2]:.2f} N")
                elif key == "d":
                    current_forces[2] = max(
                        min_force, current_forces[2] - force_increment
                    )
                    print(f"⬇️  Motor 3 force: {current_forces[2]:.2f} N")
                else:
                    if key:  # Only show message for non-empty inputs
                        print(f"❓ Unknown command: '{key}'. Press 'h' for help.")
                    continue

                # Send motor command with error checking
                try:
                    command = MotorCommand(
                        motor_1_force=current_forces[0],
                        motor_2_force=current_forces[1],
                        motor_3_force=current_forces[2],
                    )
                    success = controller.send_motor_command(command)

                    if success:
                        # Get updated status to confirm command was applied
                        status = controller.get_status()
                        if abs(status.motor_forces[0] - current_forces[0]) > 0.01:
                            print(
                                "⚠️  Warning: Motor command may not have been applied correctly"
                            )
                    else:
                        print("❌ Failed to send motor command!")

                except Exception as e:
                    print(f"❌ Error creating motor command: {e}")

                # Small delay to prevent excessive CPU usage in real-time mode
                if realtime_input:
                    time.sleep(0.02)  # 50 Hz polling rate

            except KeyboardInterrupt:
                print("\n🛑 Keyboard interrupt received. Exiting...")
                break
            except EOFError:
                print("\n📝 Input ended. Exiting...")
                break
            except Exception as e:
                print(f"❌ Error in control loop: {e}")
                continue

    except Exception as e:
        print(f"💥 Failed to initialize controller: {e}")
        return

    finally:
        # Comprehensive cleanup
        print("🧹 Cleaning up...")
        try:
            # Stop status display thread if it exists
            if stop_event is not None:
                stop_event.set()

            # Emergency stop before shutdown if controller exists
            if controller is not None:
                controller.emergency_stop()
                print("✅ Emergency stop executed")

                # Stop simulation
                controller.stop_simulation()
                print("✅ Simulation stopped")

            # Wait for threads to finish
            time.sleep(0.5)
            print("✅ Cleanup completed successfully")

        except Exception as e:
            print(f"⚠️  Error during cleanup: {e}")


if __name__ == "__main__":
    main()
