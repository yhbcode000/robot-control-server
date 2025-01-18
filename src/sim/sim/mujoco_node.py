import rclpy
from rclpy.node import Node
from interfaces.action import Motor
import threading
import numpy as np
from dm_control import mujoco
from mujoco import MjModel, MjData, mj_step, viewer
import os
from ament_index_python.packages import get_package_share_directory

class MotorController(Node):
    def __init__(self, model_path):
        super().__init__('motor_controller')
        self.motor_commands = [0.0, 0.0, 0.0]
        
        # Subscribe to goal_topic (Motor.Goal)
        self.goal_subscription = self.create_subscription(
            Motor.Goal,
            'goal_topic',
            self.goal_callback,
            10
        )

        self.model = MjModel.from_xml_path(model_path)
        self.data = MjData(self.model)
        self.viewer_handle = viewer.launch_passive(self.model, self.data)

        self.simulation_running = True
        self.simulation_thread = threading.Thread(target=self.simulation_loop)
        self.simulation_thread.start()

    def goal_callback(self, msg):
        self.motor_commands[0] = msg.motor_1_force
        self.motor_commands[1] = msg.motor_2_force
        self.motor_commands[2] = msg.motor_3_force

    def simulation_loop(self):
        while self.simulation_running:
            self.data.ctrl[:] = self.motor_commands
            mj_step(self.model, self.data)
            self.viewer_handle.sync()

    def destroy_node(self):
        self.simulation_running = False
        if self.simulation_thread.is_alive():
            self.simulation_thread.join()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    resource_path = os.path.join(get_package_share_directory('sim'), 'resource')
    model_path = os.path.join(resource_path, 'physics_model', 'spiral_3D.xml')
    node = MotorController(model_path)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
