import asyncio
import time
import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from interfaces.action import Motor
from rclpy.action import CancelResponse, GoalResponse

class MotorActionServer(Node):

    def __init__(self):
        super().__init__('motor_action_server')
        self.get_logger().info('Initializing MotorActionServer...')
        self._action_server = ActionServer(
            self,
            Motor,
            'motor',
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback)
        self.get_logger().info('MotorActionServer initialized successfully.')

    def goal_callback(self, _):
        self.get_logger().info('Received goal request')
        return GoalResponse.ACCEPT

    def cancel_callback(self, _):
        self.get_logger().info('Received cancel request')
        return CancelResponse.ACCEPT

    async def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')
        feedback_msg = Motor.Feedback()
        feedback_msg.current_position = 0.0

        # Simulate motor force application and position update
        motor_1_force = goal_handle.request.motor_1_force
        motor_2_force = goal_handle.request.motor_2_force
        motor_3_force = goal_handle.request.motor_3_force

        self.get_logger().info(f'Motor forces received: motor_1_force={motor_1_force}, motor_2_force={motor_2_force}, motor_3_force={motor_3_force}')

        # TODO impliment motor force application and position update

        goal_handle.succeed()

        result = Motor.Result()
        result.success = True
        self.get_logger().info('Goal execution succeeded.')
        return result

def main(args=None):
    rclpy.init(args=args)
    motor_action_server = MotorActionServer()
    rclpy.spin(motor_action_server)

if __name__ == '__main__':
    main()