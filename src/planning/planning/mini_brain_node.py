import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from std_msgs.msg import String
from interfaces.action import Motor

class MiniBrainNode(Node):
    def __init__(self):
        super().__init__('mini_brain_node')
        self.client = ActionClient(self, Motor, 'motor')
        self.subscription = self.create_subscription(String, 'cmd_topic', self.cmd_callback, 10)
        self.client.wait_for_server()
        self.get_logger().info('MiniBrainNode has been started.')

    def cmd_callback(self, msg):
        cmd = msg.data.strip().lower()
        self.get_logger().info(f'Received command: {cmd}')
        if cmd == 'w':
            goal = Motor.Goal()
            goal.motor_1_force = 1.0
            goal.motor_2_force = 1.0
            goal.motor_3_force = 1.0
            self.client.send_goal_async(goal, feedback_callback=self.feedback_callback).add_done_callback(self.goal_response_callback)
            self.get_logger().info('Sent goal to motors with force 1.')
        elif cmd == 's':
            goal = Motor.Goal()
            goal.motor_1_force = 0.0
            goal.motor_2_force = 0.0
            goal.motor_3_force = 0.0
            self.client.send_goal_async(
                goal, 
                feedback_callback=self.feedback_callback
            ).add_done_callback(self.goal_response_callback)
            self.get_logger().info('Stop all motor actions.')
        # TODO if we want a immediate stop, we have to updat teh motor action interface to include a status field that can be set to 'stop' and the motor action server should stop the motor immediately. The system will set to a new goal to stop the motor.

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(f'Received feedback: Current position: {feedback.current_position}')

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected.')
            return

        self.get_logger().info('Goal accepted.')
        goal_handle.get_result_async().add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        if result.success:
            self.get_logger().info(f'Goal succeeded!')
        else:
            self.get_logger().info('Goal failed.')

def main(args=None):
    rclpy.init(args=args)
    node = MiniBrainNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
