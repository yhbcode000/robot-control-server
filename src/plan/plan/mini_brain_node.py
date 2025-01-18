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
        # Publisher for the goal topic
        self.goal_pub = self.create_publisher(Motor.Goal, 'goal_topic', 10)
        self.client.wait_for_server()
        self.get_logger().info('MiniBrainNode started.')
        self.spinning = False
        self.spin_direction = None
        self.spin_stage = 0
        self.timer = self.create_timer(0.2, self.spin_timer_callback)

    def cmd_callback(self, msg):
        incoming_cmd = msg.data.strip()
        self.get_logger().info(f'Received: {incoming_cmd}')

        # TODO MPC-like logic here, might be a separate idea here.
        if incoming_cmd == 'D1':
            self.send_motor_goal(1.0, 0.0, 0.0, 'Increase dimension 1')
        elif incoming_cmd == 'D2':
            self.send_motor_goal(0.0, 1.0, 0.0, 'Increase dimension 2')
        elif incoming_cmd == 'D3':
            self.send_motor_goal(0.0, 0.0, 1.0, 'Increase dimension 3')
        elif incoming_cmd == 'X':
            self.send_motor_goal(0.0, 0.0, 0.0, 'Emergency stop')
        elif incoming_cmd == 'CS':
            self.start_spin('clockwise')
        elif incoming_cmd == 'AS':
            self.start_spin('anticlockwise')

    def start_spin(self, direction):
        self.get_logger().info(f'Starting {direction} spin...')
        self.spinning = True
        self.spin_direction = direction
        self.spin_stage = 0

    def spin_timer_callback(self):
        if not self.spinning:
            return
        goal = Motor.Goal()
        if self.spin_direction == 'clockwise':
            forces = [0.1, 0.0, 0.0] if self.spin_stage == 0 else [0.0, 0.1, 0.0] if self.spin_stage == 1 else [0.0, 0.0, 0.1]
        else:
            forces = [0.0, 0.0, 0.1] if self.spin_stage == 0 else [0.0, 0.1, 0.0] if self.spin_stage == 1 else [0.1, 0.0, 0.0]

        goal.motor_1_force, goal.motor_2_force, goal.motor_3_force = forces
        # Publish to goal_topic
        self.goal_pub.publish(goal)
        self.client.send_goal_async(goal)
        self.spin_stage = (self.spin_stage + 1) % 3

    def send_motor_goal(self, m1, m2, m3, label=''):
        goal = Motor.Goal()
        goal.motor_1_force = m1
        goal.motor_2_force = m2
        goal.motor_3_force = m3
        # Publish to goal_topic
        self.goal_pub.publish(goal)
        self.client.send_goal_async(goal, feedback_callback=self.feedback_callback)\
            .add_done_callback(self.goal_response_callback)
        self.get_logger().info(f'Sent {label} goal ({m1}, {m2}, {m3}).')
        self.spinning = False

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(f'Feedback: pos={feedback.current_position}')

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
            self.get_logger().info('Goal succeeded!')
        else:
            self.get_logger().info('Goal failed.')

def main(args=None):
    rclpy.init(args=args)
    node = MiniBrainNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
