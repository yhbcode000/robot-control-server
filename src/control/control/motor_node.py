import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MotorNode(Node):
    def __init__(self):
        super().__init__('motor_node')
        self.get_logger().info('Motor node has been started.')
        self.create_subscription(String, 'perception/keyboard_input', self.keyboard_callback, 10)

    def keyboard_callback(self, msg):
        if msg.data == 'w':
            print('+')
        elif msg.data == 's':
            print('-')

def main(args=None):
    rclpy.init(args=args)
    motor_node = MotorNode()
    rclpy.spin(motor_node)
    motor_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()