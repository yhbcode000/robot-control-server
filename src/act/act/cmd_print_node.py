import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class KeyboardSubscriber(Node):

    def __init__(self):
        super().__init__('keyboard_subscriber')
        self.subscription = self.create_subscription(
            String,
            'keyboard_input',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info('Received: "%s"' % msg.data)

def main(args=None):
    rclpy.init(args=args)
    keyboard_subscriber = KeyboardSubscriber()
    rclpy.spin(keyboard_subscriber)
    keyboard_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()