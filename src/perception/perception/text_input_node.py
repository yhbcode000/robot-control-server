#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class TextInputNode(Node):
    def __init__(self):
        super().__init__('text_input_node')
        self.pub = self.create_publisher(String, 'language_query', 10)
        self.get_logger().info('TextInputNode has been started.')

def main(args=None):
    rclpy.init(args=args)
    node = TextInputNode()
    try:
        while rclpy.ok():
            user_input = input("Enter text: ")
            msg = String()
            msg.data = user_input
            node.pub.publish(msg)
            node.get_logger().info(f'Published: "{user_input}"')
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard interrupt, shutting down.')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()