import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class CentralBrainNode(Node):
    def __init__(self):
        super().__init__('central_brain_node')
        self.cmd_pub = self.create_publisher(String, '/cmd_topic', 10)
        self.create_subscription(String, '/language_query', self.dummy_chatbot, 10)
        self.get_logger().info('CentralBrainNode has been started.')

    def dummy_chatbot(self, msg):
        text = msg.data.lower()
        self.get_logger().info(f'Received message: {text}')
        if "forward" in text:
            self.publish_cmd("W")
        elif "back" in text:
            self.publish_cmd("S")
        elif "left" in text:
            self.publish_cmd("A")
        elif "right" in text:
            self.publish_cmd("D")
        else:
            self.get_logger().warn(f'Unknown command: {text}')

    def publish_cmd(self, cmd):
        cmd_msg = String()
        cmd_msg.data = cmd
        self.cmd_pub.publish(cmd_msg)
        self.get_logger().info(f'Published command: {cmd}')

def main(args=None):
    rclpy.init(args=args)
    node = CentralBrainNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()