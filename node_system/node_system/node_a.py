import rclpy
from rclpy.node import Node
from custom_msgs.msg import NodeConnection
import time

class NodeA(Node):
    def __init__(self):
        super().__init__('node_a')
        self.pub = self.create_publisher(NodeConnection, 'a_to_b', 10)
        # Increase timer interval to 5 seconds
        self.timer = self.create_timer(5.0, self.publish)
        self.counter = 0
        
    def publish(self):
        self.counter += 1
        msg = NodeConnection()
        msg.start = 'A'
        msg.end = 'B'
        msg.command_interface = True
        msg.state_interface = False
        msg.is_hardware = False
        msg.metadata = f'Command from A to B (Seq: {self.counter})'
        self.pub.publish(msg)
        self.get_logger().info(f"Published: {msg.metadata}")

def main(args=None):
    rclpy.init(args=args)
    node = NodeA()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
