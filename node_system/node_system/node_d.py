import rclpy
from rclpy.node import Node
from custom_msgs.msg import NodeConnection
import time

class NodeD(Node):
    def __init__(self):
        super().__init__('node_d')
        self.sub = self.create_subscription(NodeConnection, 'c_to_d', self.callback, 10)
        self.counter = 0
        
    def callback(self, msg):
        self.counter += 1
        self.get_logger().info(f"Hardware received: {msg.metadata}")
        
        time.sleep(1)  
        self.get_logger().info(f"Hardware processing complete (Seq: {self.counter})")

def main(args=None):
    rclpy.init(args=args)
    node = NodeD()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
