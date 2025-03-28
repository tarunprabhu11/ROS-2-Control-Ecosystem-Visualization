import rclpy
from rclpy.node import Node
from custom_msgs.msg import NodeConnection
from custom_msgs.srv import Trigger
import time

class NodeB(Node):
    def __init__(self):
        super().__init__('node_b')
        self.sub = self.create_subscription(NodeConnection, 'a_to_b', self.callback, 10)
        self.pub = self.create_publisher(NodeConnection, 'b_to_c', 10)
        self.client = self.create_client(Trigger, 'b_to_c_service')
        self.counter = 0
        
    def callback(self, msg):
        self.counter += 1
        self.get_logger().info(f"Received: {msg.metadata}")
        
        time.sleep(1)  
        
        new_msg = NodeConnection()
        new_msg.start = 'B'
        new_msg.end = 'C'
        new_msg.command_interface = False
        new_msg.state_interface = True
        new_msg.is_hardware = False
        new_msg.metadata = f'State from B to C (Seq: {self.counter})'
        self.pub.publish(new_msg)
        
        self.call_service()

    def call_service(self):
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting...')
        
        req = Trigger.Request()
        req.request = True
        future = self.client.call_async(req)
        future.add_done_callback(self.service_callback)

    def service_callback(self, future):
        try:
            response = future.result()
            self.get_logger().info(f"Service response: {response.message}")
        except Exception as e:
            self.get_logger().error(f"Service call failed: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = NodeB()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
