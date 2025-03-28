import rclpy
from rclpy.node import Node
from custom_msgs.srv import Trigger
from custom_msgs.msg import NodeConnection
import time

class NodeC(Node):
    def __init__(self):
        super().__init__('node_c')
        self.srv = self.create_service(Trigger, 'b_to_c_service', self.service_callback)
        self.pub = self.create_publisher(NodeConnection, 'c_to_d', 10)
        self.counter = 0
        
    def service_callback(self, request, response):
        self.counter += 1
        self.get_logger().info("Service called")
        
        time.sleep(1)  
        
        # Forward to D
        msg = NodeConnection()
        msg.start = 'C'
        msg.end = 'D'
        msg.command_interface = True
        msg.state_interface = False
        msg.is_hardware = True  
        msg.metadata = f'Command from C to hardware D (Seq: {self.counter})'
        self.pub.publish(msg)
        
        response.success = True
        response.message = f"Processed by C (Seq: {self.counter})"
        return response

def main(args=None):
    rclpy.init(args=args)
    node = NodeC()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
