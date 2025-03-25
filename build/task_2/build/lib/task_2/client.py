import sys

from task_2_interfaces.srv import JointState
import rclpy
from rclpy.node import Node

class Client(Node):
    def __init__(self):
        super().__init__('client')
        self.cli=self.create_client(JointState,'joint_service')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req=JointState.Request()

    def send_request(self):
        self.req.x=float(sys.argv[1])
        self.req.y=float(sys.argv[2])
        self.req.z=float(sys.argv[3])
        self.future=self.cli.call_async(self.req)
    
def main():
    rclpy.init()

    client=Client()
    client.send_request()

    while rclpy.ok():
        rclpy.spin_once(client)
        if client.future.done():
            try:
                response=client.future.result()
            except Exception as e:
                client.get_logger().info('Service call failed %r' % (e,))
            else:
                client.get_logger().info('Answer for %f + %f + %f >=0 is %s' % (client.req.x,client.req.y,client.req.z,response.valid))
            break
    
    client.destroy_node()
    rclpy.shutdown()


if __name__=='main':
    main()