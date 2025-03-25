from task_2_interfaces.srv import JointState
import rclpy
from rclpy.node import Node

class Service(Node):
    def __init__(self):
        super().__init__('service')
        self.srv = self.create_service(JointState, 'joint_service',self.add_three_ints)

    def add_three_ints(self,request,response):
        response.valid =True
        self.get_logger().info('Incoming request\na: %f b: %f c: %f' % (request.x, request.y, request.z))
        
        if request.x+request.y+request.z<0:
            response.valid=False
        return response

def main():
    rclpy.init()
    service=Service()
    rclpy.spin(service)

    rclpy.shutdown()

if __name__=='main':
    main()

