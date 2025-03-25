import rclpy
from rclpy.node import Node

from task_2_interfaces.msg import JointData

class Sub(Node):
    def __init__(self):
        super().__init__('sub')
        self.subscription=self.create_subscription(
            JointData,
            'joint_topic',
            self.listener_callback,
            10,
        )
    
    def listener_callback(self,msg):
        self.get_logger().info('I heard that the robot is moving at a velocity of %f' % msg.vel)

def main():
    rclpy.init()
    sub=Sub()

    # keep the node running
    rclpy.spin(sub)

    sub.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()

