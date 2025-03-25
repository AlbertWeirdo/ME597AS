import rclpy
from rclpy.node import Node

from task_2_interfaces.msg import JointData

class Pub(Node):
    def __init__(self):
        super().__init__('talker')
        self.publisher_=self.create_publisher(JointData,'joint_topic',10)
        timer_period=0.5
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = JointData()
        msg.vel = 0.00 + self.i
        self.get_logger().info('Publishing: velocity is %f ' %msg.vel)
        self.publisher_.publish(msg)
        self.i += 1

def main():
    rclpy.init()
    pub=Pub()
    # keep the node running
    rclpy.spin(pub)
    pub.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()