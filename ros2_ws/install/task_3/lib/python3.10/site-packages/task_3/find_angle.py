import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

class find_angle(Node):
    def __init__(self):
        super().__init__('find_angle')
        self.subscription=self.create_subscription(LaserScan, '/robot/scan', self.fetch_angle, 10)

    def fetch_distance(self,msg):
        self.get_logger.info(f'current angle: {msg.angle_min}')


def main():
    rclpy.init()
    find_angle=find_angle()

    rclpy.spin(find_angle)
    find_angle.destroy_node()
    rclpy.shutdown()

if __name__=="main":
    main()
