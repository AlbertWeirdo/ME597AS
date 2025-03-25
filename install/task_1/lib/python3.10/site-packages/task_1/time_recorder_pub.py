import rclpy
from rclpy.node import Node

from std_msgs.msg import Float64

class TimeRecorderPub(Node):
    def __init__(self):
        super().__init__('talker')
        # declare publish message type as String, set the topic name as 'my_first_topic', and set the queue size to 10 
        self.publisher_ = self.create_publisher(Float64, 'my_first_topic', 10)
        timer_period = 1.0  # seconds
        # create a timer with the period of 1.0 seconds and call the function show_activation_time
        self.timer = self.create_timer(timer_period, self.show_activation_time)
        self.i = 0.0

    def show_activation_time(self):
        msg = Float64()
        msg.data = float(self.i)
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "Node \'time_recorder\' has been activated for %d seconds"' % msg.data)
        self.i += 1.0

def main():
    rclpy.init()
    time_recorder_pub = TimeRecorderPub()
    
    # keep the node running
    rclpy.spin(time_recorder_pub)


if __name__ == '__main__':
    main()
