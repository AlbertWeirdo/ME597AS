import rclpy
from rclpy.node import Node

from std_msgs.msg import Float64

class doubler(Node):
    def __init__(self):
        super().__init__('doubler_sub')
         # declare subscribe message type as Float64, set the topic name as 'my_first_topic', and set the queue size to 10 
        self.subscription = self.create_subscription(Float64, 'my_first_topic', self.double_message_quantity, 10)


    def double_message_quantity(self, msg):
        self.get_logger().info(
        f"Subscribing (original): 'I heard Node \"time_recorder\" has been activated for {msg.data} seconds' "
        f"Subscribing (doubled): 'I heard Node \"time_recorder\" has been activated for {msg.data * 2} seconds'"
        )


def main():
    rclpy.init()
    doubler_sub = doubler()

    # keep the node running
    rclpy.spin(doubler_sub)
        
if __name__ == '__main__':
    main()