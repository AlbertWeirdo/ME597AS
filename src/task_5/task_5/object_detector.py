import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image



class ObjectDetector(Node):
    def __init__(self):
        super().__init__('object_detector')
        self.subscriptions = self.create_subscription( ,'video_data', ,10)
    
    
        
        
        
def main():
    rclpy.init
    od = ObjectDetector()
    rclpy.spin()
    od.destroy_node
    rclpy.shutdown
    
if __name__ == '__main__':
    main()