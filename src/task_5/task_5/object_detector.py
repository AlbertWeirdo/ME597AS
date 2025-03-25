import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from vision_msgs.msg import BoundingBox2D



class ObjectDetector(Node):
    def __init__(self):
        super().__init__('object_detector')
        self.subscriptions = self.create_subscription(Image,'/video_data',self.objectDetect, 10)
        self.publishers = self.create_publisher(BoundingBox2D, '/bbox,', 10)
    
    def objectDetect(self):
        
        
        
        
def main():
    rclpy.init
    od = ObjectDetector()
    rclpy.spin()
    od.destroy_node
    rclpy.shutdown
    
if __name__ == '__main__':
    main()