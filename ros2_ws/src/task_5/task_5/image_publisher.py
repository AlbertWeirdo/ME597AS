import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

class imagePublisher(Node):
    def __init__(self):
        super().__init__('image_publisher')
        self.publisher = self.create_publisher(Image ,'video_data', 10)
        
    # def raw
        
        
        
def main():
    rclpy.init()
    image = imagePublisher()
    rclpy.spin(image)
    image.destroy_node
    rclpy.shutdown
    
if __name__ == 'main':
    main()
    
    