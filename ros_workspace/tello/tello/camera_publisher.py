import rclpy
import cv2
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

class ImagePublisher(Node):
    def __init__(self,name):
        super().__init__(name)
        self.publisher_ = self.create_publisher(Image, 'image_raw', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)
        
        self.cap = cv2.VideoCapture(0)
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'))
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 800)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 800)
        self.cap.set(cv2.CAP_PROP_FPS, 130)   
        
        self.i = 0
          
        self.cv_bridge = CvBridge()
        
    def timer_callback(self):
        ret,frame = self.cap.read()
        if ret:
            self.publisher_.publish(self.cv_bridge.cv2_to_imgmsg(frame,'bgr8'))
            
        self.get_logger().info('Publishing frame %d'%self.i)
        self.i +=1

def main(args=None):
    rclpy.init(args=args)
    image_publisher = ImagePublisher('camera_1_pub')
    rclpy.spin(image_publisher)
    image_publisher.destroy_node()
    rclpy.shutdown()        
