import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge
##for object detection
from ultralytics import YOLO
#import time


class ImageSubscriber(Node):

    def __init__(self,name):
        super().__init__(name)
        #subscriber
        self.sub = self.create_subscription(Image,'image_detected',self.listener_callback,10)
        self.cv_bridge = CvBridge()
        #self.subscription  # prevent unused variable warning
                
    def listener_callback(self, img):
        self.get_logger().info('I saw an image')
        image = self.cv_bridge.imgmsg_to_cv2(img,'bgr8')
        cv2.imshow("webcam",image)
        cv2.waitKey(10)
        
    
           
def main(args=None):
    rclpy.init(args=args)
    image_subscriber = ImageSubscriber('detected_1_subscriber')
    rclpy.spin(image_subscriber)
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    image_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
