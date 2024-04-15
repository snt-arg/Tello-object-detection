import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge
##for object detection
from ultralytics import YOLO
#import time


model = YOLO('yolov8n.pt') 
classes = model.names
minimum_prob = 0.4  
classes_needed = ["person"]
classes_ID = [k for k,v in classes.items() if v in classes_needed]

class ImageSubscriberDetectedPublisher(Node):

    def __init__(self,name):
        super().__init__(name)
        #subscriber
        self.sub = self.create_subscription(Image,'image_raw',self.listener_callback,10)
        
        #publisher
        self.publisher_ = self.create_publisher(Image, 'image_detected', 10)
        self.timer = self.create_timer(0.1, self.listener_callback)
        
        self.cv_bridge = CvBridge()
        #self.subscription  # prevent unused variable warning
        
    def detection(self,frame):
        """Function to perform object detection"""
        results = model.track(frame, persist=True, classes=classes_ID, conf=minimum_prob)
        frame_ = results[0].plot()
        frame = frame_
        
    def listener_callback(self, img):
        #self.get_logger().info('I saw an image')
        image = self.cv_bridge.imgmsg_to_cv2(img,'bgr8')
        self.detection(image)
        self.publisher_.publish(self.cv_bridge.cv2_to_imgmsg(image, 'bgr8'))
        #cv2.imshow("webcam",image)
        #cv2.waitKey(10)
        
    
        
 
    
def main(args=None):
    rclpy.init(args=args)
    image_subscriber = ImageSubscriberDetectedPublisher('camera_1_subscriber')
    rclpy.spin(image_subscriber)
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    image_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
