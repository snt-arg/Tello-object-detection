#To handle ROS node
import rclpy
from rclpy.node import Node

#ROS image message
from sensor_msgs.msg import Image 

#For image manipulation (OpenCV)
import cv2
import numpy as np

#To convert cv2 images to ROS Image messages
from cv_bridge import CvBridge

#for object detection
from ultralytics import YOLO



#load the object detection model
model = YOLO('yolov8n.pt') 

#Filtering our classes of interest
classes = model.names
classes_needed = ["cell phone"]
classes_ID = [k for k,v in classes.items() if v in classes_needed]

#Detection threshold probability
minimum_prob = 0.4  


class ImageSubscriberDetectedPublisher(Node):

    def __init__(self,name):
        #Creating the Node
        super().__init__(name)
        
        #subscriber
        self.sub = self.create_subscription(Image,'image_raw',self.listener_callback,10)
        
        #publisher
        self.publisher_ = self.create_publisher(Image, 'image_detected', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)
        
        self.cv_bridge = CvBridge()

        #output video initialization
        self.fourcc = cv2.VideoWriter_fourcc(*'MJPG')
        self.video = cv2.VideoWriter('~/output.avi',self.fourcc,20.0,(640,480))

        #Variable to read each frame
        self.image=None

    
    def detection(self,frame):
        """Function to perform object detection on frames"""
        results = model.track(frame, persist=True, classes=classes_ID, conf=minimum_prob)
        frame_ = results[0].plot()
        return frame_
        
    def listener_callback(self, img):
        """Callback function for the subscriber node (to topic image_raw).
        For each image received, save in the log that an image has been received.
        Then convert that image into cv2 format, perform detection on that image, 
        and write the frame on the output video."""
        self.get_logger().info('I saw an image')
        image = self.cv_bridge.imgmsg_to_cv2(img,'rgb8')
        self.image = self.detection(image)
        self.video.write(np.array(self.image))
          
    def timer_callback(self):
        """callback funtion for the publisher node (to topis image_detected).
        The image on which object detection has been performed (self.image) is published on the topic 'image_raw'
        """
        self.publisher_.publish(self.cv_bridge.cv2_to_imgmsg(np.array(self.image), 'rgb8'))
        
    
        
def main(args=None):
    #Intialization ROS communication 
    rclpy.init(args=args)
    
    image_subscriber = ImageSubscriberDetectedPublisher('camera_subscriber_pub_detected')

    #execute the callback function until the global executor is shutdown
    rclpy.spin(image_subscriber)

    #release the video output writer.
    image_subscriber.video.release()
    image_subscriber.destroy_node()
    
    rclpy.shutdown()


if __name__ == '__main__':
    main()
