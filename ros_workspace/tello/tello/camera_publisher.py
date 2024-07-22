#To handle ROS node
import rclpy
from rclpy.node import Node

#ROS image message
from sensor_msgs.msg import Image

#To handle images
#import cv2

#To connect to the drone
from djitellopy import Tello, tello

#To convert cv2 images to ROS Image messages
from cv_bridge import CvBridge



class ImagePublisher(Node):
    def __init__(self,name):
        #Creation of the node
        super().__init__(name)

        #Creation of the publisher
        self.publisher_ = self.create_publisher(Image, 'image_raw', 10)

        #Creation of a timer to execute the callback function
        self.timer = self.create_timer(0.1, self.timer_callback)

        #Connection to the drone
        self.drone = tello.Tello()
        self.drone.connect()
        self.drone.streamon()

        #Variable to receive a frame from the drone
        self.cap = self.drone.get_frame_read()  

        #counter for frames
        self.i = 0

        #image Converter, from OpenCv format to ROS image message format 
        self.cv_bridge = CvBridge()
        
    def timer_callback(self):
        """Callback function to publish a frame received from the drone. It notifies that the frame has been published."""
        frame = self.cap.frame
        self.publisher_.publish(self.cv_bridge.cv2_to_imgmsg(frame,'rgb8'))  
        self.get_logger().info('Publishing frame %d'%self.i)
        self.i +=1

def main(args=None):
    #Intialization ROS communication 
    rclpy.init(args=args)
    image_publisher = ImagePublisher('camera_1_pub')

    #execute the callback function until the global executor is shutdown
    rclpy.spin(image_publisher)

    #destroy the node. It is not mandatory, since the garbage collection can do it
    image_publisher.destroy_node()
    
    rclpy.shutdown()        
