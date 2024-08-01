#To handle ROS node
import rclpy
from rclpy.node import Node

#ROS image message
from sensor_msgs.msg import Image

#To handle images
#import cv2

#To connect to the drone
#from djitellopy import Tello, tello

#To convert cv2 images to ROS Image messages
from cv_bridge import CvBridge

#To handle images
import numpy as np

#Node base to be able to integrate our project to the Tello_ws
from plugin_server_base.plugin_base import PluginBase, NodeState

#YOLOv8 object detection framework
from ultralytics import YOLO

#load the object detection model
model = YOLO('yolov8n.pt') 

#Filtering our classes of interest
classes = model.names
classes_needed = ["person"]
classes_ID = [k for k,v in classes.items() if v in classes_needed]

#Detection threshold probability
minimum_prob = 0.4  

#Topic names
image_raw = "/camera/image_raw"
all_detected_topic = "/all_detected"

class DetectAll(PluginBase):

    
    def __init__(self,name):
        #Creating the Node
        super().__init__(name)
        
        #subscribers
        self.sub_raw = self.create_subscription(Image,image_raw, self.listener_callback,10)
        #self.sub_raw = self.create_subscription(Image,image_raw, self.listener_callback,1)

        
        #publishers
        self.publisher_all_detected = self.create_publisher(Image,all_detected_topic,10)
        #self.publisher_all_detected = self.create_publisher(Image,all_detected_topic,1)
        #self.timer_ = self.create_timer(0.1, self.all_detected_callback)
        

        self.cv_bridge = CvBridge()

        #Variable to contain the frame coming directly from the drone
        self.image_raw = None

        #Variable to read each frame. It contains all persons detected
        self.image_all_detected = None


########################### Subscriber ###########################################################################################   
    def listener_callback(self, img):
        """Callback function for the subscriber node (to topic /camera/image_raw).
        For each image received, save in the log that an image has been received.
        Then convert that image into cv2 format, perform tracking on that image"""
        
        #print("Received nothing")
        self.get_logger().info('I saw an image')
        self.image_raw = self.cv_bridge.imgmsg_to_cv2(img,'rgb8')
        self.image_all_detected = self.detection(self.image_raw)
        print("image,", self.image_raw)

        
    def detection(self,frame):
        """Function to perform person object detection on a single frame"""
        results = model.track(frame, persist=True, classes=classes_ID, conf=minimum_prob)
        frame_ = results[0].plot()
        return frame_

######################## Publisher #####################################################################################  
    
        
    def all_detected_callback(self):
        """
        callback funtion for the publisher node (to topic /camera/image_detected).
        The image on which object detection has been performed (self.image_all_detected) is published on the topic '/all_detected'
        """
        #self.publisher_all_detected.publish(self.cv_bridge.cv2_to_imgmsg(np.array(self.image_all_detected), 'rgb8')) 
        if(self.image_all_detected is None):
            #self.publisher_all_detected.publish(self.cv_bridge.cv2_to_imgmsg(self.image_all_detected, 'rgb8')) 
            print(self.image_all_detected)
            
        else:
            #print(self.image_all_detected)
            self.publisher_all_detected.publish(self.cv_bridge.cv2_to_imgmsg(self.image_all_detected, 'rgb8')) 
        
    def tick(self) -> NodeState:
        """This method is a mandatory for PluginBase node. It defines what we want our node to do.
        It gets called 20 times a second if state=RUNNING
        """

        #self.get_logger().info("Tick")

        #self.get_logger().info("Evaluate")

        #self.get_logger().info("Command")
        #self.listener_callback()
        self.all_detected_callback()
        return NodeState.RUNNING

def main(args=None):
    #Intialization ROS communication 
    rclpy.init(args=args)
    detector = DetectAll('all_person_detector')

    #execute the callback function until the global executor is shutdown
    rclpy.spin(detector)

    #destroy the node. It is not mandatory, since the garbage collection can do it
    detector.destroy_node()
    
    rclpy.shutdown()        
