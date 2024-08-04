#To handle ROS node
import rclpy
from rclpy.node import Node

#ROS image message
from sensor_msgs.msg import Image

from all_bounding_boxes_msg.msg import AllBoundingBoxes, Box

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

class DetectAll(PluginBase):
    #Detection threshold probability
    minimum_prob = 0.4  

    #Topic names
    image_raw_topic = "/camera/image_raw"
    all_detected_topic = "/all_detected"
    bounding_boxes_topic = "/all_bounding_boxes"
    
    def __init__(self,name):
        #Creating the Node
        super().__init__(name)
        
        #subscribers
        self.sub_raw = self.create_subscription(Image,self.image_raw_topic, self.listener_callback,10)
        #self.sub_raw = self.create_subscription(Image,image_raw, self.listener_callback,1)

        
        #publishers
        self.publisher_all_detected = self.create_publisher(Image,self.all_detected_topic,10)
        self.publisher_bounding_boxes = self.create_publisher(AllBoundingBoxes,self.bounding_boxes_topic,10)
        

        self.cv_bridge = CvBridge()

        #Variable to contain the frame coming directly from the drone
        self.image_raw = None

        #Variable to read each frame. It contains all persons detected
        self.image_all_detected = None

        #Variable containing all bounding boxes coordinates for a single frame
        self.boxes = AllBoundingBoxes()




########################### Subscriber ###########################################################################################   
    def listener_callback(self, img):
        """Callback function for the subscriber node (to topic /camera/image_raw).
        For each image received, save in the log that an image has been received.
        Then convert that image into cv2 format, perform tracking on that image"""
        
        #print("Received nothing")
        self.get_logger().info('I saw an image')
        self.image_raw = self.cv_bridge.imgmsg_to_cv2(img,'rgb8')
        self.image_all_detected = self.detection(self.image_raw)
        

        
    def detection(self,frame):
        """Function to perform person object detection on a single frame"""
        results = model.track(frame, persist=True, classes=classes_ID, conf=self.minimum_prob)
        #prepare message 
        box_msg = Box()
        self.boxes = AllBoundingBoxes()
        for box in results[0].boxes.xyxyn.tolist():
            box_msg.top_left.x = box[0]
            box_msg.top_left.y = box[1]
            box_msg.bottom_right.x = box[2]
            box_msg.bottom_right.y = box[3]
            self.boxes.bounding_boxes.append(box_msg)

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
            self.get_logger().info('No image seen')
            
        else:
            #print(self.image_all_detected)
            self.publisher_all_detected.publish(self.cv_bridge.cv2_to_imgmsg(self.image_all_detected, 'rgb8')) 
            self.publisher_bounding_boxes.publish(self.boxes)

    def tick(self) -> NodeState:
        """This method is a mandatory for PluginBase node. It defines what we want our node to do.
        It gets called 20 times a second if state=RUNNING
        """
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
