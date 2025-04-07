#To handle ROS node
import rclpy


#ROS image message
from std_msgs.msg import Empty, String

#ROS image message
from sensor_msgs.msg import Image

#Node base to be able to integrate our project to the Tello_ws
from plugin_server_base.plugin_base import PluginBase, NodeState

image_raw = "/camera/image_raw"

from ultralytics import YOLO

#for gpu 
import torch

import json

from cv_bridge import CvBridge



#load the object detection model
model = YOLO('yolov8n-oiv7.pt') 

#use gpu if available, and if not, cpu
device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
model.to(device)

#Defining claases of interest
 
persons = ["person","boy","girl","woman","man"]
objects = ["hat","doll", "backpack", "headphones", "mobile phone", "telephone"]

classes_needed = persons + objects

#Getting the ids of our classes of interest
classes = model.names
classes_ID = [k for k,v in classes.items() if v.lower() in classes_needed] 


class Test1(PluginBase):

    image_raw_topic = "/camera/image_raw" #raw image frames from the drone' camera
    all_detected_topic = "/all_detected" # image frames of which all persons (and specified objects) are detected
    bounding_boxes_topic = "/all_bounding_boxes" # list of person bounding boxes
    tracking_info_pilot = "/tracking_info_pilot_person"
    tracking_info = "/tracking_info"
    minimum_prob = 0

    def __init__(self,name):
        #Creating the Node
        super().__init__(name)
        self.get_logger().info("Arrived 2")
        self.sub_raw = self.create_subscription(String,self.tracking_info, self.listener_callback,5)
        self.publisher_all_detected = self.create_publisher(String,self.tracking_info_pilot,5)
        self.image_raw = None
        self.image_all_detected = None
        self.cv_bridge = CvBridge()
        

    
    

######################## Publisher #####################################################################################  
    
    def listener_callback(self, msg):
        person_list = json.loads(msg.data)
        if person_list:
            pilot = person_list[0]
            topublish = String()
            topublish.data = json.dumps(pilot)
            self.publisher_all_detected.publish(topublish) 
            self.get_logger().info(f"Person to track in test node {topublish.data}")

        else:
            print("Empty person list\n")


    def tick(self) -> NodeState:
        """This method is a mandatory for PluginBase node. It defines what we want our node to do.
        It gets called 20 times a second if state=RUNNING
        """

        #self.publisher_all_detected.publish(self.cv_bridge.cv2_to_imgmsg(self.image_all_detected, 'rgb8')) 


        return NodeState.RUNNING

def main(args=None):
    #Intialization ROS communication 
    print("Arrived 6")

    rclpy.init(args=args)
    test = Test1('all')

    test.get_logger().info("Arrived 7")
    #execute the callback function until the global executor is shutdown
    rclpy.spin(test)

   
    #destroy the node. It is not mandatory, since the garbage collection can do it
    test.destroy_node()
    
    
    rclpy.shutdown()        
