#To handle ROS node
import rclpy
from rclpy.node import Node

#ROS image message
from sensor_msgs.msg import Image

#Custom message to send bounding boxes
from all_bounding_boxes_msg.msg import AllBoundingBoxes, Box

#To convert cv2 images to ROS Image messages
from cv_bridge import CvBridge

#Node base to be able to integrate our project to the Tello_ws
from plugin_server_base.plugin_base import PluginBase, NodeState

#YOLOv8 object detection framework
from ultralytics import YOLO

#load the object detection model
model = YOLO('yolov8n.pt') 

#Defining claases of interest
classes_needed = ["person"]  

#Getting the ids of our classes of interest
classes = model.names
classes_ID = [k for k,v in classes.items() if v in classes_needed] 


class DetectAll(PluginBase):

    #Minimum confidence probability for a detection to be accepted
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
        
        #publishers
        self.publisher_all_detected = self.create_publisher(Image,self.all_detected_topic,10)
        self.publisher_bounding_boxes = self.create_publisher(AllBoundingBoxes,self.bounding_boxes_topic,10)
        
        #to convert cv2 images to Ros Image messages and vice versa
        self.cv_bridge = CvBridge()

        #Variable to contain the frame coming directly from the drone
        self.image_raw = None

        #Variable to hold each frame after object detection. It contains all persons detected
        self.image_all_detected = None

        #Variable containing all bounding boxes' coordinates for a single frame
        self.boxes = None

        #Counter to track how many frames were received.
        self.frame_counter = 0 




########################### Subscriber ###########################################################################################   
    def listener_callback(self, img):
        """Callback function for the subscriber node (to topic /camera/image_raw).
        For each image received, it saves in the log that an image has been received.
        Then converts that image into cv2 format before performing object detection on that image and saving 
        the result in self.image_all_detected"""
        
        self.get_logger().info(f"Frame N°{self.frame_counter} received")
        self.frame_counter += 1
        self.image_raw = self.cv_bridge.imgmsg_to_cv2(img,"rgb8")
        self.image_all_detected = self.detection(self.image_raw)
        

        
    def detection(self,frame):
        """Function to perform person object detection on a single frame.
        It saves the coordinates of all bounding boxes of persons detected on the frame in a variable named self.boxes"""

        #detection of persons in the frame. Only detections with a certain confidence level (minimum_prob) are  considered.
        results = model.track(frame, persist=True, classes=classes_ID, conf=self.minimum_prob)

        #temporary variable to hold each bounding box's coordinates on the frame.
        box_msg = Box()

        #Saving all bounding boxes's coordinates in self.boxes
        self.boxes = AllBoundingBoxes()
        for box in results[0].boxes.xyxyn.tolist(): #normalized coordinates (within 0 and 1)
            box_msg.top_left.x = box[0]
            box_msg.top_left.y = box[1]
            box_msg.bottom_right.x = box[2]
            box_msg.bottom_right.y = box[3]
            self.boxes.bounding_boxes.append(box_msg)
        self.get_logger().info(f"self.boxes : {self.boxes.bounding_boxes}")
        #returning the image where bounding boxes are displayed
        frame_ = results[0].plot()
        return frame_


######################## Publisher #####################################################################################  
    def all_detected_callback(self):
        """
        callback funtion for the publisher node (to topic /camera/image_detected).
        The image on which object detection has been performed (self.image_all_detected) is published on the topic '/all_detected'
        """
        if(self.image_all_detected is None):
            self.get_logger().info("Can't publish frames on which object detection was performed.\n No image has been received from the drone yet")    
        else:
            self.publisher_all_detected.publish(self.cv_bridge.cv2_to_imgmsg(self.image_all_detected, 'rgb8')) 
            self.get_logger().info("Publishing a frame on all detected topic")
            
    
    def bounding_boxes_callback(self):
        """
        callback funtion for the publisher node (to topic /all_bounding_boxes).
        A list of the coordinates of bounding boxes detected on the frame is published.
        """
        if(self.boxes is None):
            self.get_logger().info("Can't publish bounding boxes. No information received yet")    
        else:
            self.publisher_bounding_boxes.publish(self.boxes)
            self.get_logger().info("Publishing a bounding boxes")


    def tick(self) -> NodeState:
        """This method is a mandatory for PluginBase node. It defines what we want our node to do.
        It gets called 20 times a second if state=RUNNING
        Here we call callback functions to publish a detection frame and the list of bounding boxes.
        """
        self.all_detected_callback()
        self.bounding_boxes_callback()
        return NodeState.RUNNING


def main(args=None):
    #Initialization of ROS communication  
    rclpy.init(args=args)

    #Node instantiation
    detector = DetectAll('all_person_detector')

    #execute the callback function until the global executor is shutdown
    rclpy.spin(detector)

    #destroy the node. It is not mandatory, since the garbage collection can do it
    detector.destroy_node()
    
    rclpy.shutdown()        
