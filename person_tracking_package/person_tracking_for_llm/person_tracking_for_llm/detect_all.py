################################### Imports #######################################

#for handling ROS node
import rclpy

#----- ROS image messages : 

    # for camera frames
from sensor_msgs.msg import Image 

    # for tracking info
from std_msgs.msg import String 

    # custom message to send bounding boxes
from person_tracking_msgs.msg import AllBoundingBoxes, Box


#----- Utilities 

    # for converting cv2 images to ROS Image messages and vice versa
from cv_bridge import CvBridge

    # for converting tracking info in JSON format
import json

    # node base for behaviour tree
from plugin_server_base.plugin_base import PluginBase, NodeState


    # YOLOv8 object detection framework
from ultralytics import YOLO

    # for sending model to gpu if available
import torch

    # custom helper functions
from person_tracking_for_llm.helpers import yolo_box_to_Box_msg, assign_objects_to_persons, construct_JSON_string



################################### Global variables #######################################
# object detection model
model = YOLO('yolov8n.pt') 

# using gpu if available
device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
model.to(device)

# classes of interest (for detection)
person_classes = ["person"] # person class names
objects = ["cell phone"] # list of objects 
classes_needed = person_classes + objects

# ids of our classes of interest
classes = model.names
classes_ID = [k for k,v in classes.items() if v.lower() in classes_needed] 


class DetectAllLLM(PluginBase):
    
    # minimum confidence probability for a detection to be accepted
    minimum_prob = 0.4

    # overlapping method
    overlapping_method = "intersection"

    # topic names
    image_raw_topic = "/camera/image_raw" #raw image frames from the drone's camera
    all_detected_topic = "/all_detected" # image frames in which all persons (and specified objects) are detected
    bounding_boxes_topic = "/all_bounding_boxes" # list of person bounding boxes
    tracking_info_topic = "/tracking_info" # topic to publish the information on persons and objects for llm-base commands package

    #ROS Subscriptions
    sub_raw = None
    
    #ROS Publishers
    publisher_all_detected = None
    publisher_bounding_boxes = None
    publisher_tracking_info = None

    def __init__(self,name):

        #Creating the Node
        super().__init__(name)

        #init topic names
        self._init_parameters()

        #init subscribers
        self._init_subscriptions()
        
        #init publishers
        self._init_publishers()

        #to convert cv2 images to Ros Image messages and vice versa
        self.cv_bridge = CvBridge()

        #Variable to contain the frame coming directly from the drone
        self.image_raw = None

        #Variable to hold each frame after object detection. It contains all persons detected
        self.image_all_detected = None

        #Variable containing all bounding boxes' coordinates for a single frame
        self.boxes = None

        # Variable containing the list of persons and the objects on them (with their coordinates)
        #self.persons_objects_list = None

        #Counter to track how many frames were received.
        self.frame_counter = 0 


        #Variable to perform object detection on only some frames
        self.process = 0

        # Variable to contain the tracking info in JSON format
        self.json_string_msg = None

        
################################ Init functions ##################################################################################
    def _init_parameters(self)->None:
        """Method to initialize parameters such as ROS topics' names """

        #Topic names
        self.declare_parameter("image_raw_topic",self.image_raw_topic) 
        self.declare_parameter("all_detected_topic",self.all_detected_topic) 
        self.declare_parameter("bounding_boxes_topic", self.bounding_boxes_topic )
        self.declare_parameter("tracking_info_topic",self.tracking_info_topic)

        # others 
        self.declare_parameter("minimum_prob",self.minimum_prob)
        self.declare_parameter("overlapping_method",self.overlapping_method)

        self.image_raw_topic= (
        self.get_parameter("image_raw_topic").get_parameter_value().string_value
        )

        self.all_detected_topic= (
        self.get_parameter("all_detected_topic").get_parameter_value().string_value
        )

        self.bounding_boxes_topic = (
        self.get_parameter("bounding_boxes_topic").get_parameter_value().string_value
        )


        self.tracking_info_topic = (
        self.get_parameter("tracking_info_topic").get_parameter_value().string_value
        ) 

        self.minimum_prob = (
        self.get_parameter("minimum_prob").get_parameter_value().double_value
        ) 

        self.overlapping_method = (
        self.get_parameter("overlapping_method").get_parameter_value().string_value
        ) 

           
    def _init_publishers(self)->None:
        """Method to initialize publishers"""
        self.publisher_all_detected = self.create_publisher(Image,self.all_detected_topic,5)
        self.publisher_bounding_boxes = self.create_publisher(AllBoundingBoxes,self.bounding_boxes_topic,5)
        self.publisher_tracking_info = self.create_publisher(String,self.tracking_info_topic,5)
        

    def _init_subscriptions(self)->None:
        """Method to initialize subscriptions"""
        self.sub_raw = self.create_subscription(Image,self.image_raw_topic, self.listener_callback,5)



########################### Callback functions ###########################################################################################   

    def listener_callback(self, img)->None:
        """Callback function for the subscriber node (to topic /camera/image_raw).
        For each image received, it saves in the log that an image has been received.
        Then converts that image into cv2 format before performing object detection on that image and saving 
        the result in self.image_all_detected"""

        # processing only 1/4 of frames to reduce computing power consumption
        if self.process % 4 == 0:
            
            # Logs
            #self.get_logger().info(f"Frame N°{self.frame_counter} received")
            self.frame_counter += 1
        
            # performing object detection
            self.image_raw = self.cv_bridge.imgmsg_to_cv2(img,"rgb8") # converting ROS Image message to cv2 image
            self.image_all_detected = self.detection(self.image_raw) # performing detection on the cv2 image
            
        self.process += 1
        
    def detection(self,frame)->None:
        """Function to perform person object detection on a single frame.
        It saves the coordinates of all bounding boxes of persons detected on the frame in a variable named self.boxes"""

        # detection of persons & objects in the frame. Only detections with a certain confidence level (minimum_prob) are  considered.
        results = model.track(frame, persist=True, classes=classes_ID, conf=self.minimum_prob)

        # Initializing all bounding boxes messages
        self.boxes = AllBoundingBoxes()
        
        # Initializing tracking info message
        self.json_string_msg = String()

        # temporary variable to contain the list of all objects detected
        tmp_list_object = []

        #temporary variable to contain the list of all person detected
        tmp_list_person = []

        # dictionnary to map each object to its possessor (the person whose bounding box shares the biggest area with the object's bounding box)
        possessor_to_objects_dict = dict()

        # For loop to get all go through all detections (persons and objects)
        for box in results[0].boxes:  
            
            # coordinates of the bounding box (top left and bottom right)
            box_coordinates = yolo_box_to_Box_msg(box.xyxyn[0].tolist()) #normalized coordinates (within 0 and 1) 

            # class of the detection (person, cell phone, ...)
            box_class = classes[box.cls.item()].lower()

            # if there is an id assigned by the YOLO model, we save it
            if box.id is not None:
                box_id = box.id.item()
            else:
                box_id = -1

            # If the detection is a person
            if box_class in person_classes:
                # Saving the coordinates of the bounding box around the person for it to be published on bounding_boxes_topic ("/all_bounding_boxes") 
                self.boxes.bounding_boxes.append(box_coordinates)

                # Adding the person to the temporary list of all persons
                tmp_list_person.append((box_coordinates,box_id))

            # if the detection is an object 
            else:
                #Adding the object to the temporary list of objects
                tmp_list_object.append((box_coordinates,box_class)) 


        possessor_to_objects_dict = assign_objects_to_persons(tmp_list_person, tmp_list_object, self.overlapping_method)
        
        #print(f"\n## Possessors : {possessor_to_objects_dict}\n") #<--- to debug
        
        # JSON string message to send to LLM command interpreter
        self.json_string_msg.data = construct_JSON_string(tmp_list_person,possessor_to_objects_dict)

        self.get_logger().info(str(self.json_string_msg.data))

        #returning the image where bounding boxes are displayed
        frame_ = results[0].plot()

        return frame_


######################## Publisher #####################################################################################  
    def all_detected_callback(self)->None:
        """
        callback funtion for the publisher node (to topic /camera/image_detected).
        The image on which object detection has been performed (self.image_all_detected) is published on the topic '/all_detected'
        """
        if(self.image_all_detected is None):
            self.get_logger().info("Can't publish frames on which object detection was performed.\n No image has been received from the drone yet")    
        else:
            self.publisher_all_detected.publish(self.cv_bridge.cv2_to_imgmsg(self.image_all_detected, 'rgb8')) 
            self.get_logger().info("Publishing a frame on all detected topic")
            
    
    def bounding_boxes_callback(self)->None:
        """
        callback funtion for the publisher node (to topic /all_bounding_boxes).
        A list of the coordinates of bounding boxes around persons detected on the frame is published.
        """
        if(self.boxes is None):
            self.get_logger().info("Can't publish bounding boxes. No information received yet")    
        else:
            self.publisher_bounding_boxes.publish(self.boxes)
            self.get_logger().info("\n##############################################\nPublishing a bounding boxes list\n\n")


    def person_objects_callback(self)->None:
        """Callback function for the publisher to the /tracking_info topic.
        A list of the coordinates of the bounding boxes around persons, persons ID and the objects they have on them"""
        if self.json_string_msg is not None:
            self.publisher_tracking_info.publish(self.json_string_msg)


    def tick(self) -> NodeState:
        """This method is a mandatory for PluginBase node. It defines what we want our node to do.
        It gets called 20 times a second if state=RUNNING
        Here we call callback functions to publish a detection frame and the list of bounding boxes.
        """
        self.all_detected_callback()
        self.bounding_boxes_callback()
        self.person_objects_callback()
        
        return NodeState.RUNNING


def main(args=None):
    #Initialization of ROS communication  
    rclpy.init(args=args)

    #Node instantiation
    detector = DetectAllLLM('person_detector_llm_node')

    #execute the callback function until the global executor is shutdown
    rclpy.spin(detector)

    #destroy the node. It is not mandatory, since the garbage collection can do it
    detector.destroy_node()
    
    rclpy.shutdown()        
