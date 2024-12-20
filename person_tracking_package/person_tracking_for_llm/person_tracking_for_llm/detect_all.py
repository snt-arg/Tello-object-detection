################################### Imports #######################################
#To handle ROS node
import rclpy

#ROS image message
from sensor_msgs.msg import Image # for camera frames

from std_msgs.msg import String # for tracking info

#Custom message to send bounding boxes
from person_tracking_msgs.msg import AllBoundingBoxes, Box

#To convert cv2 images to ROS Image messages
from cv_bridge import CvBridge

#Node base to be able to integrate our project to the Behaviour tree
from plugin_server_base.plugin_base import PluginBase, NodeState

#Pygame interface
from tello_control_station.interface import Interface, matching_keys

#YOLOv8 object detection framework
from ultralytics import YOLO

#for gpu 
import torch

# to convert tracking info in JSON format
import json




##### Load the yolo model and send to appropriate device
model = YOLO('yolov8n.pt') 

#use gpu if available, and if not, cpu
device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
model.to(device)

##### Define classes of interest

person_classes = ["person"] # person class names
objects = ["cell phone"] # list of objects 
classes_needed = person_classes + objects

#Getting the ids of our classes of interest
classes = model.names
classes_ID = [k for k,v in classes.items() if v.lower() in classes_needed] 


class DetectAllLLM(PluginBase):
    
    #Minimum confidence probability for a detection to be accepted
    minimum_prob = 0.4

    # Overlapping method
    overlapping_method = "intersection"

    #Topic names
    image_raw_topic = "/camera/image_raw" #raw image frames from the drone' camera
    all_detected_topic = "/all_detected" # image frames of which all persons (and specified objects) are detected
    bounding_boxes_topic = "/all_bounding_boxes" # list of person bounding boxes
    key_pressed_topic = "/key_pressed" # topic to get the key pressed on the pygame interface
    tracking_info_topic = "/tracking_info" # topic to publish the information on persons and objects for llm-base commands package

    #ROS Subscriptions
    sub_raw = None
    
    #ROS Publishers
    publisher_all_detected = None
    publisher_bounding_boxes = None
    publisher_key_pressed = None
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
        self.persons_objects_list = None

        #Counter to track how many frames were received.
        self.frame_counter = 0 

        #Pygame interface
        self.pg_interface = Interface()

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
        self.declare_parameter("key_pressed_topic", self.key_pressed_topic)
        self.declare_parameter("minimum_prob",self.minimum_prob)
        self.declare_parameter("tracking_info_topic",self.tracking_info_topic)
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

        self.key_pressed_topic = (
        self.get_parameter("key_pressed_topic").get_parameter_value().string_value
        )

        self.minimum_prob = (
        self.get_parameter("minimum_prob").get_parameter_value().double_value
        ) 

        self.tracking_info_topic = (
        self.get_parameter("tracking_info_topic").get_parameter_value().string_value
        ) 

        self.overlapping_method = (
        self.get_parameter("overlapping_method").get_parameter_value().string_value
        ) 

           
    def _init_publishers(self)->None:
        """Method to initialize publishers"""
        self.publisher_all_detected = self.create_publisher(Image,self.all_detected_topic,5)
        self.publisher_bounding_boxes = self.create_publisher(AllBoundingBoxes,self.bounding_boxes_topic,5)
        self.publisher_key_pressed = self.create_publisher(String,self.key_pressed_topic,5)
        self.publisher_tracking_info = self.create_publisher(String,self.tracking_info_topic,5)
        

    def _init_subscriptions(self)->None:
        """Method to initialize subscriptions"""
        self.sub_raw = self.create_subscription(Image,self.image_raw_topic, self.listener_callback,5)



########################### Subscriber ###########################################################################################   

    def listener_callback(self, img)->None:
        """Callback function for the subscriber node (to topic /camera/image_raw).
        For each image received, it saves in the log that an image has been received.
        Then converts that image into cv2 format before performing object detection on that image and saving 
        the result in self.image_all_detected"""

        # we process only 1/4 of frames to reduce computing power consumption
        if self.process % 4 == 0:

            self.get_logger().info(f"Frame N°{self.frame_counter} received")
            self.frame_counter += 1
        
            self.image_raw = self.cv_bridge.imgmsg_to_cv2(img,"rgb8") # converting ROS Image message to cv2 image
            self.image_all_detected = self.detection(self.image_raw) # performing detection on the cv2 image
            self.pg_interface.update_bg_image(img) # updating the image on the pygame console

        self.process += 1
        
    def detection(self,frame)->None:
        """Function to perform person object detection on a single frame.
        It saves the coordinates of all bounding boxes of persons detected on the frame in a variable named self.boxes"""

        # Detection of persons & objects in the frame. Only detections with a certain confidence level (minimum_prob) are  considered.
        results = model.track(frame, persist=True, classes=classes_ID, conf=self.minimum_prob)

        # Initializing all bounding boxes messages
        self.boxes = AllBoundingBoxes()
        
        # Initializing tracking info message
        self.json_string_msg = String()

        # temporary variable to contain the list of all objects detected
        tmp_list_object = []

        #temporary variable to contain the list of all person detected
        tmp_list_person = []

        # dictionnary to map each object to its possessor (the person whose bounding box shares the biggest area with the objects bounding box)
        object_possessor_dict = dict()

        # list to contain info dictionnary for each person. This list will be converted to JSON. 
        tojson_list = []
        
        # For loop to get all go through all detections (persons and objects)
        for box in results[0].boxes:  
            
            # coordinates of the bounding box (top left and bottom right)
            box_coordinates = box.xyxyn[0].tolist() #normalized coordinates (within 0 and 1)

            # class of the detection (person, cell phone, ...)
            box_class = classes[box.cls.item()].lower()

            # if there is an id assigned by the YOLO model, we save it
            if box.id is not None:
                box_id = box.id.item()
            else:
                box_id = None

            # If the detection is a person
            if box_class in person_classes:
                # Saving the coordinates of the bounding box around the person for it to be published on bounding_boxes_topic ("/all_bounding_boxes") 
                box_msg = Box()
                box_msg.top_left.x = box_coordinates[0]
                box_msg.top_left.y = box_coordinates[1]
                box_msg.bottom_right.x = box_coordinates[2]
                box_msg.bottom_right.y = box_coordinates[3]

                self.boxes.bounding_boxes.append(box_msg)

                # Adding the person to the temporary list of all persons
                if box_id is not None:
                    tmp_list_person.append((box_coordinates,box_id))
                else:
                    tmp_list_person.append((box_coordinates,-1)) # if the person doesn't have an id are no ids, we append -1 

            # if the detection is an object 
            else:
                object_class = box_class

                #Adding the object to the temporary list of objects
                tmp_list_object.append((box_coordinates,object_class)) 

        # Merging duplicate detections in person classes

        # For loop to map each object to its possessor
        for object_coordinates, object_class in tmp_list_object:

            # list of overlapping areas between the object and each person
            overlapping_area_list = [self.overlapping_area(object_coordinates,person_coordinates) for person_coordinates,_ in tmp_list_person]
            max_overlap = max(overlapping_area_list, default=-1)
            # possessor is the person whose bounding box shares the biggest area with the object's box

            print(f"\n%%% overlapping list, {object_class} : {overlapping_area_list}\n")

            if max_overlap >= 0: # the overlapping_area method returns -1 if the rectangles don't overlap. So this condition is to ensure that the object's box overlaps with at least one person, before assigning the object to that person

                possessor_index = overlapping_area_list.index(max(overlapping_area_list))
                # add mapping in dictionnary
                if possessor_index in object_possessor_dict:
                    object_possessor_dict[possessor_index].append(object_class)
                else:
                    object_possessor_dict[possessor_index] = [object_class]
        
        print(f"\n## Possessors : {object_possessor_dict}\n")
        # For loop to build the list of infos about each person.
        # For each person we save the coordinates of the bounding box, the id assigned by the YOLO model, and the list of objects that the person possesses

        for person_counter in range(len(tmp_list_person)):
            person_coordinates,person_id = tmp_list_person[person_counter]

            info_dict = dict()
            info_dict["bottom_right"] = (person_coordinates[2],person_coordinates[3])
            info_dict["top_left"] = (person_coordinates[0],person_coordinates[1])
            info_dict["YOLO_id"] = person_id
            info_dict["objects"] = object_possessor_dict.get(person_counter,[]) #["hat"] if person_counter == 0 else []

            # Additional format needed by convention
            person_dict = dict()
            person_dict["action"] = "tracking"
            person_dict["id"] = person_counter
            person_dict["info"] = info_dict

            # append the person to the list to be converted to JSON
            tojson_list.append(person_dict)
        
        # JSON string message to send to LLM command interpreter

        self.json_string_msg.data = json.dumps(tojson_list)

        print(str(self.json_string_msg.data))

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
            self.get_logger().info("Publishing a bounding boxes list")

    def key_pressed_callback(self)->None:
        keys = self.pg_interface.get_key_pressed()
        msg = String()
        # takeoff/land
        if keys[matching_keys["t"]]:
            msg.data = "t"
            self.publisher_key_pressed.publish(msg)
            return 
        
        if keys[matching_keys["l"]]:
            msg.data = "l"
            self.publisher_key_pressed.publish(msg)
            return

    def person_objects_callback(self)->None:
        """Callback function for the publisher to the /tracking_info topic.
        A list of the coordinates of the bounding boxes around persons, persons ID and the objects they have on them"""
        if self.json_string_msg is not None:
            self.publisher_tracking_info.publish(self.json_string_msg)

    def overlapping_area(self,rect1,rect2):
        """Function to calculate the overlapping area between two rectangles
        Parameters: rect1 (rectangle given by a list [top_left_x, top_left_y, bottom_right_x, bottom_right_y])
                    rect2 (rectangle given by a list [top_left_x, top_left_y, bottom_right_x, bottom_right_y])

        """
        dx = min(rect1[2],rect2[2]) - max(rect1[0],rect2[0]) 
        dy = min(rect1[3],rect2[3]) - max(rect1[1],rect2[1]) 

        area1 = abs(rect1[2] - rect1[0]) * abs(rect1[3] - rect1[1]) # area rectangle 1
        area2 = abs(rect2[2] - rect2[0]) * abs(rect2[3] - rect2[1]) # area rectangle 2

        if dx < 0 or dy < 0: # if the rectangles don't overlap
            return -1

        if self.overlapping_method == "intersection":
            return dx * dy
            
        elif self.overlapping_method == "IOU":
            intersection_area = dx * dy 
            union_area = area1 + area2 - intersection_area

            return intersection_area/union_area
            
        else:
            raise ValueError("Overlapping method must be either intersection or IOU.\nPlease change configuration to enter a correct overlapping method")


    def tick(self) -> NodeState:
        """This method is a mandatory for PluginBase node. It defines what we want our node to do.
        It gets called 20 times a second if state=RUNNING
        Here we call callback functions to publish a detection frame and the list of bounding boxes.
        """
        self.pg_interface.tick()
        self.key_pressed_callback()
        self.all_detected_callback()
        self.bounding_boxes_callback()
        self.person_objects_callback()
        
        return NodeState.RUNNING


def main(args=None):
    #Initialization of ROS communication  
    rclpy.init(args=args)

    #Node instantiation
    detector = DetectAllLLM('all_person_detector')

    #execute the callback function until the global executor is shutdown
    rclpy.spin(detector)

    #destroy the node. It is not mandatory, since the garbage collection can do it
    detector.destroy_node()
    
    rclpy.shutdown()        
