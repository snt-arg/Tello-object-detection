#To handle ROS node
import rclpy
from rclpy.node import Node

#----- ROS messages
from std_msgs.msg import String 


#person tracked messages and bounding boxes messages
from person_tracking_msgs.msg import PersonTracked, PointMsg, AllBoundingBoxes, Box

from copy import copy

import json

from person_tracking_for_llm.helpers import euclidean_distance_squared, overlapping_area, person_lost, extract_point_msg

             
###################################################################################################################################       
class TriggerTrackingLLM(Node):

    # topic names
    person_tracked_topic = "/person_tracked"
    bounding_boxes_topic = "/all_bounding_boxes"
    tracking_signal_topic = "/tracking_info_pilot_person"

    # amount of (0,0) midpoints to receive before concluding that the person is lost.  
    max_empty_midpoint_before_lost = 100

    # subscribers
    sub_bounding_boxes = None
    sub_tracking_signal = None

    # publisher
    publisher_to_track = None
    timer = None

    publishing_rate = 0.1 #20 hertz

    overlapping_method = "intersection"

    
    def __init__(self,name):

        # creating the Node
        super().__init__(name)

        # initializing parameters
        self._init_parameters()

        # initializing subscribers
        self._init_subscriptions()
        
        # initializing publishers
        self._init_publishers()
        

        # variable to receive bounding boxes containing all persons detected
        self.boxes = None

        # variable to contain the midpoint of the bounding box around the person we want to track
        self.person_tracked_midpoint = None

        # coordinates of the bounding box around the pilot person
        self.person_box = None

        # variable to contain the custom message to send to the ROS node responsible of following the person
        self.person_tracked_msg = PersonTracked()

        # boolean variable to know whether to start or stop the tracking when receiving a tracking signal
        self.tracking = False 
         
        # variable to count empty midpoints. We send an empty midpoint when the midpoint of the tracked person doesn't change.
        # when the person gets out of the field of view, the midpoint will stay the same for a long period. 
        # hence, if for a certain amount of time, the midpoint doesn't change, 
        # we can conclude that the person went out of the field of view of the camera, and we can rotate.
        self.empty_midpoint_count = 0 # variable to count the amount of empty messages received. If this number is higher than a certain number, the person is considered lost.
        
        ### test
        self.i = 0
        ### end test
       

	
        
         
        
    def _init_parameters(self)->None:
        """Method to initialize parameters such as ROS topics' names """

        self.declare_parameter("person_tracked_topic",self.person_tracked_topic) 
        self.declare_parameter("bounding_boxes_topic", self.bounding_boxes_topic)
        self.declare_parameter("tracking_signal_topic",self.tracking_signal_topic)
        self.declare_parameter("max_empty_midpoint_before_lost",self.max_empty_midpoint_before_lost)
        self.declare_parameter("overlapping_method",self.overlapping_method)

        self.person_tracked_topic = (
        self.get_parameter("person_tracked_topic").get_parameter_value().string_value
        )
        self.bounding_boxes_topic = (
        self.get_parameter("bounding_boxes_topic").get_parameter_value().string_value
        )
        self.tracking_signal_topic = (
        self.get_parameter("tracking_signal_topic").get_parameter_value().string_value
        )
        self.max_empty_midpoint_before_lost = (
        self.get_parameter("max_empty_midpoint_before_lost").get_parameter_value().integer_value
        )
        self.overlapping_method = (
        self.get_parameter("overlapping_method").get_parameter_value().string_value
        )


    def _init_publishers(self)->None:
        """Method to initialize publishers"""
        self.publisher_to_track = self.create_publisher(PersonTracked,self.person_tracked_topic,10)
        self.timer = self.create_timer(self.publishing_rate, self.person_tracked_callback)



    def _init_subscriptions(self)->None:
        """Method to initialize subscriptions"""
        self.sub_bounding_boxes = self.create_subscription(AllBoundingBoxes,self.bounding_boxes_topic, self.bounding_boxes_listener_callback,5)
        self.sub_tracking_signal = self.create_subscription(String,self.tracking_signal_topic,self.tracking_signal_listener_callback,5)

########################### First Subscriber ##################################################################################################   
    def bounding_boxes_listener_callback(self, boxes_msg):
        """Callback function for the subscriber node (to topic /all_bounding_boxes).
        For each bounding box received, save it in a variable for processing"""

        self.get_logger().info('\nBounding boxes message received')
        self.boxes = boxes_msg
        
        ### test
        if self.boxes is not None and self.boxes.bounding_boxes != []: 
            if self.i == 0:
                self.tracking = True
                bounding_boxes = copy(self.boxes.bounding_boxes)
                self.person_tracked_midpoint = PointMsg()
    
                self.person_box = copy(bounding_boxes[0])
                self.person_tracked_midpoint.x = (self.person_box.bottom_right.x + self.person_box.top_left.x) / 2 
                self.person_tracked_midpoint.y = (self.person_box.bottom_right.y + self.person_box.top_left.y) / 2

                self.get_logger().info(f"\nTest found the pilot person indicated in the tracking signal. Midpoint : {self.person_tracked_midpoint}\n")

                self.person_tracked_msg = PersonTracked()
                self.person_tracked_msg.midpoint = copy(self.person_tracked_midpoint)
                self.person_tracked_msg.bounding_box = copy(self.person_box)
                
                self.publisher_to_track.publish(self.person_tracked_msg)

                self.i += 1
             

        ### end test
       
     
######################### Second subscriber ####################################################################################################
    def tracking_signal_listener_callback(self, signal_msg):
        """Callback function to receive the signal message to start tracking a person at a specific location"""
        self.get_logger().info(f"\n!!!Tracking signal received {signal_msg}!!!\n")

        infodict = json.loads(signal_msg.data)

        # if we aren't tracking and receive a tracking signal prompting to start the tracking:
        if self.tracking == False and infodict["action"] == "tracking":
            self.tracking = True
            self.person_tracked_midpoint = PointMsg()
            bottom_right = infodict["info"]["bottom_right"]
            top_left = infodict["info"]["top_left"]

            self.person_box = Box()
            self.person_box.top_left.x = top_left[0]
            self.person_box.top_left.y = top_left[1]
            self.person_box.bottom_right.x = bottom_right[0]
            self.person_box.bottom_right.y = bottom_right[1]

            # Search the pilot person box in our list of bounding boxes
            self.find_bounding_box_of_pilot_person()

        # if we receive a message to stop the tracking.
        elif infodict["action"] == "stop_tracking":
            self.tracking = False
            self.person_tracked_midpoint = PointMsg()

######################### First Publisher #####################################################################################################
    def person_tracked_callback(self):
        """This function listens to the /hand/landmarks topic, and waits to spot the person who did the triggering move. 
        In case a person did the trigger move, the midpoint of the bounding box around that person is published on a the topic named /person_tracked. 
        The last node (track_person.py) will subscribe to /person_tracked and send commands to the drone to follow the tracked person."""
        
        if self.boxes is None:
            self.get_logger().info("No bounding box received")

        else:
            if not self.boxes.bounding_boxes:#empty lists in Python can be evaluated as a boolean False. Hence this test is to make sure that boxes are received
                self.get_logger().info(f"The list of bounding boxes is empty. Hence, maybe no detection were made.")
                

            if self.tracking:
                if not person_lost(self.empty_midpoint_count, self.max_empty_midpoint_before_lost): #if the tracked person is not lost (is still within the camera's field)
                    self.person_tracked_msg = PersonTracked()
                    self.update_middlepoint()
                    self.publisher_to_track.publish(self.person_tracked_msg) 
                    self.get_logger().info(f"\nNow we know the person to track. midpoint is {self.person_tracked_msg.midpoint} \n")
                
                else: #if the tracked person is lost, we start tracking the person detected by our YOLO model with the highest confidence score (the person from the first bounding box)
                    
                    if self.boxes.bounding_boxes:#empty lists in Python can be evaluated as a boolean False. Hence this test is to make sure that boxes are received
                        highest_conf_box = self.boxes.bounding_boxes[0]
                        midpoint = PointMsg()
                        midpoint.x = (highest_conf_box.top_left.x + highest_conf_box.bottom_right.x) / 2
                        midpoint.y = (highest_conf_box.top_left.y + highest_conf_box.bottom_right.y) / 2

                        # if the midpoint is updated (it is a new detection)
                        #if midpoint.x != self.person_tracked_midpoint.x or midpoint.y != self.person_tracked_midpoint.y:
                        self.person_tracked_msg = PersonTracked()

                        self.person_tracked_midpoint = copy(midpoint)
                        self.person_box = copy(highest_conf_box)

                        self.person_tracked_msg.midpoint = copy(self.person_tracked_midpoint)
                        self.person_tracked_msg.bounding_box = copy(self.person_box)

                        self.empty_midpoint_count = 0
                          
                        self.publisher_to_track.publish(self.person_tracked_msg)
                        #else:
                        #    self.get_logger().info(f'Person lost and midpoint unchanged. The connection with the drone might be lost') 
                        #    self.empty_midpoint_count += 1
                                               

            else :#if we didn't receive a signal to track the person or received the signal to stop tracking the person, we send midpoint(-1,-1) to tell tracker_node to stop sending velocity messages
                self.person_tracked_msg = PersonTracked()
                self.person_tracked_msg.midpoint.x = -1.0
                self.person_tracked_msg.midpoint.y = -1.0
                self.publisher_to_track.publish(self.person_tracked_msg)
                self.get_logger().info(f"\n Not tracking. Publishing a (-1,-1) midpoint")

    def find_bounding_box_of_pilot_person(self)->None:
        """Finds the bounding box of the pilot person
        Precondition: self.boxes is not None"""
        
        #Loop to go through all the bounding boxes to find the box of the pilot person
        if self.boxes is not None and self.person_box is not None:

            bounding_boxes = copy(self.boxes.bounding_boxes)

            overlap_list = [overlapping_area(self.person_box,box,self.overlapping_method) for box in bounding_boxes]

            max_overlap = max(overlap_list, default=-1)  # if the list is empty

            if max_overlap == -1:
            
                self.get_logger().info(f"\nCannot find the pilot person indicated in the tracking signal")
                self.tracking = False 
                return None
            
            else:
                self.person_box = copy(bounding_boxes[overlap_list.index(max_overlap)])
                self.person_tracked_midpoint.x = (self.person_box.bottom_right.x + self.person_box.top_left.x) / 2 
                self.person_tracked_midpoint.y = (self.person_box.bottom_right.y + self.person_box.top_left.y) / 2

                self.get_logger().info(f"\nFound the pilot person indicated in the tracking signal. Midpoint : {self.person_tracked_midpoint}")

                self.person_tracked_msg = PersonTracked()
                self.person_tracked_msg.midpoint = copy(self.person_tracked_midpoint)
                self.person_tracked_msg.bounding_box = copy(self.person_box)
                
                self.publisher_to_track.publish(self.person_tracked_msg)
                
        else:
            self.get_logger().info("No bounding box received or no tracking info received")
                                    
    
    def update_middlepoint(self)->None:
        """Find the nearest bounding box containing the middlepoint and updates the middlepoint.
        This function is to use only when we already have a person to track (and hence, self.person_tracked_midpoint is not None)

        Preconditions: self.person_tracked_midpoint is not None
                      self.boxes is not None
        """
        #error between the actual midpoint (self.person_tracked_midpoint) and the midpoint of the bounding box
        smallest_error_margin = -1
        prev_midpoint_x = self.person_tracked_midpoint.x
        prev_midpoint_y = self.person_tracked_midpoint.y
        bounding_box = Box()

        #Loop to go through all the bounding boxes to find the bounding box whose midpoint is the closest to the midpoint of person tracked in the previous frame.
        #The midpoint found will be the new midpoint of the person to track
        for box in self.boxes.bounding_boxes:
            #Extracting the normalized coordinates of the top left and bottom right corner of each box.            
            top_left_x = box.top_left.x
            top_left_y = box.top_left.y
            bottom_right_x = box.bottom_right.x
            bottom_right_y = box.bottom_right.y


            #self.get_logger().info(f"Searching the person from last midpoint {box}\n")
            
            #if the previous midpoint is contained within the box, we calculate the distance between both points
            #if that distance is the shortest encountered yet, we update the midpoint
            if top_left_x <= prev_midpoint_x and top_left_y <= prev_midpoint_y:
                if bottom_right_x >= prev_midpoint_x and bottom_right_y >= prev_midpoint_y:
                    
                    new_midpoint_x = top_left_x/ 2 + bottom_right_x/2
                    new_midpoint_y = top_left_y/2 + bottom_right_y/2
                    error_margin = euclidean_distance_squared(prev_midpoint_x,prev_midpoint_y,new_midpoint_x,new_midpoint_y) #distance between midpoints
                
            #updating the midpoint
            
                    if (smallest_error_margin == -1) or (error_margin < smallest_error_margin):
                        self.person_tracked_midpoint.x = new_midpoint_x
                        self.person_tracked_midpoint.y = new_midpoint_y
                        smallest_error_margin = error_margin
                        bounding_box = copy(box)

                        self.get_logger().info(f"possible update top_left: {top_left_x}, {top_left_y}")
                        self.get_logger().info(f"possible update bottom_right: {bottom_right_x}, {bottom_right_y}")

        
        #In case no bounding box is found, we send a midpoint of coordinates (0,0)
        if prev_midpoint_x == self.person_tracked_midpoint.x and prev_midpoint_y == self.person_tracked_midpoint.y:
            self.person_tracked_msg.midpoint = PointMsg()
            self.person_tracked_msg.bounding_box = Box()
            self.empty_midpoint_count += 1
            self.get_logger().info(f'No Midpoint update. Publishing empty midpoint') 
            
        else:
            self.person_tracked_msg.midpoint = copy(self.person_tracked_midpoint)#self.denormalize()
            self.person_tracked_msg.bounding_box = copy(bounding_box)
            self.empty_midpoint_count = 0
            self.get_logger().info(f'Midpoint updated to {self.person_tracked_msg.midpoint}') 
            


#############################################################################################################################################################

def main(args=None):
    #Initialization ROS communication 
    rclpy.init(args=args)

    #Node instantiation
    trigger_tracking = TriggerTrackingLLM('pilot_person_selector_llm_node')

    #Execute the callback function until the global executor is shutdown
    rclpy.spin(trigger_tracking)
    
    #destroy the node. It is not mandatory, since the garbage collection can do it
    trigger_tracking.destroy_node()
    
    rclpy.shutdown()        
