#To handle ROS node
import rclpy
from rclpy.node import Node

#----- ROS messages

    # landmarks messages
from hand_gestures_msgs.msg import Landmarks, Landmark

    # person tracked messages and bounding boxes messages
from person_tracking_msgs.msg import PersonTracked, PointMsg, AllBoundingBoxes, Box


#----- Utilities
from copy import copy

from person_tracking.helpers import euclidean_distance_squared, extract_box_msg, extract_point_msg, person_lost, calculate_midpoint, equal_point_msg



class TriggerTrackingHandGestures(Node):

    # topic names
    hand_landmarks_topic = "/hand/landmarks"
    person_tracked_topic = "/person_tracked"
    bounding_boxes_topic = "/all_bounding_boxes"

    # trigger gestures 
    right_hand_gesture_trigger = "Open_Palm"
    left_hand_gesture_trigger = "Open_Palm"

    right_hand_gesture_stop= "Closed_Fist"
    left_hand_gesture_stop = "Closed_Fist"

    # amount of (0,0) midpoints to receive before concluding that the person is lost. 
    max_empty_midpoint_before_lost = 10

    # subscribers
    sub_bounding_boxes = None
    sub_landmark  = None

    # publisher
    publisher_to_track = None
    timer = None

    publishing_rate = 0.05 #20 hertz
    
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

        # variable to contain received landmarks messages
        self.landmarks = None

        # variable to contain the midpoint of the bounding box around the person we want to track
        self.person_tracked_midpoint = None

        # variables to contain the points located at the center of the left and right hand wrists of the person who performed the triggering gestures
        self.person_tracked_left_hand_point = None
        self.person_tracked_right_hand_point = None

        # variable to contain the custom message to send to the tracking node
        self.person_tracked_msg = PersonTracked()

        # boolean variable to know whether to start or stop the tracking when spotting the trigger gesture.
        self.tracking = False 
         
        # variable to count empty midpoints. We send an empty midpoint when the midpoint of the tracked person doesn't change.
        # when the person gets out of the field of view, the midpoint will stay the same for a long period. 
        # hence, if for a certain amount of time, the midpoint doesn't change, 
        # we can conclude that the person went out of the firld of view of the camera, and we can rotate.
        self.empty_midpoint_count = 0 # variable to count the amount of empty messages received. If this number is higher than a certain number, the person is considered lost.
        

       
      
    def _init_parameters(self)->None:
        """Method to initialize parameters such as ROS topics' names """
        self.declare_parameter("hand_landmarks_topic",self.hand_landmarks_topic) 
        self.declare_parameter("person_tracked_topic",self.person_tracked_topic) 
        self.declare_parameter("bounding_boxes_topic", self.bounding_boxes_topic)
        self.declare_parameter("right_hand_gesture_trigger",self.right_hand_gesture_trigger)
        self.declare_parameter("left_hand_gesture_trigger",self.left_hand_gesture_trigger)
        self.declare_parameter("right_hand_gesture_stop",self.right_hand_gesture_stop)
        self.declare_parameter("left_hand_gesture_stop",self.left_hand_gesture_stop)
        self.declare_parameter("max_empty_midpoint_before_lost",self.max_empty_midpoint_before_lost)

        self.hand_landmarks_topic= (
        self.get_parameter("hand_landmarks_topic").get_parameter_value().string_value
        )
        self.person_tracked_topic= (
        self.get_parameter("person_tracked_topic").get_parameter_value().string_value
        )
        self.bounding_boxes_topic = (
        self.get_parameter("bounding_boxes_topic").get_parameter_value().string_value
        )

        self.right_hand_gesture_trigger = (
        self.get_parameter("right_hand_gesture_trigger").get_parameter_value().string_value
        )

        self.left_hand_gesture_trigger = (
        self.get_parameter("left_hand_gesture_trigger").get_parameter_value().string_value
        ) 
        self.right_hand_gesture_stop = (
        self.get_parameter("right_hand_gesture_stop").get_parameter_value().string_value
        )

        self.left_hand_gesture_stop = (
        self.get_parameter("left_hand_gesture_stop").get_parameter_value().string_value
        ) 
        self.max_empty_midpoint_before_lost = (
        self.get_parameter("max_empty_midpoint_before_lost").get_parameter_value().integer_value
        )

        


    def _init_publishers(self)->None:
        """Method to initialize publishers"""
        self.publisher_to_track = self.create_publisher(PersonTracked,self.person_tracked_topic,5)
        self.timer = self.create_timer(self.publishing_rate, self.person_tracked_callback)

 

    def _init_subscriptions(self)->None:
        """Method to initialize subscriptions"""
        self.sub_bounding_boxes = self.create_subscription(AllBoundingBoxes,self.bounding_boxes_topic, self.bounding_boxes_listener_callback,5)
        self.sub_landmark = self.create_subscription(Landmarks,self.hand_landmarks_topic, self.landmarks_listener_callback,5)

########################### First Subscriber ###########################################################################################   
    def bounding_boxes_listener_callback(self, boxes_msg):
        """Callback function for the subscriber node (to topic /all_bounding_boxes).
        For each bounding box received, save it in a variable for processing"""
        self.get_logger().info('Bounding boxes message received')
        self.boxes = boxes_msg
        
        
########################### Second Subscriber #########################################################################################
    def landmarks_listener_callback(self, lndmrk):
        """Callback function for the subscriber node (to topic /hand/landmarks).
        Receives a landmark from the hand gesture plugin and saves that landmark in a variable for further processing."""
        self.get_logger().info('Landmark received')
        self.landmarks = lndmrk
        

######################### Publisher #####################################################################################################
    def person_tracked_callback(self):
        """This function listens to the /hand/landmarks topic, and waits to spot the person who did the triggering move. 
        In case a person did the trigger move, the midpoint of the bounding box around that person is published on a the topic named /person_tracked. 
        The last node (track_person.py) will subscribe to /person_tracked and send commands to the drone to follow the tracked person."""

        if self.boxes is None:
            self.get_logger().info("No bounding box received")

        else:
            if not self.boxes.bounding_boxes:#empty lists in Python can be evaluated as a boolean False. Hence this test is to make sure that boxes are received
                self.get_logger().info(f"The list of bounding boxes is empty. Hence, maybe no detection were made.")
                return None

            if self.tracking == False: #if no one has done the trigger move yet
                if self.check_gesture(True):
                    self.get_logger().info("\n Tracking Started!!")
                    self.tracking = True

                    #Saving the location of the hands of the person who did the move so that we can map him/her to a bounding box.
                    #Only the points at the center of the person's wrists are kept.
                    self.person_tracked_left_hand_point = self.landmarks.left_hand.normalized_landmarks[0]
                    self.person_tracked_right_hand_point = self.landmarks.right_hand.normalized_landmarks[0]
                    self.get_logger().info(f"{self.landmarks.right_hand.gesture} {self.landmarks.left_hand.gesture}")
                    
                    #Instantiating the midpoint ROS message
                    self.person_tracked_midpoint = PointMsg()
                    
                    #Find the bounding box around the person who did the trigger gesture, so that the midpoint of the box can be calculated
                    self.find_bounding_box_of_tracked_person()
                                        
                    if self.person_tracked_midpoint is not None:
                        self.publisher_to_track.publish(self.person_tracked_msg)                  

            else: #if someone did the trigger move yet 
            
                if not person_lost(self.empty_midpoint_count, self.max_empty_midpoint_before_lost): #if the tracked person is not lost (is still within the camera's field)

                    if self.check_gesture(False):
                        self.get_logger().info("\n Tracking Ended!")
                        self.tracking = False
                        self.person_tracked_midpoint = None
                            
                    else:
                        self.update_middlepoint()
                        self.publisher_to_track.publish(self.person_tracked_msg) 
                        self.get_logger().info(f"\nNow we know the person to track. midpoint is {self.person_tracked_msg.midpoint} \n")
                        #self.get_logger().info(f"{self.landmarks.right_hand.gesture} {self.landmarks.left_hand.gesture}")
                
                else: #if the tracked person is lost, we start tracking the person detected by our YOLO model with the highest confidence score (the person from the first bounding box)
                    if self.boxes.bounding_boxes:#empty lists in Python can be evaluated as a boolean False. Hence this test is to make sure that boxes are received
                        highest_conf_box = self.boxes.bounding_boxes[0]
                        new_midpoint = PointMsg()
                        new_midpoint.x , new_midpoint.y  = calculate_midpoint(highest_conf_box.top_left, highest_conf_box.bottom_right)

                        if equal_point_msg(new_midpoint, self.person_tracked_midpoint):
                            self.get_logger().info(f'Person lost and midpoint unchanged. The connection with the drone might be lost') 
                            self.empty_midpoint_count += 1
                        else:
                            self.person_tracked_midpoint = new_midpoint
                            self.empty_midpoint_count = 0
                            self.get_logger().info(f"\n\n#######################################\n\n#######################################\nStarted tracking a new person!\n#############################################\n")
                            self.get_logger().info(f'So the midpoint of that person is {self.person_tracked_midpoint}') 
                            #self.person_tracked_msg = PersonTracked()
                            self.person_tracked_msg.midpoint = copy(self.person_tracked_midpoint)
                            self.person_tracked_msg.bounding_box = copy(highest_conf_box) 
                            self.publisher_to_track.publish(self.person_tracked_msg)   


        
           
    def check_gesture(self, trigger:bool):
        """Function used to check if the trigger gesture was done by someone.
        Returns True if someone did the gesture and False if not
        Parameter trigger is used to specify whether the function is used to spot the trigger move (trigger == True)
        Or the gesture prompting to stop the tracking (trigger == False)"""

        if trigger and self.landmarks is not None and self.landmarks.right_hand.gesture == self.right_hand_gesture_trigger and self.landmarks.left_hand.gesture == self.left_hand_gesture_trigger:
            return True
        elif not trigger and self.landmarks is not None and self.landmarks.right_hand.gesture == self.right_hand_gesture_stop and self.landmarks.left_hand.gesture == self.left_hand_gesture_stop:
            return True
        else:
            return False


    def find_bounding_box_of_tracked_person(self)->None:
        """Finds the bounding box around the person who did the triggering move and updates self.person_tracked_midpoint
        Precondition: self.boxes is not None"""
        
        #Loop to go through all the bounding boxes to find the box in which are contained both the left hand and right hand wrist center point
        for box in self.boxes.bounding_boxes:

            #Extracting the normalized coordinates of the top left and bottom right corner of each box. 
            top_left_x , top_left_y, bottom_right_x, bottom_right_y = extract_box_msg(box)
            
            #Extracting the normalized coordinates of the left and right hand wrists' center points
            left_hand_x, left_hand_y = extract_point_msg(self.person_tracked_left_hand_point)
            right_hand_x, right_hand_y = extract_point_msg(self.person_tracked_right_hand_point)
            

            #If both wrist' center points are in the bounding box, we update the midpoint 
            if top_left_x <= left_hand_x and top_left_x <= right_hand_x:
                if bottom_right_x >= left_hand_x and bottom_right_x >= right_hand_x:
                    if top_left_y <= left_hand_y and top_left_y <= right_hand_y:
                        if bottom_right_y >= left_hand_y and bottom_right_y >= right_hand_y:

                            self.person_tracked_midpoint.x = top_left_x/ 2 + bottom_right_x/2 
                            self.person_tracked_midpoint.y = top_left_y/2 + bottom_right_y/2
                            
                            self.person_tracked_msg.midpoint = copy(self.person_tracked_midpoint) #self.denormalize()
                            self.person_tracked_msg.bounding_box = copy(box)

                            self.get_logger().info(f"top_left: {top_left_x}, {top_left_y}")
                            self.get_logger().info(f"bottom_right: {bottom_right_x}, {bottom_right_y}")
                            self.get_logger().info(f'First time midpoint updated to {self.person_tracked_msg.midpoint} \n')#{self.person_tracked_midpoint}')

                            return  
                            
        if self.person_tracked_midpoint.x == 0 and self.person_tracked_midpoint.y == 0:
            self.get_logger().info(f"\nCannot find the person who did the gesture")
            self.tracking = False #<-------- if there is ever an error, check this ----####----####----####----####----####----####----####----####----####----####

    
    def update_middlepoint(self)->None:
        """Find the nearest bounding box containing the middlepoint and updates the middlepoint.
        This function is to use only when we already have a person to track (and hence, self.person_tracked_midpoint is not None)

        Preconditions: self.person_tracked_midpoint is not None
                      self.boxes is not None
        """
        #error between the actual midpoint (self.person_tracked_midpoint) and the midpoint of the bounding box
        smallest_error_margin = -1
        prev_midpoint_x, prev_midpoint_y =  extract_point_msg(copy(self.person_tracked_midpoint))

        bounding_box = Box()
        self.get_logger().info(f"\nUpdate middlepoint boxeS {self.boxes.bounding_boxes}\n\n")

        #Loop to go through all the bounding boxes to find the bounding box whose midpoint is the closest to the midpoint of person tracked in the previous frame.
        #The midpoint found will be the new midpoint of the person to track
        for box in self.boxes.bounding_boxes:
            #Extracting the normalized coordinates of the top left and bottom right corner of each box.            
            top_left_x, top_left_y, bottom_right_x, bottom_right_y = extract_box_msg(copy(box))


            # self.get_logger().info(f"Searching the person from last midpoint {box}\n")
            
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
            #temp_midpoint = PointMsg() is initialized to x = 0 and y = 0 by default since ROS initializes all numeric values to 0 by default
            self.person_tracked_msg.midpoint = PointMsg()
            self.person_tracked_msg.bounding_box = Box()
            self.empty_midpoint_count += 1
            self.get_logger().info(f'No Midpoint update. Publishing empty midpoint') 
            
        else:
            self.person_tracked_msg.midpoint = copy(self.person_tracked_midpoint)#self.denormalize()
            self.person_tracked_msg.bounding_box = copy(bounding_box)
            self.empty_midpoint_count = 0
            self.get_logger().info(f'Midpoint updated to {self.person_tracked_msg.midpoint}') 
            
            
    
###################################################################################################################################       
  


def main(args=None):
    #Initialization ROS communication 
    rclpy.init(args=args)

    #Node instantiation
    trigger_tracking = TriggerTrackingHandGestures('pilot_person_selector_node')

    #Execute the callback function until the global executor is shutdown
    rclpy.spin(trigger_tracking)


    #destroy the node. It is not mandatory, since the garbage collection can do it
    trigger_tracking.destroy_node()
    
    rclpy.shutdown()        