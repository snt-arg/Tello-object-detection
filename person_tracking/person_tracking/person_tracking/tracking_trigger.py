#To handle ROS node
import rclpy
from rclpy.node import Node

#ROS image message
#from sensor_msgs.msg import Image

#Landmarks messages
from hand_gestures_msgs.msg import Landmarks, Landmark

#person tracked messages
from person_tracked.msg import PersonTracked, PointMsg

#bounding boxes messages
from all_bounding_boxes_msg.msg import AllBoundingBoxes

#To handle images
#import cv2

#To convert cv2 images to ROS Image messages
from cv_bridge import CvBridge




class TriggerTracking(Node):

    #Topic names
    hand_landmarks_topic = "/hand/landmarks"
    person_tracked_topic = "/person_tracked"
    bounding_boxes_topic = "/all_bounding_boxes"

    #Trigger gestures 
    right_hand_gesture_trigger = "Open_Palm"
    left_hand_gesture_trigger = "Open_Palm"

    right_hand_gesture_stop= "Closed_Fist"
    left_hand_gesture_stop = "Closed_Fist"

    
    def __init__(self,name):

        #Creating the Node
        super().__init__(name)
        
        #subscribers
        self.sub_bounding_boxes = self.create_subscription(AllBoundingBoxes,self.bounding_boxes_topic, self.bounding_boxes_listener_callback,10)
        self.sub_landmark = self.create_subscription(Landmarks,self.hand_landmarks_topic, self.landmarks_listener_callback,10)
        
        """self.test_sub = self.create_subscription(Image,"/all_detected",self.test_listener,10)"""

        #publishers
        self.publisher_to_track= self.create_publisher(PersonTracked,self.person_tracked_topic,10)
        self.timer_1 = self.create_timer(0.1, self.person_tracked_callback)
        
        #Used to convert cv2 frames into ROS Image messages and vice versa
        self.cv_bridge = CvBridge()

        #Variable to receive bounding boxes containing all persons detected
        self.boxes = None

        #Variable to contain received landmarks messages
        self.landmarks = None

        #variable to contain the midpoint of the bounding box around the person we want to track
        self.person_tracked_midpoint = None

        #variables to contain the points located at the center of the left and right hand wrists of the person who performed the triggering gestures
        self.person_tracked_left_hand_point = None

        self.person_tracked_right_hand_point = None

        #Variable to contain the custom message to send to 
        self.person_tracked_msg = PersonTracked()

        #Boolean variable to know whether to start or stop the tracking when spotting the trigger gesture.
        self.tracking = False 
         
        """self.image_height = None

        self.image_width = None

        self.fourcc = cv2.VideoWriter_fourcc(*'mp4v')
        self.video = cv2.VideoWriter('/media/maeri/UBUNTUUU/output1118.mp4',self.fourcc,20.0,(640,480))"""

################################ test subscriber #####################################################################################
    """def test_listener(self,frame_msg):
        #To delete , just a test listener callback to verify the midpoint is really the midpoint
        frame = self.cv_bridge.imgmsg_to_cv2(frame_msg,'bgr8')
        self.image_height, self.image_width, _ = frame.shape
        if self.person_tracked_midpoint is not None:
            self.video.write(cv2.circle(frame,(int(self.person_tracked_msg.middle_point.x),int(self.person_tracked_msg.middle_point.y)),10,(255,0,0),-1))"""

########################### First Subscriber ###########################################################################################   
    def bounding_boxes_listener_callback(self, boxes_msg):
        """Callback function for the subscriber node (to topic /all_bounding_boxes).
        For each bounding box received, save it in a variable for processing"""
        self.get_logger().info('Bounding boxes received')
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
            if self.tracking == False: #if no one has done the trigger move yet
                if self.check_gesture(True):
                    self.get_logger().info("\n Tracking Started!!")
                    self.tracking = True
                    #Saving the location of the hands of the person who did the move so that we can map him/her to a bounding box.
                    #Only the points at the center if ther person's wrists are kept.
                    self.person_tracked_left_hand_point = self.landmarks.left_hand.normalized_landmarks[0]
                    self.person_tracked_right_hand_point = self.landmarks.right_hand.normalized_landmarks[0]
                    self.get_logger().info(f"{self.landmarks.right_hand.gesture} {self.landmarks.left_hand.gesture}")
                    
                    #Instantiating the midepoint ROS message
                    self.person_tracked_midpoint = PointMsg()
                    #print("Bounding boxes",self.boxes.bounding_boxes,"\n\n")
                    #self.video.write(cv2.line(self.image_all_detected,(int(middle_left.x*self.width),int(middle_left.y*self.height)),(int(middle_right.x*self.width),int(middle_right.y*self.height)),(255,0,0),4))
                    
                    #Find the bounding box around the person who did the trigger gesture, so that the midpoint of the box can be calculated
                    self.find_bounding_box_of_tracked_person()
                    
                    self.person_tracked_msg.middle_point = self.person_tracked_midpoint #self.denormalize()  <--- This is useful if we want to send the real coordinates of the midpoint, and not coordinate within the range [0,1]
                    
                    if self.person_tracked_midpoint is not None:
                        self.publisher_to_track.publish(self.person_tracked_msg)                  

            else: #if someone did the trigger move yet
                if self.check_gesture(False):
                    self.get_logger().info("\n Tracking Ended!")
                    self.tracking = False
                    self.person_tracked_midpoint = None
                        
                else:
                    self.find_bounding_box_middlepoint()
                    self.person_tracked_msg.middle_point = self.person_tracked_midpoint #self.denormalize() <--- This is useful if we want to send real coordinates of the midpoint, and not coordinate within the range [0,1]
                    self.publisher_to_track.publish(self.person_tracked_msg) 
                    print("\nNow we know the person to track. midpoint is ", self.person_tracked_midpoint,"\n")
                    self.get_logger().info(f"{self.landmarks.right_hand.gesture} {self.landmarks.left_hand.gesture}")


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

    def denormalize(self)->PointMsg():
        """Function to denormalize the coordinates of the midpoint"""
        result = PointMsg()
        result.x = self.person_tracked_midpoint.x * self.image_width
        result.y = self.person_tracked_midpoint.y * self.image_height
        return result

    def find_bounding_box_of_tracked_person(self)->None:
        """Finds the bounding box around the person who did the triggering move and updates self.person_tracked_midpoint
        Precondition: self.boxes is not None"""
        
        #Loop to go through all the bounding boxes to find the box in which are contained both the left hand and right hand wrist center point
        for box in self.boxes.bounding_boxes:

            #Extracting the normalized coordinates of the top left and bottom right corner of each box. 
            top_left_x = box.top_left.x
            top_left_y = box.top_left.y
            bottom_right_x = box.bottom_right.x
            bottom_right_y = box.bottom_right.y

            #Extracting the normalized coordinates of the left and right hand wrists' center points
            left_hand_x = self.person_tracked_left_hand_point.x
            left_hand_y = self.person_tracked_left_hand_point.y
            right_hand_x = self.person_tracked_right_hand_point.x
            right_hand_y = self.person_tracked_right_hand_point.y

            #If both wrist' center points are in the bounding box, we update the midpoint 
            if top_left_x <= left_hand_x and top_left_x <= right_hand_x:
                if bottom_right_x >= left_hand_x and bottom_right_x >= right_hand_x:
                    if top_left_y <= left_hand_y and top_left_y <= right_hand_y:
                        if bottom_right_y >= left_hand_y and bottom_right_y >= right_hand_y:
                            self.person_tracked_midpoint.x = top_left_x/ 2 + bottom_right_x/2 
                            self.person_tracked_midpoint.y = top_left_y/2 + bottom_right_y/2
                            self.get_logger().info(f"top_left: {top_left_x}, {top_left_y}")
                            self.get_logger().info(f"bottom_right: {bottom_right_x}, {bottom_right_y}")
                            self.get_logger().info(f'First time midpoint updated to {self.person_tracked_midpoint}')
                            return  
        if self.person_tracked_midpoint is None:
            self.get_logger().info(f"\nCannot find the person who did the gesture")

    
    def find_bounding_box_middlepoint(self)->None:
        """Find the nearest bounding box containing the middlepoint and updates the middlepoint.
        This function is to use only when we already have a person to track (and hence, self.person_tracked_midpoint is not None)

        Preconditions: self.person_tracked_midpoint is not None
                      self.boxes is not None
        """
        #error between the actual midpoint (self.person_tracked_midpoint) and the midpoint of the bounding box
        smallest_error_margin = -1
        prev_midpoint = self.person_tracked_midpoint
        #Loop to go through all the bounding boxes to find the bounding box whose midpoint is the closest to the midpoint of person tracked in the previous frame.
        #The midpoint found will be the new midpoint of the person to track
        for box in self.boxes.bounding_boxes:
            #Extracting the normalized coordinates of the top left and bottom right corner of each box.            
            top_left_x = box.top_left.x
            top_left_y = box.top_left.y
            bottom_right_x = box.bottom_right.x
            bottom_right_y = box.bottom_right.y

            #Extracting the previous midpoint's coordinates
            midpoint_x = self.person_tracked_midpoint.x
            midpoint_y = self.person_tracked_midpoint.y

            #if the previous midpoint is contained within the box, we calculate the distance between both points
            #if that distance is the shortest encountered yet, we update the midpoint
            if top_left_x <= midpoint_x and top_left_y <= midpoint_y and bottom_right_x >= midpoint_x and bottom_right_y >= midpoint_y:
                new_midpoint_x = top_left_x/ 2 + bottom_right_x/2
                new_midpoint_y = top_left_y/2 + bottom_right_y/2
                error_margin = self.euclidean_distance_squared(midpoint_x,midpoint_y,new_midpoint_x,new_midpoint_y) #distance between midpoints
                self.get_logger().info(f"top_left: {top_left_x}, {top_left_y}")
                self.get_logger().info(f"bottom_right: {bottom_right_x}, {bottom_right_y}")
                #updating the midpoint
                if (smallest_error_margin == -1) or (error_margin < smallest_error_margin):
                    self.person_tracked_midpoint.x = new_midpoint_x
                    self.person_tracked_midpoint.y = new_midpoint_y
                    smallest_error_margin = error_margin
        
        self.get_logger().info(f'Midpoint updated to {self.person_tracked_midpoint}') 
    

    def euclidean_distance_squared(self,x1,y1,x2,y2):
        """Calculates the eucliedean distance between two points (x1,y1) and (x2,y2)"""
        return (x2-x1)**2 + (y2-y1)**2


"""
    def track_target(self,frame,id_of_interest):
        #Function to track a person of a certain id only on a frame

        results = model.track(frame,persist=True) #stream = True
        index_of_target = 0
        target_in_frame = False
        for result in results[0]:
            if id_of_interest in result.boxes.id.tolist():
                #midpoint = result.boxes.xywh[0][0:2]
                results[0] = results[0][i]
                print(f"Tracking person ID {id_of_interest} at {result.boxes.xywh[0][0:2]}")
                frame_ = results[0].plot()
                target_in_frame = True
                index_of_target = index_of_target + 1
        if target_in_frame:
            return frame_
        else:
            return frame
"""
       
	    
    
###################################################################################################################################       
  


def main(args=None):
    #Initialization ROS communication 
    rclpy.init(args=args)

    #Node instantiation
    trigger_tracking = TriggerTracking('trigger_tracking_node')

    #Execute the callback function until the global executor is shutdown
    rclpy.spin(trigger_tracking)
    
        #trigger_tracking.video.release()

    #destroy the node. It is not mandatory, since the garbage collection can do it
    trigger_tracking.destroy_node()
    
    rclpy.shutdown()        
