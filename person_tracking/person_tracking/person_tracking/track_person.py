#To handle ROS node
import rclpy
from rclpy.node import Node

from std_msgs.msg import Empty

#ROS Twist message import. This message type is used to send commands to the drone.
from geometry_msgs.msg import Twist


#Custom message containing the midpoint of the bounding box surrounding the tracked person
from person_tracked.msg import PersonTracked, PointMsg

#For image manipulation (OpenCV)
import cv2
#import numpy as np

#To convert cv2 images to ROS Image messages
from cv_bridge import CvBridge

from person_tracking.pid import PIDPoint

class TrackPerson(Node):

    
    person_tracked_topic = "/person_tracked"
    #bounding_boxes_topic = "/all_bounding_boxes"
    commands_topic = "/cmd_vel" #carries Twist msgs
    land_topic = "/land" #carries Empty msgs


    image_height = 480
    image_width = 640

    
    def __init__(self,name):
        #Creating the Node
        super().__init__(name)
        
        #subscribers
        self.sub_person_tracked = self.create_subscription(PersonTracked,self.person_tracked_topic, self.listener_callback,10)
        #self.sub_landmark = self.create_subscription(Landmarks,self.hand_landmarks_topic, self.landmarks_listener_callback,10)
        
    
        #publishers
        self.publisher_commands = self.create_publisher(Twist,self.commands_topic,10)
        self.timer_1 = self.create_timer(0.1, self.commands_callback)
        
        #self.publisher_land = self.create_publisher(Empty,self.land_topic,10)
        #self.timer_2 = self.create_timer(0.1, self.land_callback)

        #self.cv_bridge = CvBridge()

        #Variable to received bounding boxes containing all persons detected
        #self.boxes = None
        self.pid = PIDPoint((0.5, 0.5)) #middle of the screen for normalized midpoint coordinates
       
        self.person_tracked_midpoint = None

        self.commands_msg = None

        self.correction = None

        
###########################first subscriber###########################################################################################   
    def listener_callback(self, msg):
        """Callback function for the subscriber node (to topic /person_tracked).
        Receives the midpoint of the boundign box surrounding the person tracked"""
        
        self.get_logger().info('Midpoint received')
        self.person_tracked_midpoint = msg.middle_point
        self.correction = self.pid.compute(self.person_tracked_midpoint)
               
######################### Publisher #####################################################################################################
    def commands_callback(self):
        """This function sends appropriate to the drone in order to keep the tracked person within the camera's field while ensuring safety"""
        self.commands_msg = Twist()
        
        if self.correction is not None:
            correction_x, correction_y = self.correction
            if correction_x < -0.2 : #Here I don't put 0 to avoid having the drone always moving
                #print("move left")
                #self.commands_msg.li
                print("move right")
                self.commands_msg.linear.y -= 0.5

            elif correction_x > 0.2 :
                #print("move right")
                print("move left")
                self.commands_msg.linear.y += 0.5

            if correction_y < -0.2 :
                #print("move down")
                print("move up")
                self.commands_msg.linear.z += 0.5

            elif correction_y > 0.2 :
                #print("move up")
                print("move down")
                self.commands_msg.linear.z -= 0.5

            self.publisher_commands.publish(self.commands_msg) 
                    
###################################################################################################################################       
  


def main(args=None):
    #Intialization ROS communication 
    rclpy.init(args=args)
    track_person = TrackPerson('Track_Person_node')

    #execute the callback function until the global executor is shutdown
    rclpy.spin(track_person)
    
    track_person.video.release()

    #destroy the node. It is not mandatory, since the garbage collection can do it
    track_person.destroy_node()
    
    rclpy.shutdown()        
