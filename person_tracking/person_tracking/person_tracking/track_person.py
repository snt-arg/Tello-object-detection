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

from collections import deque

from math import pi

class TrackPerson(Node):

    
    person_tracked_topic = "/person_tracked"
    #bounding_boxes_topic = "/all_bounding_boxes"
    commands_topic = "/cmd_vel" #carries Twist msgs
    land_topic = "/land" #carries Empty msgs


    #image_height = 480
    #image_width = 640

    max_length_midpoint_queue = 2

    max_empty_midpoint_before_lost = 10

    
    def __init__(self,name):
        #Creating the Node
        super().__init__(name)
        
        #subscribers
        self.sub_person_tracked = self.create_subscription(PersonTracked,self.person_tracked_topic, self.listener_callback,10)

        #publishers
        self.publisher_commands = self.create_publisher(Twist,self.commands_topic,10)
        self.timer_1 = self.create_timer(0.5, self.commands_callback)
        #self.timer_1 = self.create_timer(10, self.commands_callback)

        
        #self.publisher_land = self.create_publisher(Empty,self.land_topic,10)
        #self.timer_2 = self.create_timer(0.1, self.land_callback)

        self.pid = PIDPoint((0.5, 0.5)) #middle of the screen for normalized midpoint coordinates
       
        self.person_tracked_midpoint = None

        self.commands_msg = None

        self.correction = None

        self.midpoint_queue = deque(maxlen=self.max_length_midpoint_queue)
        self.empty_midpoint_count = 0 #variable to count the amount of empty messages received. If this number is higher than a certain number, the person is considered lost.
        
        self.test = 0

###########################first subscriber###########################################################################################   
    def listener_callback(self, msg):
        """Callback function for the subscriber node (to topic /person_tracked).
        Receives the midpoint of the bounding box surrounding the person tracked"""
        
        #self.get_logger().info('Midpoint received')
        self.person_tracked_midpoint = msg.middle_point

        if self.empty_midpoint():
            self.empty_midpoint_count += 1
        else:
            self.midpoint_queue.append(self.person_tracked_midpoint)
            self.empty_midpoint_count = 0

        
        #self.get_logger().info(f'midpoint {self.person_tracked_midpoint}')
         
        self.correction = self.pid.compute(self.person_tracked_midpoint)

    def direction_person_lost(self):
        """Function to determine whether the drone should rotate left, right, up or down to find the lost person"""
        if len(self.midpoint_queue) == self.max_length_midpoint_queue:
            point_1 = self.midpoint_queue[1] #most recent midpoint
            point_2 = self.midpoint_queue[0] #previous midpoint
            slope = ( point_2.y - point_1.y ) / ( point_2.x - point_1.x )
            if slope >= 0 :
                if point_1.x >= point_2.x:
                    return "right"
                else:
                    return "left"

            else:
                if point_1.x >= point_2.x:
                    return "right"
                else:
                    return "left"

            """
            if slope >= 1:
                if point_1.y <= point_2.y:
                    return "down"

                else:
                    return "up"

            elif slope < 1 and slope > -1:

                if point_1.x <= point_2.x:
                    return "left"

                else:
                    return "right"

            else: #slope <-1
                if point_1.y <= point_2.y:
                    return "down"

                else:
                    return "up"
            """

        else:
            self.get_logger().info(f"\nNot enough midpoints received yet to predict the person's position\n")



               
######################### Publisher #####################################################################################################
    def commands_callback(self):
        """This function sends appropriate to the drone in order to keep the tracked person within the camera's field while ensuring safety"""
        self.commands_msg = Twist()
        self.get_logger().info(f"\nself.person_lost :{self.person_lost()}\n")
        self.get_logger().info(f"\nself.empty_midpoint_count :{self.empty_midpoint_count}\n")

        if self.person_lost():

            #self.get_logger().info("\nWalaheiiiiiiiiiii\nWalaheiiiiiiiiiii\nWalaheiiiiiiiiiii\n")
            direction = self.direction_person_lost()
            
            if direction == "left":
                print("Rotate left") 
                self.rotation(pi,2*pi)
                #to del
            
            elif direction == "right":
                print("Rotate right")
                self.rotation(-pi,2*pi)
                

           # elif direction == "up":
                #self.commands_msg.linear.y += 0.5
            #    print("go up")

            #elif direction == "down":
                #self.commands_msg.linear.y -= 0.5
            #    print("go down") 
            
            else:
                self.get_logger().info(f'No trajectory can be found')

        else:
            
            if self.correction is not None:
                correction_x, correction_y = self.correction
                self.get_logger().info(f'Correction x:{correction_x}, y:{correction_y}')
                self.get_logger().info(f'midpoint {self.person_tracked_midpoint}')

                if self.empty_midpoint():
                    self.get_logger().info('Empty midpoint')

                elif self.person_tracked_midpoint.x < 0.3:#correction_x < -0.6 : #Here I don't put 0 to avoid having the drone always moving
                    self.get_logger().info("move left")
                    #print("move right")
                    self.commands_msg.linear.y -= 0.3

                elif self.person_tracked_midpoint.x > 0.7:#correction_x > 0.6 :
                    self.get_logger().info("move right")
                    #print("move left")                
                    self.commands_msg.linear.y += 0.3

            self.publisher_commands.publish(self.commands_msg)
            #if correction_y < -0.6 :
            #    print("move down")
                #print("move up")
                #if self.move_down > 0:
            #    self.commands_msg.linear.z -= 0.3
                #    self.move_down -= 1
                #else:
                #    self.commands_msg.linear.z += 0.0

            #elif correction_y > 0.6 :
            #    print("move up")
                #print("move down")
                #if self.move_up > 0:
            #    self.commands_msg.linear.z += 0.3
                 #   self.move_up -= 1
                #else:
                #    self.commands_msg.linear.z += 0.0
        
    def person_lost(self):
        """Function to call when someone is lost.
        Returns true when the person is lost and False else."""  
        if self.empty_midpoint_count >= self.max_empty_midpoint_before_lost:
            return True
        else: 
            return False    
 
    def rotation(self, angular_speed, target_angle):
        """Function to send rotation commands to the drone. """
        if self.commands_msg is not None:
            current_angle = 0
            #target_angle = 2*pi

            self.commands_msg.angular.z = angular_speed#pi

            t0 = self.get_clock().now()

            while abs(current_angle) <= abs(target_angle): 
                if not self.empty_midpoint:
                    self.empty_midpoint_count = 0
                    break
                self.publisher_commands.publish(self.commands_msg) 
                t1 = self.get_clock().now()

                current_angle = self.commands_msg.angular.z * ((t1-t0).to_msg().sec)

            self.get_logger().info(f"rotation! angular.z is {self.commands_msg.angular.z} and current_angle is {current_angle}")

    def empty_midpoint(self):
        """Function to test if the current midpoint is empty (x==0 and y==0). 
        Returns True if the midpoint is empty, and false else"""
        if self.person_tracked_midpoint.x == 0 and self.person_tracked_midpoint.y == 0:
            return True
        else:
            return False               
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

"""####################TEST#######################
        #if (self.test % 10) == 0:
        #self.commands_msg.angular.z = 0.5
        #while self.commands_msg.angular.z != 0 :
        current_angle = 0
        target_angle = 2*pi

        self.commands_msg.angular.z = pi
        self.commands_msg.linear.x = 0.0
        self.commands_msg.linear.y = 0.0
        self.commands_msg.linear.z = 0.0
        self.commands_msg.angular.y = 0.0
        self.commands_msg.angular.x = 0.0

        t0 = self.get_clock().now()

        while current_angle < target_angle: 
            self.publisher_commands.publish(self.commands_msg) 
            t1 = self.get_clock().now()

            current_angle = self.commands_msg.angular.z * ((t1-t0).to_msg().sec)

            self.get_logger().info(f"rotation {self.test}. angular.z is {self.commands_msg.angular.z} and current_angle is {current_angle}")
            self.test += 1
        
        #############END TEST#########################
"""  
