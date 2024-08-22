#To handle ROS node
import rclpy
from rclpy.node import Node

from std_msgs.msg import Empty, String

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

from threading import Thread


##NB : all directions : left, right... are from the drone's perspective

class TrackPerson(Node):

    key_pressed_topic = "/key_pressed"
    person_tracked_topic = "/person_tracked"
    #bounding_boxes_topic = "/all_bounding_boxes"
    commands_topic = "/cmd_vel" #carries Twist msgs
    land_topic = "/land" #carries Empty msgs
    takeoff_topic = "/takeoff"


    #image_height = 480
    #image_width = 640

    max_length_midpoint_queue = 2

    max_empty_midpoint_before_lost = 10

    
    def __init__(self,name):
        #Creating the Node
        super().__init__(name)
        
        #subscribers
        self.sub_person_tracked = self.create_subscription(PersonTracked,self.person_tracked_topic, self.listener_callback,5)

        self.sub_key_pressed = self.create_subscription(String,self.key_pressed_topic, self.key_pressed_subscriber_callback,5)

        #publishers
        self.publisher_commands = self.create_publisher(Twist,self.commands_topic,10)
        self.timer = self.create_timer(0.09, self.commands_callback)
        #self.timer_1 = self.create_timer(10, self.commands_callback)

        self.publisher_takeoff = self.create_publisher(Empty,self.takeoff_topic,1)
       
        self.publisher_land = self.create_publisher(Empty,self.land_topic,1)
        

        self.pid = PIDPoint((0.5, 0.5)) #middle of the screen for normalized midpoint coordinates
       
        self.person_tracked_midpoint = None

        self.prev_midpoint = None 

        self.commands_msg = None

        self.correction = None
        
        #Queue to keep the last nonempty midpoints. Help to calculate the trajectory of the person before he got lost
        self.midpoint_queue = deque(maxlen=self.max_length_midpoint_queue)
        
        self.empty_midpoint_count = 0 #variable to count the amount of empty messages received. If this number is higher than a certain number, the person is considered lost.
        
        #Counter for self.persont_tracked_midpoint in the update message function.
        #If the midpoint is the same after a certain number of calls to the function, the connection might be broken. so we send empty command messages
        self.connection_lost_midpoint_unchanged_counter = 0  
        
        self.update_midpoint = False #is true when we receive a new midpoint that is different than (0,0)

        self.key_pressed = None #variable to contain key pressed on the Pygame GUI, either to land or takeoff

        self.flying = True#variable is True when the dron is flying and False if it landed

        self.rotating = False #variable to check if the drone is rotating

        

        
        
###########################first subscriber###########################################################################################   
    def listener_callback(self, msg):
        """Callback function for the subscriber node (to topic /person_tracked).
        Receives the midpoint of the bounding box surrounding the person tracked"""
        self.get_logger().info('Midpoint received')
        tmp = msg.middle_point

        if self.empty_midpoint(tmp):
            self.empty_midpoint_count += 1
            self.get_logger().info("Empty midpoint")
            self.update_midpoint = False
        else:
            self.update_midpoint = True
            self.person_tracked_midpoint = tmp
            self.midpoint_queue.append(self.person_tracked_midpoint)
            self.empty_midpoint_count = 0
            self.get_logger().info(f'midpoint {self.person_tracked_midpoint}')
            self.correction = self.pid.compute(self.person_tracked_midpoint)

    def direction_person_lost(self)->str|None:
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
        
        return None


############################Second subscriber#######################################################################3333
            
    def key_pressed_subscriber_callback(self,msg):
        self.key_pressed = msg.data
                      
######################### Publisher #####################################################################################################
    def commands_callback(self):
        self.land_takeoff()
        if self.flying and not self.rotating:
            #command_thread = Thread(target=self.update_commands).start()
            self.update_commands()
            

    def update_commands(self)->None:
        """This function makes appropriate commands messages in order to keep the tracked person within the camera's field while ensuring safety"""
        self.commands_msg = Twist()
        self.commands_msg.linear.x = 0.0
        self.commands_msg.linear.y = 0.0
        self.commands_msg.linear.z = 0.0
        self.commands_msg.angular.x = 0.0
        self.commands_msg.angular.y = 0.0
        self.commands_msg.angular.z = 0.0

        self.get_logger().info(f"\nself.person_lost :{self.person_lost()}\n")
        self.get_logger().info(f"\nself.empty_midpoint_count :{self.empty_midpoint_count}\n")

        
        if self.person_lost():

            #self.get_logger().info("\nWalaheiiiiiiiiiii\nWalaheiiiiiiiiiii\nWalaheiiiiiiiiiii\n")
            direction : str = self.direction_person_lost()

            if direction == "left":
                print("Rotate left") 
                #self.rotation(pi/5,2*pi)
                rotation_thread_left = Thread(target=self.rotation,args=(pi/7,2*pi))
                rotation_thread_left.start()
                #rotation_thread_left.join()
            
            elif direction == "right":
                print("Rotate right")
                #self.rotation(-pi/5,2*pi)
                rotation_thread_right = Thread(target=self.rotation,args=(-pi/7,2*pi))
                rotation_thread_right.start()  
                #rotation_thread_right.join()

            else:
                self.get_logger().info(f'No trajectory can be found')

        else:
            #check if the connection is still passing (meaning self.person_tracked_midpoint and hence self.empty_midpoint_count are updated in the listener callback)
            if not self.check_midpoint_changed():
                self.publisher_commands.publish(self.commands_msg) 
                return None

            
            if self.person_tracked_midpoint is not None and self.correction is not None :
                correction_x, correction_y = self.correction
                self.get_logger().info(f'Correction x:{correction_x}, y:{correction_y}')
                #self.get_logger().info(f'midpoint {self.person_tracked_midpoint}')

                if self.person_tracked_midpoint.x < 0.3:#correction_x < -0.6 : #Here I don't put 0 to avoid having the drone always moving
                    self.get_logger().info("move left") #(a in keyboard mode control station) 
                    self.commands_msg.linear.y = 0.22
                     
                elif self.person_tracked_midpoint.x > 0.7:#correction_x > 0.6 :
                    self.get_logger().info("move right") #(d in keyboard mode control station )             
                    self.commands_msg.linear.y = -0.22

            self.publisher_commands.publish(self.commands_msg) 
        
        return None
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
 
    def rotation(self, angular_speed, target_angle)->None:
        """Function to send rotation commands to the drone. 
        Returns True if the drone did a complete rotation (no one was found) and False else"""
        if self.commands_msg is not None:
            self.rotating = True
            current_angle = 0
            #target_angle = 2*pi
            self.commands_msg.angular.z = angular_speed#pi


            self.get_logger().info(f"Before rotating,  angular.z is {self.commands_msg.angular.z} and current_angle is {current_angle}")

            t0 = self.get_clock().now()

            while abs(current_angle) <= abs(target_angle): 
                #self.get_logger().info(f"Found a person? {self.update_midpoint}")
                if self.update_midpoint:
                    self.rotating = False
                    self.empty_midpoint_count = 0
                    self.get_logger().info(f"While rotating, found a person to track")
                    return 
                self.publisher_commands.publish(self.commands_msg) 
                t1 = self.get_clock().now()

                current_angle = self.commands_msg.angular.z * ((t1-t0).to_msg().sec)

            self.get_logger().info(f"After rotating ,angular.z is {self.commands_msg.angular.z} and current_angle is {current_angle}")
            self.rotating = False
            self.publisher_land.publish(Empty()) #land if we found no one after a complete rotation
            self.flying = False
            

    def empty_midpoint(self, midpoint):
        """Function to test if the current midpoint is empty (x==0 and y==0). 
        Returns True if the midpoint is empty, and false else
        Precondition : midpoint is not None and is of type PointMsg"""
        if midpoint.x == 0 and midpoint.y == 0:
            return True
        else:
            return False  

    def land_takeoff(self)->None:
        """Prompts the drone to land or takeoff, depending on the messages received"""
        if self.key_pressed is not None:
            if self.key_pressed == "t":
                self.publisher_takeoff.publish(Empty())
                self.flying = True
                return None
            if self.key_pressed == "l":
                self.publisher_land.publish(Empty())
                self.flying = False
                return None

    def equal_point_msg(self, p1, p2)->bool:
        """function to compare to point messages (PointMsg).
        Returns True if they have the same coordinates, and False else"""
        if isinstance(p1, PointMsg) and isinstance(p2, PointMsg):
            return p1.x == p2.x and p1.y == p2.y
        else:
            raise TypeError("Error in function equal_point_msg, tried to compare two objects that are not of type PointMsg")

    def check_midpoint_changed(self)->bool:
        """Function to check if the self.person_tracked_midpoint changed. Returns True if it stayed the same for less than max_empty_midpoint_before_lost, and False if it stayed the same for more.
        """
        if self.prev_midpoint is None and self.person_tracked_midpoint is not None:
            self.prev_midpoint = self.person_tracked_midpoint
            return True

        elif self.prev_midpoint is not None :
            if self.equal_point_msg(self.prev_midpoint, self.person_tracked_midpoint):
                self.connection_lost_midpoint_unchanged_counter += 1
            else:
                self.connection_lost_midpoint_unchanged_counter = 0
        
        self.prev_midpoint = self.person_tracked_midpoint

        if self.connection_lost_midpoint_unchanged_counter <= self.max_empty_midpoint_before_lost :
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

        ###########test##################################
        
        if self.test % 2 == 0:
            #print("move left")
            self.move("left")
            self.test += 1
        else:
            self.move("right")
            self.test += 1
                
        
        #########################end test################

        def move(self,direction):
        #function to move the drone left or right
        #direction can only be 'left' or 'right' 
            msg = Twist()

            if direction == "left":
                msg.linear.y = 0.5
            else:
                msg.linear.y = -0.5

            t0 = self.get_clock().now()
            t1 = self.get_clock().now()

            time = 2 if direction == "left" else 1
            while (t1-t0).to_msg().sec < 2: 
                self.publisher_commands.publish(msg) 
                t1 = self.get_clock().now()
            self.get_logger().info(f"movement  {direction}!!")


                

           # elif direction == "up":
                #self.commands_msg.linear.y += 0.5
            #    print("go up")

            #elif direction == "down":
                #self.commands_msg.linear.y -= 0.5
            #    print("go down") 
"""  

