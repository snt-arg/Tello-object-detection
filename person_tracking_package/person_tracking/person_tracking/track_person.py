#To handle ROS node
import rclpy
from rclpy.node import Node

from std_msgs.msg import Empty, String

#ROS Twist message import. This message type is used to send commands to the drone.
from geometry_msgs.msg import Twist


#Custom message containing the midpoint of the bounding box surrounding the tracked person
from person_tracking_msgs.msg import PersonTracked, PointMsg

#For image manipulation (OpenCV)
import cv2

import math

from person_tracking.pid import PID

from collections import deque

from math import pi

from threading import Thread

#Node base to be able to integrate our project to the Behaviour tree
from plugin_server_base.plugin_base import PluginBase, NodeState

##NB : all directions : left, right... are from the drone's perspective

class TrackPerson(PluginBase):

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

    #subscribers
    sub_person_tracked = None

    sub_key_pressed = None

    #publishers
    publisher_commands = None

    timer = None

    publisher_takeoff = None
       
    publisher_land = None
        

    def __init__(self,name):
        #Creating the Node
        super().__init__(name)
        
        #init topic names
        self._init_parameters()

        #init subscribers
        self._init_subscriptions()
        
        #init publishers
        self._init_publishers()

        self.pid_y_axis = PID(0.5,(0.1,0.1,0),(-0.3,0.3)) # Controller for y axis (horizontal position to keep the person within the field of view)
        self.pid_x_axis = PID(1,(0.1,0.1,0),(-0.3,0.3)) # Controller for x axis (distance between drone and target person).
        self.pid_z_axis = PID(0.5,(0.1,0.1,0),(-0.1,0.1)) # controller for z axis (altitude)

        self.person_tracked_midpoint = None

        self.bounding_box_size = None

        self.prev_midpoint = None 

        self.commands_msg = None

        #self.correction = None
        
        #Queue to keep the last nonempty midpoints. Help to calculate the trajectory of the person before he got lost
        self.midpoint_queue = deque(maxlen=self.max_length_midpoint_queue)
        
        self.empty_midpoint_count = 0 #variable to count the amount of empty messages received. If this number is higher than a certain number, the person is considered lost.
        
        #Counter for self.person_tracked_midpoint in the update message function.
        #If the midpoint is the same after a certain number of calls to the function, the connection might be broken. so we send empty command messages
        self.connection_lost_midpoint_unchanged_counter = 0  
        
        self.update_midpoint = False #is true when we receive a new midpoint that is different than (0,0)

        self.key_pressed = None #variable to contain key pressed on the Pygame GUI, either to land or takeoff

        self.flying = True#variable is True when the drone is flying and False if it landed

        self.rotating = False #variable to check if the drone is rotating

        self.tracking = False #Boolean variable. if it is True, we send velocity messages, if not, we do not send Twist messages on /cmd_vel.

        


    def _init_parameters(self)->None:
        """Method to initialize parameters such as ROS topics' names """

        #Topic names
        self.declare_parameter("key_pressed_topic",self.key_pressed_topic) 
        self.declare_parameter("person_tracked_topic",self.person_tracked_topic) 
        self.declare_parameter("commands_topic", self.commands_topic)
        self.declare_parameter("land_topic",self.land_topic)
        self.declare_parameter("takeoff_topic",self.takeoff_topic)
        self.declare_parameter("max_length_midpoint_queue",self.max_length_midpoint_queue)
        self.declare_parameter("max_empty_midpoint_before_lost",self.max_empty_midpoint_before_lost)

        self.key_pressed_topic= (
        self.get_parameter("key_pressed_topic").get_parameter_value().string_value
        )
        self.person_tracked_topic= (
        self.get_parameter("person_tracked_topic").get_parameter_value().string_value
        )
        self.commands_topic = (
        self.get_parameter("commands_topic").get_parameter_value().string_value
        )

        self.land_topic = (
        self.get_parameter("land_topic").get_parameter_value().string_value
        )

        self.takeoff_topic = (
        self.get_parameter("takeoff_topic").get_parameter_value().string_value
        ) 
        self.max_length_midpoint_queue = (
        self.get_parameter("max_length_midpoint_queue").get_parameter_value().integer_value
        )

        self.max_empty_midpoint_before_lost = (
        self.get_parameter("max_empty_midpoint_before_lost").get_parameter_value().integer_value
        ) 


           
    def _init_publishers(self)->None:
        """Method to initialize publishers"""
        
        #publishers
        self.publisher_commands = self.create_publisher(Twist,self.commands_topic,10)
        #self.timer = self.create_timer(0.05, self.commands_callback)


        self.publisher_takeoff = self.create_publisher(Empty,self.takeoff_topic,1)
       
        self.publisher_land = self.create_publisher(Empty,self.land_topic,1)
        

    def _init_subscriptions(self)->None:
        """Method to initialize subscriptions"""
        self.sub_person_tracked = self.create_subscription(PersonTracked,self.person_tracked_topic, self.listener_callback,5)

        self.sub_key_pressed = self.create_subscription(String,self.key_pressed_topic, self.key_pressed_subscriber_callback,5)
        
        
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

        elif self.stop_tracking_signal_midpoint(tmp):
            self.tracking = False

        else:
            self.tracking = True 
            self.update_midpoint = True
            self.person_tracked_midpoint = tmp
            self.bounding_box_size = math.sqrt((msg.bounding_box.top_left.x - msg.bounding_box.bottom_right.x)**2 +(msg.bounding_box.top_left.y - msg.bounding_box.bottom_right.y)**2)
            self.midpoint_queue.append(self.person_tracked_midpoint)
            self.empty_midpoint_count = 0
            self.get_logger().info(f'midpoint {self.person_tracked_midpoint}')
            #self.correction = self.pid.compute(self.person_tracked_midpoint)

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
        if self.tracking:
            self.land_takeoff()
            #if self.flying and not self.rotating:
            if not self.rotating:
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

            
            #if self.person_tracked_midpoint is not None: 
            #    correction = self.pid.compute(self.person_tracked_midpoint)
            #    correction_x = self.correction
               

            #    if self.person_tracked_midpoint.x < 0.3:#correction_x < -0.6 : #Here I don't put 0 to avoid having the drone always moving
            #        self.get_logger().info("move left") #(a in keyboard mode control station) 
            #        self.commands_msg.linear.y = 0.22
                     
            #    elif self.person_tracked_midpoint.x > 0.6:#correction_x > 0.6 :
            #        self.get_logger().info("move right") #(d in keyboard mode control station )             
            #        self.commands_msg.linear.y = -0.22
            
            #if self.bounding_box_size is not None:
            #    self.get_logger().info(f"Bounding box size : {self.bounding_box_size}") 
            #    if self.bounding_box_size < 0.5 and self.bounding_box_size > 0:
            #        self.get_logger().info("approach") 
            #        self.commands_msg.linear.x = 0.22

                #elif self.bounding_box_size > 1.:
                #    self.get_logger().info("move back")
                #    self.commands_msg.linear.x = -0.22
            
            if self.person_tracked_midpoint is not None:
                # controls horizontal position
                correction_y = self.pid_y_axis.compute(self.person_tracked_midpoint.x)
                self.commands_msg.linear.y = correction_y

                #logs
                if correction_y > 0:
                    self.get_logger().info("move left") #(a in keyboard mode control station) 
                elif correction_y < 0:
                    self.get_logger().info("move right") #(d in keyboard mode control station )  


                # controls altitude
                correction_z = self.pid_z_axis.compute(self.person_tracked_midpoint.y)
                self.commands_msg.linear.z = - correction_z # we take the opposite of correction_z because the vertical axis obeys computer vision's convention : it points down, not up.

                #logs
                if correction_z < 0:
                    self.get_logger().info("move up") 
                elif correction_z > 0:
                    self.get_logger().info("move down") 

            if self.bounding_box_size is not None:
                # controls distance between drone and target person
                correction_x = self.pid_x_axis.compute(self.bounding_box_size)

                #logs
                self.commands_msg.linear.x = correction_x
                if correction_x > 0:
                    self.get_logger().info("approach") 
                elif correction_x < 0:
                    self.get_logger().info("move back")

            
            self.publisher_commands.publish(self.commands_msg) 
        
        return None
            

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
            self.commands_msg.angular.z = angular_speed


            self.get_logger().info(f"Before rotating,  angular.z is {self.commands_msg.angular.z} and current_angle is {current_angle}")

            t0 = self.get_clock().now()

            while abs(current_angle) <= abs(target_angle): 
                #self.get_logger().info(f"Found a person? {self.update_midpoint}")
                if self.update_midpoint:
                    self.rotating = False
                    self.empty_midpoint_count = 0
                    self.get_logger().info(f"While rotating, found a person to track")
                    return 
                #self.publisher_commands.publish(self.commands_msg) 
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
        
    def stop_tracking_signal_midpoint(self,midpoint):
        """Function to test if the midpoint is (-1 ,-1). 
        Returns True if the midpoint is (-1,-1), and false else
        Precondition : midpoint is not None and is of type PointMsg"""
        if midpoint.x == -1 and midpoint.y == -1:
            return True
        else:
            return False  



    def land_takeoff(self)->None:
        """Prompts the drone to land or takeoff, depending on the messages received"""
        if self.key_pressed is not None:
            if self.key_pressed == "t":
                self.publisher_takeoff.publish(Empty())
                self.flying = True
                
            elif self.key_pressed == "l":
                self.publisher_land.publish(Empty())
                self.flying = False

            self.key_pressed = '' #To avoid taking off after the drone lands when it lost the person.
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
        
    def tick(self) -> NodeState:
        """This method is a mandatory for PluginBase node. It defines what we want our node to do.
        It gets called 20 times a second if state=RUNNING
        Here we call callback functions to publish a detection frame and the list of bounding boxes.
        """
        self.commands_callback()
        
        return NodeState.RUNNING




            
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


