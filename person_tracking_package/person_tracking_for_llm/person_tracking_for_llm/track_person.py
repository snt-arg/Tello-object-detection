# for handling ROS node
import rclpy
from rclpy.node import Node

# ROS messages
from std_msgs.msg import Empty, String

# ROS Twist message import. This message type is used to send commands to the drone.
from geometry_msgs.msg import Twist

# custom message containing the midpoint of the bounding box surrounding the tracked person
from person_tracking_msgs.msg import PersonTracked, PointMsg

from math import pi

from person_tracking_for_llm.pid import PID
from person_tracking_for_llm.mpc import MPC

from collections import deque

from threading import Thread

from person_tracking_for_llm.helpers import calculate_box_size, person_lost, equal_point_msg



##NB : all directions : left, right... are from the drone's perspective

class TrackPersonLLM(Node):

    key_pressed_topic = "/key_pressed"  # carries key pressed on pygame interface
    person_tracked_topic = "/person_tracked" # carries the position of the tracked person (midpoint and box)
    commands_topic = "/cmd_vel" #carries Twist msgs
    land_topic = "/land" #carries Empty msgs
    takeoff_topic = "/takeoff" # carries Empty msgs 


    max_length_midpoint_queue = 2

    max_empty_midpoint_before_lost = 100

    publishing_rate = 0.07 #14 hertz

    #subscribers
    sub_person_tracked = None

    sub_key_pressed = None

    #publishers
    publisher_commands = None

    timer = None

    publisher_takeoff = None
       
    publisher_land = None

    # control method for autonomous tracking
    control_method = "on/off"

    # on-off parameters
    
    # PID parameters
    pid_y_axis = None # Controller for y axis (horizontal position to keep the person within the field of view)
    pid_x_axis = None
    pid_z_axis = None

    # MDP parameters 
    mpc_y_axis = None
    mpc_x_axis = None
    
    lower_threshold_midpoint = 0.3
    higher_threshold_midpoint = 0.7
    lower_threshold_box_size = 0.75
    higher_threshold_box_size = 1

    def __init__(self,name):
        #Creating the Node
        super().__init__(name)
        
        #init topic names
        self._init_parameters()

        #init subscribers
        self._init_subscriptions()
        
        #init publishers
        self._init_publishers()

        #init control method parameters
        self._init_control()

        #self.pid_y_axis = PID(0.5,(0.1,0.1,0),(-0.3,0.3)) # Controller for y axis (horizontal position to keep the person within the field of view)
        #self.pid_x_axis = PID(1,(0.1,0.1,0),(-0.3,0.3)) # Controller for x axis (distance between drone and target person).

        self.person_tracked_midpoint = None

        self.bounding_box_size = None

        self.prev_midpoint = None 

        self.commands_msg = None

        
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
        self.declare_parameter("publishing_rate", self.publishing_rate)
        self.declare_parameter("control_method", self.control_method)

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

        self.publishing_rate = (
        self.get_parameter("publishing_rate").get_parameter_value().double_value
        )

        self.control_method = (
        self.get_parameter("control_method").get_parameter_value().string_value
        ) 


           
    def _init_publishers(self)->None:
        """Method to initialize publishers"""
        
        #publishers
        self.publisher_commands = self.create_publisher(Twist,self.commands_topic,10)
        self.timer = self.create_timer(self.publishing_rate, self.commands_callback)

        self.publisher_takeoff = self.create_publisher(Empty,self.takeoff_topic,1)
       
        self.publisher_land = self.create_publisher(Empty,self.land_topic,1)
        

    def _init_subscriptions(self)->None:
        """Method to initialize subscriptions"""
        self.sub_person_tracked = self.create_subscription(PersonTracked,self.person_tracked_topic, self.listener_callback,5)

        self.sub_key_pressed = self.create_subscription(String,self.key_pressed_topic, self.key_pressed_subscriber_callback,5)
        

    def _init_control(self)->None:
        """Method to initialize the control parameters, depending on the control method"""

        if self.control_method.lower() == "on/off":
            pass

        elif self.control_method.lower() == "pid":
            self.pid_y_axis = PID(0.5,(0.1,0.1,0),(-0.25,0.25)) # Controller for y axis (horizontal position to keep the person within the field of view)
            self.pid_x_axis = PID(0.9,(0.1,0.1,0),(-0.25,0.25)) # Controller for bounding box size (distance between the drone and the person)
            

        elif self.control_method.lower() == "mpc":
            self.mpc_x_axis = MPC(10, 0.5, 0.5)
            self.mpc_y_axis = MPC(10, 1.25, 1.25)

        else:
            raise ValueError("Unknown control method")
        
###########################first subscriber###########################################################################################   
    def listener_callback(self, msg):
        """Callback function for the subscriber node (to topic /person_tracked).
        Receives the midpoint of the bounding box surrounding the person tracked"""
        self.get_logger().info('Midpoint received')
        tmp = msg.midpoint

        if self.empty_midpoint(tmp):
            self.empty_midpoint_count += 1
            self.get_logger().info("Empty midpoint")
            self.update_midpoint = False

        elif self.stop_tracking_signal_midpoint(tmp):
            self.tracking = False
            # Reset parameters and PID ...

        else:
            self.tracking = True 
            self.update_midpoint = True
            self.person_tracked_midpoint = tmp
            self.bounding_box_size = calculate_box_size(msg.bounding_box)
            if self.prev_midpoint is not None and not equal_point_msg(self.person_tracked_midpoint, self.prev_midpoint):
                self.midpoint_queue.append(self.person_tracked_midpoint)
                
            self.empty_midpoint_count = 0
            self.get_logger().info(f'midpoint {self.person_tracked_midpoint}')

    def direction_person_lost(self)->str|None:
        """Function to determine whether the drone should rotate left, right, up or down to find the lost person"""
        if len(self.midpoint_queue) == self.max_length_midpoint_queue:
            point_1 = self.midpoint_queue[1] #most recent midpoint
            point_2 = self.midpoint_queue[0] #previous midpoint
            
            """
            slope = ( point_2.y - point_1.y ) / ( point_2.x - point_1.x ) # zero division risks
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
            if point_1.x > point_2.x:
                return "right"
                
            
            elif point_1.x < point_2.x:
                return "left"
            


        else:
            self.get_logger().info(f"\nNot enough midpoints received yet to predict the person's position\n")
        
        return None


############################ Second subscriber#######################################################################3333
            
    def key_pressed_subscriber_callback(self,msg):
        self.key_pressed = msg.data
                      
########################### takeoff/land publishers #################################################################
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
        
######################### Publisher #####################################################################################################
    def commands_callback(self):
        self.land_takeoff()
        if self.tracking:
            if self.flying and not self.rotating:
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
        


        self.get_logger().info(f"\nself.person_lost :{person_lost(self.empty_midpoint_count, self.max_empty_midpoint_before_lost)}\n")
        self.get_logger().info(f"\nself.empty_midpoint_count :{self.empty_midpoint_count}\n")

        
        if person_lost(self.empty_midpoint_count, self.max_empty_midpoint_before_lost):

            direction : str = self.direction_person_lost()

            if direction == "left":
                self.get_logger().info("\n*************************\nRotate left\n*************************") 
                
                rotation_thread_left = Thread(target=self.rotation,args=(pi/7,2*pi))
                rotation_thread_left.start()
                
            
            elif direction == "right":
                self.get_logger().info("\n*************************\nRotate right\n*************************")
                
                rotation_thread_right = Thread(target=self.rotation,args=(-pi/7,2*pi))
                rotation_thread_right.start()  
                

            else:
                self.get_logger().info(f'No trajectory can be found')

        else:
            #check if the connection is still passing (meaning self.person_tracked_midpoint and hence self.empty_midpoint_count are updated in the listener callback)
            if not self.check_midpoint_changed():
                self.publisher_commands.publish(self.commands_msg) 
                return None

            
            if (self.person_tracked_midpoint is not None) and (self.bounding_box_size is not None) and self.update_midpoint:
                self.commands_msg = self.control_commands()
                self.publisher_commands.publish(self.commands_msg) 
        
        return None
              
 
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
                if self.update_midpoint or self.tracking == False:
                    self.rotating = False
                    self.empty_midpoint_count = 0
                    self.get_logger().info(f"While rotating, found a person to track")
                    return None
                
                self.publisher_commands.publish(self.commands_msg) 
                t1 = self.get_clock().now()

                current_angle = self.commands_msg.angular.z * ((t1-t0).to_msg().sec)

            self.get_logger().info(f"After rotating ,angular.z is {self.commands_msg.angular.z} and current_angle is {current_angle}")
            self.rotating = False
            self.publisher_land.publish(Empty()) #land if we found no one after a complete rotation
            self.get_logger().info(f"\n********************\nLanding the drone\n********************")
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
        A midpoint equal to (-1,-1) signifies that the tracking stopped but the node is still running
        Returns True if the midpoint is (-1,-1), and false else
        Precondition : midpoint is not None and is of type PointMsg"""
        if midpoint.x == -1 and midpoint.y == -1:
            return True
        else:
            return False  


    
    def check_midpoint_changed(self)->bool:
        """Function to check if the self.person_tracked_midpoint changed. Returns True if it stayed the same for less than max_empty_midpoint_before_lost, and False if it stayed the same for more.
        """
        if self.prev_midpoint is None and self.person_tracked_midpoint is not None:
            self.prev_midpoint = self.person_tracked_midpoint
            return True

        elif self.prev_midpoint is not None :
            if equal_point_msg(self.prev_midpoint, self.person_tracked_midpoint):
                self.connection_lost_midpoint_unchanged_counter += 1
            else:
                self.connection_lost_midpoint_unchanged_counter = 0
                self.prev_midpoint = self.person_tracked_midpoint
        

        if self.connection_lost_midpoint_unchanged_counter <= self.max_empty_midpoint_before_lost :
            return True
        else: 
            return False
        
    def control_commands(self):
        """Function to determine the command to send to the drone given the current position of the person
        in the camera's field, the distance between the person and the drone, and the control method"""
        commands = Twist()
        if self.control_method.lower() == "on/off":
            
            # horizontal move
            if self.person_tracked_midpoint.x < self.lower_threshold_midpoint:
                self.get_logger().info("move left") #(a in keyboard mode control station) 
                commands.linear.y += 0.35
                    
            elif self.person_tracked_midpoint.x > self.higher_threshold_midpoint:
                self.get_logger().info("move right") #(d in keyboard mode control station )             
                commands.linear.y -= 0.35

            # distance
            if self.bounding_box_size < self.lower_threshold_box_size:
                self.get_logger().info("approach") 
                commands.linear.x += 0.45

            elif self.bounding_box_size > self.higher_threshold_box_size:
                self.get_logger().info("move back")
                commands.linear.x -= 0.45


        elif self.control_method.lower() == "pid":

            # horizontal move
            correction_y = self.pid_y_axis.compute(self.person_tracked_midpoint.x)
            commands.linear.y = correction_y
    
            if correction_y > 0:
                self.get_logger().info("move left") #(a in keyboard mode control station) 
                
            elif correction_y < 0:
                self.get_logger().info("move right") #(d in keyboard mode control station )  

            # distance
            correction_x = self.pid_x_axis.compute(self.bounding_box_size)
            commands.linear.x = correction_x

            if correction_x > 0:
                self.get_logger().info("approach") 
            elif correction_x < 0:
                self.get_logger().info("move back")


        elif self.control_method.lower() == "mpc":
            #horizontal
            correction_y =  self.mpc_y_axis.solve_mpc(self.person_tracked_midpoint.x) 
            commands.linear.y = correction_y

            if correction_y > 0:
                self.get_logger().info("move left") #(a in keyboard mode control station) 
                
            elif correction_y < 0:
                self.get_logger().info("move right") #(d in keyboard mode control station )  
        

            # distance
            correction_x = self.mpc_x_axis.solve_mpc(self.bounding_box_size)
            commands.linear.x = correction_x

            if correction_x > 0:
                self.get_logger().info("approach") 
            elif correction_x < 0:
                self.get_logger().info("move back")
            

        else:
            raise ValueError("Unknown control method")
        
        return commands

            
###################################################################################################################################       
  


def main(args=None):
    #Intialization ROS communication 
    rclpy.init(args=args)
    track_person = TrackPersonLLM('person_tracker_llm_node')

    #execute the callback function until the global executor is shutdown
    rclpy.spin(track_person)
    
    #track_person.video.release()

    #destroy the node. It is not mandatory, since the garbage collection can do it
    track_person.destroy_node()
    
    rclpy.shutdown()      


