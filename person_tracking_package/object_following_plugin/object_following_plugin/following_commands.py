# for handling ROS node
import rclpy


# ROS Messages
from std_msgs.msg import Bool

# ROS Twist message import. This message type is used to send commands to the drone.
from geometry_msgs.msg import Twist

# custom message containing  the bounding box surrounding the tracked person
from person_tracking_msgs.msg import Box

from object_following_plugin.pid import PID
from object_following_plugin.mpc import MPC



from plugin_server_base.plugin_base import PluginBase, NodeState
from person_tracking_helpers.helpers import calculate_box_size, calculate_midpoint_box


##NB : all directions : left, right... are from the drone's perspective

class TrackPerson(PluginBase):

  
    person_tracked_topic = "/person_tracked" # carries the position of the tracked person (midpoint and box)
    commands_topic = "/following_vel" # carries Twist msgs
    tracking_status_topic = "/tracking_status"


    #subscribers
    sub_person_tracked = None

    sub_tracking_status = None

    #publishers
    publisher_commands = None

    # control method for autonomous tracking
    control_method = "PID"

    # on-off parameters
    lower_threshold_midpoint = None
    higher_threshold_midpoint = None

    lower_threshold_box_size = None
    higher_threshold_box_size = None
    
    # PID parameters
    pid_y_axis = None # Controller for y axis (horizontal position to keep the person within the field of view)
    pid_x_axis = None
    pid_z_axis = None

    # MDP parameters
    mpc_y_axis = None
    mpc_x_axis = None

    
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
        

        self.person_tracked_midpoint = None

        self.bounding_box_size = None


        self.commands_msg = None

        self.tracking = False

        
        

        
    def _init_parameters(self)->None:
        """Method to initialize parameters such as ROS topics' names """

        #Topic names
      
        self.declare_parameter("person_tracked_topic",self.person_tracked_topic) 
        self.declare_parameter("commands_topic", self.commands_topic)
        self.declare_parameter("tracking_status_topic",self.tracking_status_topic)
       
       
        self.declare_parameter("control_method", self.control_method)

        
        self.person_tracked_topic = (
        self.get_parameter("person_tracked_topic").get_parameter_value().string_value
        )
        self.commands_topic = (
        self.get_parameter("commands_topic").get_parameter_value().string_value
        )

        self.tracking_status_topic = (
        self.get_parameter("tracking_status_topic").get_parameter_value().string_value
        )


        self.control_method = (
        self.get_parameter("control_method").get_parameter_value().string_value
        ) 


           
    def _init_publishers(self)->None:
        """Method to initialize publishers"""
        
        #publishers
        self.publisher_commands = self.create_publisher(Twist,self.commands_topic,10)
    

        

    def _init_subscriptions(self)->None:
        """Method to initialize subscriptions"""
        self.sub_person_tracked = self.create_subscription(Box,self.person_tracked_topic, self.person_tracked_callback,5)

        self.sub_tracking_status = self.create_subscription(Bool,self.tracking_status_topic, self.tracking_status_callback,5)


    def _init_control(self)->None:
        """Method to initialize the control parameters, depending on the control method"""
        

        if self.control_method.lower() == "on/off":
            self.lower_threshold_midpoint = 0.3
            self.higher_threshold_midpoint = 0.7
    
            self.lower_threshold_box_size = 0.9
            self.higher_threshold_box_size = 1.2

        elif self.control_method.lower() == "pid":
            self.pid_y_axis = PID(0.5,(0.1,0.1,0),(-0.25,0.25)) # Controller for y axis (horizontal position to keep the person within the field of view)
            self.pid_x_axis = PID(1.25,(0.1,0.1,0),(-0.25,0.25)) # Controller for bounding box size (distance between the drone and the person)
            

        elif self.control_method.lower() == "mpc":
            self.mpc_x_axis = MPC(10, 0.5, 0.5)
            self.mpc_y_axis = MPC(10, 1.25, 1.25)

        else:
            raise ValueError("Unknown control method")
        
        
###########################first subscriber###########################################################################################   
    def person_tracked_callback(self, msg):
        """Callback function for the subscriber node (to topic /person_tracked).
        Receives the midpoint of the bounding box surrounding the person tracked"""
        self.get_logger().info("Pilot's position received")
        self.person_tracked_midpoint = calculate_midpoint_box(msg)
        self.bounding_box_size = calculate_box_size(msg)
        self.get_logger().info(f"Person tracked midpoint: {self.person_tracked_midpoint}")
        self.get_logger().info(f"Bounding box size: {self.bounding_box_size}")
       
######################### Publisher #####################################################################################################
    def commands_callback(self):
        """This function makes appropriate commands messages in order to keep the tracked person within the camera's field while ensuring safety"""
        self.commands_msg = Twist()
        self.commands_msg.linear.x = 0.0
        self.commands_msg.linear.y = 0.0
        self.commands_msg.linear.z = 0.0
        self.commands_msg.angular.x = 0.0
        self.commands_msg.angular.y = 0.0
        self.commands_msg.angular.z = 0.0

        
        if (self.person_tracked_midpoint is not None) and (self.bounding_box_size is not None) :

            self.commands_msg = self.control_commands()
            
            self.publisher_commands.publish(self.commands_msg)    
        
        
    def control_commands(self):
        """Function to determine the command to send to the drone given the current position of the person
        in the camera's field, the distance between the person and the drone, and the control method"""
        commands = Twist()
        if self.control_method.lower() == "on/off":
            
            # horizontal move
            if self.person_tracked_midpoint.x < self.lower_threshold_midpoint:
                self.get_logger().info("move left") #(a in keyboard mode control station) 
                commands.linear.y += 0.22
                    
            elif self.person_tracked_midpoint.x > self.higher_threshold_midpoint:
                self.get_logger().info("move right") #(d in keyboard mode control station )             
                commands.linear.y += -0.22

            # distance
            if self.bounding_box_size < self.lower_threshold_box_size:
                self.get_logger().info("approach") 
                commands.linear.x += 0.22

            elif self.bounding_box_size > self.higher_threshold_box_size:
                self.get_logger().info("move back")
                commands.linear.x += -0.22


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
    
    def tracking_status_callback(self, msg):
        """Method to receive the tracking status"""
        self.tracking = msg.data
        print(f"Tracking status : {self.tracking}")
   

    
###################################################################################################################################       
    def tick(self):
        """This method is a mandatory for PluginBase node. It defines what we want our node to do.
        It gets called 20 times a second if state=RUNNING
        Here we call callback functions to publish a detection frame and the list of bounding boxes.
        """
        if self.tracking:
            print("Tracking")
            self.commands_callback()
            return NodeState.RUNNING
        else:
            return NodeState.SUCCESS




def main(args=None):
    #Intialization ROS communication 
    rclpy.init(args=args)
    track_person = TrackPerson('person_tracker_node')

    #execute the callback function until the global executor is shutdown
    rclpy.spin(track_person)
    
    #track_person.video.release()

    #destroy the node. It is not mandatory, since the garbage collection can do it
    track_person.destroy_node()
    
    rclpy.shutdown()      

