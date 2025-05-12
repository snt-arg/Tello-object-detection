# for handling ROS node
import rclpy


# ROS Twist message import. This message type is used to send commands to the drone.
from geometry_msgs.msg import Twist


from math import pi

from person_tracking_helpers.helpers import calculate_box_size, person_lost, equal_point_msg
from plugin_server_base.plugin_base import PluginBase, NodeState

##NB : all directions : left, right... are from the drone's perspective

class RotateTello(PluginBase):

    
    #publishers
    publisher_commands = None

    rotation_speed = pi/7

    rotation_angle = pi/7 # complete rotation

    rotation_direction = "left" # or right

    commands_topic = "/cmd_vel"

    
    def __init__(self,name):
        #Creating the Node
        super().__init__(name)

        #init topic names
        self._init_parameters()

        
        #init publishers
        self._init_publishers()


        self.commands_msg = None

        self.rotation_complete = False

        
    def _init_parameters(self)->None:
        """Method to initialize parameters such as ROS topics' names """

        #Topic names
        self.declare_parameter("rotation_speed",self.rotation_speed) 
        self.declare_parameter("rotation_angle",self.rotation_angle) 
        self.declare_parameter("rotation_direction",self.rotation_direction) 
        self.declare_parameter("commands_topic",self.commands_topic) 
        

        self.rotation_speed = (
        self.get_parameter("rotation_speed").get_parameter_value().double_value
        )

        self.rotation_angle = (
        self.get_parameter("rotation_angle").get_parameter_value().double_value
        )

        self.rotation_direction = (
        self.get_parameter("rotation_direction").get_parameter_value().string_value
        )
        
        self.commands_topic = (
        self.get_parameter("commands_topic").get_parameter_value().string_value
        )

           
    def _init_publishers(self)->None:
        """Method to initialize publishers"""
        
        #publishers
        self.publisher_commands = self.create_publisher(Twist,self.commands_topic,10)
        
                      
######################### Publisher #####################################################################################################
    def commands_callback(self):
        self.commands_msg = Twist()
        self.rotation(self.rotation_speed, self.rotation_angle)
        self.rotation_complete = True
                      
 
    def rotation(self, angular_speed, target_angle)->None:
        """Function to send rotation commands to the drone. 
        Returns True if the drone did a complete rotation (no one was found) and False else"""
        if self.commands_msg is not None:
            current_angle = 0
            self.commands_msg.angular.z = angular_speed


            self.get_logger().info(f"Before rotating,  angular.z is {self.commands_msg.angular.z} and current_angle is {current_angle}")

            t0 = self.get_clock().now()

            while abs(current_angle) <= abs(target_angle): 
                #self.get_logger().info(f"Found a person? {self.update_midpoint}")
                
                self.publisher_commands.publish(self.commands_msg) 
                t1 = self.get_clock().now()

                current_angle = self.commands_msg.angular.z * ((t1-t0).to_msg().sec)

            self.get_logger().info(f"After rotating ,angular.z is {self.commands_msg.angular.z} and current_angle is {current_angle}")
    
    def tick(self):
        """This method is a mandatory for PluginBase node. It defines what we want our node to do.
        It gets called 20 times a second if state=RUNNING
        Here we call callback functions to publish a detection frame and the list of bounding boxes.
        """
        if not self.rotation_complete:
            self.commands_callback()
            return NodeState.RUNNING
        else:
            return NodeState.SUCESS


    
###################################################################################################################################       
  


def main(args=None):
    #Intialization ROS communication 
    rclpy.init(args=args)
    rotate_tello = RotateTello('Rotate_Tello_node')

    #execute the callback function until the global executor is shutdown
    rclpy.spin(rotate_tello)
    
    #track_person.video.release()

    #destroy the node. It is not mandatory, since the garbage collection can do it
    rotate_tello.destroy_node()
    
    rclpy.shutdown()      

