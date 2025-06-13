#To handle ROS node
import rclpy
from rclpy.node import Node

#----- ROS messages
from std_msgs.msg import String 
from sensor_msgs.msg import Image

from cv_bridge import CvBridge

    # pygame interface
from tello_control_station.interface import Interface



import pygame


matching_keys = {
    "0": pygame.K_0,
    "1": pygame.K_1,
    "2": pygame.K_2,
    "3": pygame.K_3,
    "4": pygame.K_4,
    "p": pygame.K_p,
    "j": pygame.K_j,
    "n": pygame.K_n,
    "k": pygame.K_k,
    "t": pygame.K_t,
    "l": pygame.K_l,
    "q": pygame.K_q,
    "e": pygame.K_e,
    "h": pygame.K_h,
    "f": pygame.K_f,
    "m": pygame.K_m,
    "w": pygame.K_w,
    "s": pygame.K_s,
    "a": pygame.K_a,
    "d": pygame.K_d,
    "up": pygame.K_UP,
    "down": pygame.K_DOWN,
    "left": pygame.K_LEFT,
    "right": pygame.K_RIGHT,
}
###################################################################################################################################       
class VideoInterface(Node):

    # topic names
    image_raw_topic = "/camera/image_raw" # topic for raw  frames from the drone's camera
    all_detected_topic = "/all_detected" # topic for image frames in which all persons (and specified objects) are detected
    drawing_person_tracked_topic = "/drawing_person_tracked" # topic for frames where the target person is outlined
    key_pressed_topic = "/key_pressed" # topic to send the key pressed on the pygame interface
    image_annotated_hands_topic = "/hand/annotated/image"
    drawing_person_tracked_and_hands_topic = "/drawing_person_tracked_hands"

    # display mode : {raw, all, target}
    modes : dict[int,str] = {0:"raw",1:"all", 2:"target", 3:"hands", 4:"target_hands"}
    default_display_mode = 2

    mode = modes[default_display_mode]

    #ROS Subscriptions
    sub_raw = None
    sub_all_detected = None
    sub_person_tracked = None
    sub_annotated_hands = None
    sub_person_tracked_and_hands = None

    #ROS Publishers
    publisher_key_pressed = None
    

    
    def __init__(self,name):

        # creating the Node
        super().__init__(name)

        # initializing parameters
        self._init_parameters()

        # initializing subscribers
        self._init_subscriptions()

        #init publishers
        self._init_publishers()
        

        # variable to contain raw images
        self.image_raw = None

        # variable to contain frames on which all persons (and potentially objects) are detected
        self.image_all_detected = None

        # variable to contain frames on which only the target person is outlined
        self.image_person_tracked = None

        self.image_annotated_hands = None

        self.image_person_tracked_and_hands = None

        #Pygame interface
        self.pg_interface = Interface()

        #to convert cv2 images to Ros Image messages and vice versa
        self.cv_bridge = CvBridge()

        # timer to update the pygame interface
        self.timer_1 = self.create_timer(0.01,self.update_interface)

        # timer to publish key pressed
        self.timer_2 = None
        
    def _init_parameters(self)->None:
        """Method to initialize parameters such as ROS topics' names """

        self.declare_parameter("image_raw_topic",self.image_raw_topic) 
        self.declare_parameter("all_detected_topic", self.all_detected_topic)
        self.declare_parameter("drawing_person_tracked_topic",self.drawing_person_tracked_topic)
        self.declare_parameter("image_annotated_hands_topic", self.image_annotated_hands_topic) 
        self.declare_parameter("drawing_person_tracked_and_hands_topic", self.drawing_person_tracked_and_hands_topic)
        self.declare_parameter("key_pressed_topic",self.key_pressed_topic)
        self.declare_parameter("default_display_mode",self.default_display_mode)

        self.image_raw_topic = (
        self.get_parameter("image_raw_topic").get_parameter_value().string_value
        )
        self.all_detected_topic = (
        self.get_parameter("all_detected_topic").get_parameter_value().string_value
        )
        self.drawing_person_tracked_topic = (
        self.get_parameter("drawing_person_tracked_topic").get_parameter_value().string_value
        )
        self.image_annotated_hands_topic  = (
        self.get_parameter("image_annotated_hands_topic").get_parameter_value().string_value
        )
        self.drawing_person_tracked_and_hands_topic  = (
        self.get_parameter("drawing_person_tracked_and_hands_topic").get_parameter_value().string_value
        )

        self.key_pressed_topic = (
        self.get_parameter("key_pressed_topic").get_parameter_value().string_value
        )
        self.default_display_mode = (
        self.get_parameter("default_display_mode").get_parameter_value().integer_value
        )



    def _init_subscriptions(self)->None:
        """Method to initialize the subscriptions"""
        self.sub_raw = self.create_subscription(Image,self.image_raw_topic, self.image_raw_listener_callback,5)
        self.sub_all_detected = self.create_subscription(Image,self.all_detected_topic, self.all_detected_listener_callback,5)
        self.sub_person_tracked = self.create_subscription(Image,self.drawing_person_tracked_topic, self.person_tracked_listener_callback,5)
        self.sub_annotated_hands = self.create_subscription(Image,self.image_annotated_hands_topic, self.annotated_hands_callback,5)
        self.sub_person_tracked_and_hands = self.create_subscription(Image,self.drawing_person_tracked_and_hands_topic, self.person_tracked_and_hands_callback,5)

    
    def _init_publishers(self)->None:
        """Method to initialize the publisher"""
        self.publisher_key_pressed = self.create_publisher(String,self.key_pressed_topic,5)
        self.timer_2 = self.create_timer(0.01,self.key_pressed_callback)
      

########################### First Subscriber ##################################################################################################   
    def image_raw_listener_callback(self, img_msg):
        """Callback function for the subscriber node (to topic /all_detected_topic).
        For each bounding box received, save it in a variable for processing"""

        self.get_logger().debug('\nRaw frame received')
        self.image_raw = img_msg
           
       
     
######################### Second subscriber ####################################################################################################
    def all_detected_listener_callback(self, img_msg):
        """Callback function for the subscriber node (to topic /drawing_person_tracked_topic).
        For each bounding box received, save it in a variable for processing"""

        self.get_logger().debug('\nAll detected frame received')
        self.image_all_detected = img_msg
        

######################### Third subscriber ####################################################################################################
    def person_tracked_listener_callback(self, img_msg):
        """Callback function for the subscriber node (to topic /image_raw_topic).
        For each bounding box received, save it in a variable for processing"""

        self.get_logger().debug('\nTracked person frame received')
        self.image_person_tracked = img_msg
        

    def annotated_hands_callback(self,img_msg):
        self.get_logger().debug('\nAnnotated hands frame received')
        self.image_annotated_hands = img_msg

    def person_tracked_and_hands_callback(self,img_msg):
        self.get_logger().debug('\nTracked person and annotated hands frame received')
        self.image_person_tracked_and_hands = img_msg

######################### Publisher ####################################################################################################
    def key_pressed_callback(self)->None:
        """Callback function to capture and send keys that are pressed on the pygame interface"""
        keys = self.pg_interface.get_key_pressed()
        msg = String()
        # takeoff/land
        if keys[matching_keys["t"]]:
            msg.data = "t"
            self.publisher_key_pressed.publish(msg)
            return 
        
        if keys[matching_keys["l"]]:
            msg.data = "l"
            self.publisher_key_pressed.publish(msg)
            return
        
        if keys[matching_keys["0"]]:
            msg.data = "0"
            self.publisher_key_pressed.publish(msg)

            display_mode = 0
            self.mode = self.modes[display_mode]
            self.get_logger().info(f'#####\nDisplay mode updated to :\n{self.mode}')
            return
        
        if keys[matching_keys["1"]]:
            msg.data = "1"
            self.publisher_key_pressed.publish(msg)

            display_mode = 1
            self.mode = self.modes[display_mode]
            self.get_logger().info(f'#####\nDisplay mode updated to :\n{self.mode}')
            return
        
        if keys[matching_keys["2"]]:
            msg.data = "2"
            self.publisher_key_pressed.publish(msg)

            display_mode = 2
            self.mode = self.modes[display_mode]
            self.get_logger().info(f'#####\nDisplay mode updated to :\n{self.mode}')
            return
        
        if keys[matching_keys["3"]]:
            msg.data = "3"
            self.publisher_key_pressed.publish(msg)

            display_mode = 3
            self.mode = self.modes[display_mode]
            self.get_logger().info(f'#####\nDisplay mode updated to :\n{self.mode}')
            return
        
        if keys[matching_keys["4"]]:
            msg.data = "4"
            self.publisher_key_pressed.publish(msg)

            display_mode = 4
            self.mode = self.modes[display_mode]
            self.get_logger().info(f'#####\nDisplay mode updated to :\n{self.mode}')
            return
        

    def update_interface(self):
        new_frame = None
        raw_mode = self.modes.get(0,"raw")
        all_mode = self.modes.get(1,"all")
        target_mode = self.modes.get(2,"target")
        hands_mode = self.modes.get(3,"hands")
        target_hands_mode = self.modes.get(4,"target_hands")

        
        if self.mode == raw_mode:
            new_frame = self.image_raw

        elif self.mode == all_mode:
            new_frame = self.image_all_detected

        elif self.mode == target_mode:
            new_frame = self.image_person_tracked

        elif self.mode == hands_mode:
            new_frame = self.image_annotated_hands

        elif self.mode == target_hands_mode:
            new_frame = self.image_person_tracked_and_hands

        else:
            self.get_logger().info('!!!!!!!!!!!\nUnknown mode!!!\n\nDisplaying target person frames.\n')
            new_frame = self.image_person_tracked

        if(new_frame is not None):
            self.pg_interface.update_bg_image(new_frame) # updating the image on the pygame console
            
        else:
            self.get_logger().debug('!!!!!!!!!!!\nDidn\'t receive frames for the desired mode\n')
        self.pg_interface.tick()



#############################################################################################################################################################

def main(args=None):
    #Initialization ROS communication 
    rclpy.init(args=args)

    #Node instantiation
    video_interface = VideoInterface('Video_interface_node')

    #Execute the callback function until the global executor is shutdown
    rclpy.spin(video_interface)
    
    #destroy the node. It is not mandatory, since the garbage collection can do it
    video_interface.destroy_node()
    
    rclpy.shutdown()        
