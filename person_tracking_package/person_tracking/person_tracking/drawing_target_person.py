#To handle ROS node
import rclpy
from rclpy.node import Node

#ROS image message
from sensor_msgs.msg import Image

#person tracked messages
from person_tracking_msgs.msg import PersonTracked

#To convert cv2 images to ROS Image messages
from cv_bridge import CvBridge

#To draw rectangles
import cv2 

class DrawTarget(Node):

    #Topic names
    image_raw_topic = "/camera/image_raw"
    person_tracked_topic = "/person_tracked" 
    drawing_person_tracked_topic = "/drawing_person_tracked"

    #subscribers
    sub_image_raw = None
    sub_person_tracked = None

    #publishers
    publisher_drawing = None
    timer_1 = None
    
    def __init__(self,name):

        #Creating the Node
        super().__init__(name)

        #init topic names
        self._init_parameters()

        #init subscribers
        self._init_subscriptions()
        
        #init publishers
        self._init_publishers()
        
        #Used to convert cv2 frames into ROS Image messages and vice versa
        self.cv_bridge = CvBridge()

        #Variable to contain the coordinates of the person tracked (midpoint and coordinates of the bounding box)
        self.person_tracked_position = None

        #Variable to contain the image received from the drone
        self.image = None

        #Dimensions of the image
        self.image_height = None
        self.image_width = None

         
        
    def _init_parameters(self)->None:
        """Method to initialize parameters such as ROS topics' names """

        self.declare_parameter("image_raw_topic",self.image_raw_topic) 
        self.declare_parameter("person_tracked_topic",self.person_tracked_topic) 
        self.declare_parameter("drawing_person_tracked_topic", self.drawing_person_tracked_topic)

        self.image_raw_topic = (
        self.get_parameter("image_raw_topic").get_parameter_value().string_value
        )

        self.person_tracked_topic= (
        self.get_parameter("person_tracked_topic").get_parameter_value().string_value
        )

        self.drawing_person_tracked_topic  = (
        self.get_parameter("drawing_person_tracked_topic").get_parameter_value().string_value
        )

           
    def _init_publishers(self)->None:
        """Method to initialize publishers"""
        self.publisher_drawing = self.create_publisher(Image,self.drawing_person_tracked_topic,10)
        self.timer_1 = self.create_timer(0.05,self.drawing_person_tracked_callback)
        

    def _init_subscriptions(self)->None:
        """Method to initialize subscriptions"""
        self.sub_image_raw = self.create_subscription(Image,self.image_raw_topic,self.image_raw_listener_callback,5)
        self.sub_person_tracked = self.create_subscription(PersonTracked, self.person_tracked_topic, self.person_tracked_listener_callback,5)

########################### First Subscriber ###########################################################################################  
# 
    def image_raw_listener_callback(self, img_msg):
        """Callback function for the subscriber node (to topic /camera/image_raw).
        For each image frame, save it in a variable for processing"""
        self.get_logger().info('Raw images received')
        self.image = self.cv_bridge.imgmsg_to_cv2(img_msg ,'rgb8')

        #Dimensions of the image. Useful to denormalise bounding box coordinates
        if self.image_height is None or self.image_width is None:
            self.image_height, self.image_width, _ = self.image.shape

    def person_tracked_listener_callback(self, msg):
        """Callback function for the subscriber node (to topic /person_tracked).
        For each message received, save it in a variable for processing
        The messages contain the midpoint and coordinates of the bounding box around the target person
        midpoint : msg.middle_point
        coordinates : msg.top_left and msg.bottom_right"""
        self.get_logger().info("Target person's position received")
        self.person_tracked_position = msg
        
######################### Publisher #####################################################################################################
    def drawing_person_tracked_callback(self):
        """This methods is the callback functio for the publisher of images where ONLY the target person is highlighted."""
        image_drawn = self.draw_rectangle(self.image)
        if image_drawn is not None:
            self.publisher_drawing.publish(self.cv_bridge.cv2_to_imgmsg(image_drawn,'rgb8'))

    def draw_rectangle(self,raw_image):
        if raw_image is not None and self.person_tracked_position is not None:
            midpoint = self.person_tracked_position.middle_point
            if midpoint.x != 0 or midpoint.y != 0:
                bounding_box = self.person_tracked_position.bounding_box
                top_left_point = (round(bounding_box.top_left.x * self.image_width),round(bounding_box.top_left.y * self.image_height))
                bottom_right_point = (round(bounding_box.bottom_right.x * self.image_width),round(bounding_box.bottom_right.y * self.image_height))
                cv2.rectangle(raw_image,top_left_point,bottom_right_point,(86, 237, 81),2)

                circle_center = (round(midpoint.x * self.image_width),round(midpoint.y * self.image_height))
                cv2.circle(raw_image,circle_center,20,(166, 237, 164),-1)
                cv2.circle(raw_image,circle_center,15,(118, 237, 114),-1)
                cv2.circle(raw_image,circle_center,10,(86, 237, 81),-1)
            return raw_image
        return None

           
###################################################################################################################################       
  


def main(args=None):
    #Initialization ROS communication 
    rclpy.init(args=args)

    #Node instantiation
    draw_target = DrawTarget('drawing_target_person_node')

    #Execute the callback function until the global executor is shutdown
    rclpy.spin(draw_target)

    #destroy the node. It is not mandatory, since the garbage collection can do it
    draw_target.destroy_node()
    
    rclpy.shutdown()        
