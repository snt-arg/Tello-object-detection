#To handle ROS node
import rclpy
from rclpy.node import Node

#ROS image message
from sensor_msgs.msg import Image

#To handle images
import cv2

#To connect to the drone
#from djitellopy import Tello, tello

#To convert cv2 images to ROS Image messages
from cv_bridge import CvBridge

#To handle images
import numpy as np

#Node base to be able to integrate our project to the Tello_ws
from plugin_server_base.plugin_base import PluginBase, NodeState

print("Arrived 1")



class Test(PluginBase):

    #Topic names
    image_raw_topic = "/camera/image_raw"
    def __init__(self,name):
        #Creating the Node
        super().__init__(name)
        self.get_logger().info("Arrived 2")
                #publishers
        #self.publisher_ = self.create_publisher(Image,image_raw,10)
        self.sub_raw = self.create_subscription(Image,self.image_raw_topic, self.listener_callback,5)
        self.cv_bridge = CvBridge()

        #Variable to contain the frame coming directly from the drone
        self.image_raw = None
        
        #self.video_path = "./walk.mp4"

        #self.cap = cv2.VideoCapture(self.video_path)

        #self.width  = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        #self.height = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        #self.fps    = self.cap.get(cv2.CAP_PROP_FPS)
        self.fps = 20
        self.width  = None
        self.height = None
        
#ret is a boolean value that we will use to verify if we successfully read a frame.
#We will also use it for the while loop
        self.ret = True

#encoding format of the output video. Here, mp4
        self.cv2_fourcc = cv2.VideoWriter_fourcc(*'mp4v')

        #creating a VideoWriter object to contain the output video where objects have been detetcted
        self.video = None 
        self.get_logger().info("Arrived 3")
######################## Publisher #####################################################################################  
    def listener_callback(self,img):
        self.image_raw = self.cv_bridge.imgmsg_to_cv2(img, 'bgr8')
        if self.height is None or self.width is None or self.video is None:
            self.height, self.width, _ = self.image_raw.shape
            self.video = cv2.VideoWriter('./outputnew.mp4',self.cv2_fourcc,self.fps,(self.width,self.height))
        self.video.write(self.image_raw)

        
    def tick(self) -> NodeState:
        """This method is a mandatory for PluginBase node. It defines what we want our node to do.
        It gets called 20 times a second if state=RUNNING
        """
        self.get_logger().info("Arrived 4")
        self.get_logger().info("Tick")

        #self.get_logger().info("Evaluate")

        #self.get_logger().info("Command")
        #self.listener_callback()
        #if self.cap.isOpened():
        #    self.get_logger().info("Arrived 5")
        #    success,frame = self.cap.read()
        #    if success:
        #        self.image = frame
        #        self.get_logger().info("Publishing frame")
        #        self.publisher_.publish(self.cv_bridge.cv2_to_imgmsg(self.image_raw, 'rgb8')) 
        #    else:
        #        self.cap = cv2.VideoCapture(self.video_path)
        #else:
        #    self.get_logger().info("Cap not opened.")
        #self.listener_callback()
        return NodeState.RUNNING

def main(args=None):
    #Intialization ROS communication 
    print("Arrived 6")

    rclpy.init(args=args)
    test = Test('test')

    test.get_logger().info("Arrived 7")
    #execute the callback function until the global executor is shutdown
    rclpy.spin(test)

    test.get_logger().info("Arrived 8")
    #destroy the node. It is not mandatory, since the garbage collection can do it
    test.destroy_node()
    
    test.get_logger().info("Arrived 9")
    rclpy.shutdown()        
