
#To handle ROS node
import rclpy
from rclpy.node import Node

#ROS image message
from sensor_msgs.msg import Image

#To handle images
#import cv2

#To connect to the drone
#from djitellopy import Tello, tello

#To convert cv2 images to ROS Image messages
from cv_bridge import CvBridge

#Node base to be able to integrate our project to the Tello_ws
from plugin_server_base.plugin_base import PluginBase

from ultralytics import YOLO

#load the object detection model
model = YOLO('yolov8n.pt') 

#Filtering our classes of interest
classes = model.names
classes_needed = ["person"]
classes_ID = [k for k,v in classes.items() if v in classes_needed]

#Detection threshold probability
minimum_prob = 0.4  

class TriggerTracking(PluginBase):

    #Topic names
    hand_landmark = "/hand/landmark"
    person_tracked = "/person/tracked"
    image_raw= "/camera/image_raw"
    all_detected_topic = "/person/all_detected"
    #person_list_topic_name = "/persons_list"
    #detector_images_topic = "/image_person"
    velocity_command_topic_name = "/cmd_vel"
    land_topic_name = "/land"
    takeoff_topic_name = "/takeoff" 
    
    def __init__(self,name):
        #Creating the Node
        super().__init__(name)
        
        #subscribers
        self.sub_raw = self.create_subscription(Image,image_raw, self.listener_callback,10)
        self.sub_landmark = self.create_subscription(Image,hand_landmark, self.spot_trigger,10)
        
        #publishers
        self.publisher_all_detected = self.create_publisher(Image,all_detected_topic,10)
        self.timer_1 = self.create_timer(0.1, self.all_detected_callback)
        
        self.publisher_person_tracked = self.create_publisher(Image,person_tracked,10)
        self.timer_2 = self.create_timer(0.1, self.person_tracked_callback)

        self.cv_bridge = CvBridge()

        #output video initialization
        #self.fourcc = cv2.VideoWriter_fourcc(*'MJPG')
        #self.video = cv2.VideoWriter('~/output.avi',self.fourcc,20.0,(640,480))

        #Variable to contain the frame coming directly from the drone
        self.image_raw = None

        #Variable to read each frame. It contains all persons detected
        self.image_all_detected = None
        
        #Variable to contain only the image of target person 
        self.image_target = None


###########################first subscriber###########################################################################################   
    def listener_callback(self, img):
        """Callback function for the subscriber node (to topic /camera/image_raw).
        For each image received, save in the log that an image has been received.
        Then convert that image into cv2 format, perform tracking on that image"""
        
        self.get_logger().info('I saw an image')
        self.image_raw = self.cv_bridge.imgmsg_to_cv2(img,'rgb8')
        self.image_all_detected = self.detection(image)
        ##########################################
        ##Add code to track a single person.
        #########################################
        #self.video.write(np.array(self.image))
        
    def detection(self,frame):
        """Function to perform object detection on frames"""
        results = model.track(frame, persist=True, classes=classes_ID, conf=minimum_prob)
        frame_ = results[0].plot()
        return frame_

###########################second subscriber#########################################################################################
    def track_target(self,frame,id_of_interest):
        """Function to track a person of a certain id only on a frame"""

        results = model.track(frame,persist=True) #stream = True
        index_of_target = 0
        target_in_frame = False
        for result in results[0]:
            if id_of_interest in result.boxes.id.tolist():
                #midpoint = result.boxes.xywh[0][0:2]
                results[0] = results[0][i]
                print(f"Tracking person ID {id_of_interest} at {result.boxes.xywh[0][0:2]}")
                frame_ = results[0].plot()
                target_in_frame = True
                index_of_target = index_of_target + 1
        if target_in_frame:
            return frame_
        else:
            return frame

    def spot_trigger_callback(self,frame):
        """This function listens to the /hand/landmarks topic, and waits to spot the person who did the triggering move. I
        n case a person did the trigger move, the function calls a publish the frames with only that person tracked. It sends the coordinates of the 
        center of the bounding boxes around that person to person_tracker node, so that the latter node sends commands to the drone to follow that person."""
        pass

########################first publisher#####################################################################################  
    
        
    def all_detected_callback(self):
        """
        callback funtion for the publisher node (to topis /camera/image_detected).
        The image on which object detection has been performed (self.image) is published on the topic '/camera/image_raw'
        """
        self.publisher_all_detected.publish(self.cv_bridge.cv2_to_imgmsg(np.array(self.image_all_detected), 'rgb8')) 
        
  
    
   
	    
    
###################################################################################################################################       
  


def main(args=None):
    #Intialization ROS communication 
    rclpy.init(args=args)
    trigger_tracking = TriggerTracking('trigger_tracking')

    #execute the callback function until the global executor is shutdown
    rclpy.spin(image_publisher)

    #destroy the node. It is not mandatory, since the garbage collection can do it
    image_publisher.destroy_node()
    
    rclpy.shutdown()        
