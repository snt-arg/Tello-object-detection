#To handle ROS node
import rclpy
from rclpy.node import Node

#ROS image message
from sensor_msgs.msg import Image

#Landmarks messages
from hand_gestures_msgs.msg import Landmarks, Landmark

#person tracked messages
from person_tracked.msg import PersonTracked

#bounding boxes messages
from all_bounding_boxes_msg.msg import AllBoundingBoxes

#To handle images
import cv2

#To convert cv2 images to ROS Image messages
from cv_bridge import CvBridge

#Node base to be able to integrate our project to the Tello_ws
#from plugin_server_base.plugin_base import PluginBase

from ultralytics import YOLO

#load the object detection model
model = YOLO('yolov8n.pt') 

#Filtering our classes of interest
classes = model.names
classes_needed = ["person"]
classes_ID = [k for k,v in classes.items() if v in classes_needed]

#Detection threshold probability
minimum_prob = 0.4  

right_hand_gesture = "Open_palm"

left_hand_gesture = "Open_palm"
class TriggerTracking(Node):

    #Topic names
    hand_landmarks_topic = "/hand/landmarks"
    person_tracked_topic = "/person_tracked"
    #image_raw= "/camera/image_raw"
    bounding_boxes_topic = "/all_bounding_boxes"


    #person_list_topic_name = "/persons_list"
    #detector_images_topic = "/image_person"
    right_hand_gesture = "Open_Palm"

    left_hand_gesture = "Open_Palm"
    
    def __init__(self,name):
        #Creating the Node
        super().__init__(name)
        
        #subscribers
        self.sub_bounding_boxes = self.create_subscription(AllBoundingBoxes,self.bounding_boxes_topic, self.bounding_boxes_listener_callback,10)
        self.sub_landmark = self.create_subscription(Landmarks,self.hand_landmarks_topic, self.landmarks_listener_callback,10)
        
        #publishers
        self.publisher_to_track= self.create_publisher(PersonTracked,self.person_tracked_topic,10)
        self.timer_1 = self.create_timer(0.1, self.person_tracked_callback)
        

        self.cv_bridge = CvBridge()

        #Variable to received bounding boxes containing all persons detected
        self.boxes = None

        #self.height = None

        #self.width = None
        
        #Variable to contain only the image of target person 
        #self.image_target = None

        #Variable to contain landmarks messages
        self.landmarks = None

        self.person_tracked_middlepoint = None

        self.person_tracked_left_hand_point = None

        self.person_tracked_right_hand_point = None

        self.fourcc = cv2.VideoWriter_fourcc(*'mp4v')
        self.video = cv2.VideoWriter('/media/maeri/UBUNTUUU/output670.mp4',self.fourcc,20.0,(640,480))


###########################first subscriber###########################################################################################   
    def bounding_boxes_listener_callback(self, boxes_msg):
        """Callback function for the subscriber node (to topic /all_detected).
        For each frame received, save in the log that an image has been received.
        Then convert that image into cv2 format and keep that frame in a variable for further processing"""
        
        self.get_logger().info('Bounding boxes received')
        self.boxes = boxes_msg
        print
       
        
        
########################### second subscriber #########################################################################################
    def landmarks_listener_callback(self, lndmrk):
        """Callback function for the subscriber node (to topic /hand/landmarks).
        Receives a landmark from the hand gesture plugin and saves that landmark in a variable for further processing."""
        
        self.get_logger().info('Landmark received')
        self.landmarks = lndmrk
        



######################### Publisher #####################################################################################################
    def person_tracked_callback(self):
        """This function listens to the /hand/landmarks topic, and waits to spot the person who did the triggering move. I
        n case a person did the trigger move, the function calls a publish the frames with only that person tracked. It sends the coordinates of the 
        center of the bounding boxes around that person to person_tracker node, so that the latter node sends commands to the drone to follow that person."""
        #self.publisher_all_detected.publish(self.cv_bridge.cv2_to_imgmsg(np.array(self.image_all_detected), 'rgb8')) 
        if self.person_tracked_middlepoint is None and self.landmarks is not None :
            if self.landmarks.right_hand.gesture == self.right_hand_gesture and self.landmarks.left_hand.gesture == self.left_hand_gesture:
                print("\n Open hand!! ")
                self.person_tracked_left_hand_point = self.landmarks.left_hand.normalized_landmarks[0]
                self.person_tracked_right_hand_point = self.landmarks.right_hand.normalized_landmarks[0]
                print("\n Beware",self.person_tracked_left_hand_point)
                print(self.person_tracked_right_hand_point,"\n\n")
                print("Bounding boxes",self.boxes.bounding_boxes,"\n\n")
                #self.video.write(cv2.line(self.image_all_detected,(int(middle_left.x*self.width),int(middle_left.y*self.height)),(int(middle_right.x*self.width),int(middle_right.y*self.height)),(255,0,0),4))
            
    
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

       
	    
    
###################################################################################################################################       
  


def main(args=None):
    #Intialization ROS communication 
    rclpy.init(args=args)
    trigger_tracking = TriggerTracking('trigger_tracking_node')

    #execute the callback function until the global executor is shutdown
    rclpy.spin(trigger_tracking)
    

    trigger_tracking.video.release()
    #destroy the node. It is not mandatory, since the garbage collection can do it
    trigger_tracking.destroy_node()
    
    rclpy.shutdown()        
