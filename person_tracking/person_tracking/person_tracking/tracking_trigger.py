#To handle ROS node
import rclpy
from rclpy.node import Node

#ROS image message
from sensor_msgs.msg import Image

#Landmarks messages
from hand_gestures_msgs.msg import Landmarks, Landmark

#person tracked messages
from person_tracked.msg import PersonTracked, PointMsg

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
        
        self.test_sub = self.create_subscription(Image,"/all_detected",self.test_listener,10)

        #publishers
        self.publisher_to_track= self.create_publisher(PersonTracked,self.person_tracked_topic,10)
        self.timer_1 = self.create_timer(0.1, self.person_tracked_callback)
        

        self.cv_bridge = CvBridge()

        #Variable to received bounding boxes containing all persons detected
        self.boxes = None

        #Variable to contain landmarks messages
        self.landmarks = None

        self.person_tracked_middlepoint = None

        self.person_tracked_left_hand_point = None

        self.person_tracked_right_hand_point = None

        self.person_tracked_msg = PersonTracked()
         
        self.image_height = None

        self.image_width = None

        self.fourcc = cv2.VideoWriter_fourcc(*'mp4v')
        self.video = cv2.VideoWriter('/media/maeri/UBUNTUUU/output1118.mp4',self.fourcc,20.0,(640,480))

################################ test subscriber #####################################################################################
    def test_listener(self,frame_msg):
        """To delete , just a test listener callback to verify the midpoint is really the midpoint"""
        frame = self.cv_bridge.imgmsg_to_cv2(frame_msg,'bgr8')
        self.image_height, self.image_width, _ = frame.shape
        if self.person_tracked_middlepoint is not None:
            self.video.write(cv2.circle(frame,(int(self.person_tracked_msg.middle_point.x),int(self.person_tracked_msg.middle_point.y)),10,(255,0,0),-1))

###########################first subscriber###########################################################################################   
    def bounding_boxes_listener_callback(self, boxes_msg):
        """Callback function for the subscriber node (to topic /all_detected).
        For each frame received, save in the log that an image has been received.
        Then convert that image into cv2 format and keep that frame in a variable for further processing"""
        
        self.get_logger().info('Bounding boxes received')
        self.boxes = boxes_msg
        
        
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

        if self.person_tracked_middlepoint is None and self.landmarks is not None :
            if self.landmarks.right_hand.gesture == self.right_hand_gesture and self.landmarks.left_hand.gesture == self.left_hand_gesture:
                self.get_logger().info("\n Open hand!! ")

                self.person_tracked_left_hand_point = self.landmarks.left_hand.normalized_landmarks[0]
                self.person_tracked_right_hand_point = self.landmarks.right_hand.normalized_landmarks[0]
                
                self.person_tracked_middlepoint = PointMsg()

                #print("\n Beware",self.person_tracked_left_hand_point)
                #print(self.person_tracked_right_hand_point,"\n\n")

                if self.boxes is None:
                    self.get_logger().info("No bounding box received")
                else:
                    print("Bounding boxes",self.boxes.bounding_boxes,"\n\n")
                #self.video.write(cv2.line(self.image_all_detected,(int(middle_left.x*self.width),int(middle_left.y*self.height)),(int(middle_right.x*self.width),int(middle_right.y*self.height)),(255,0,0),4))
                self.find_bounding_box_of_tracked_person(self.person_tracked_left_hand_point,self.person_tracked_right_hand_point,self.boxes)
                
                self.person_tracked_msg.middle_point = self.denormalize()
                
                if self.person_tracked_middlepoint is not None:
                    self.publisher_to_track.publish(self.person_tracked_msg) 

        elif self.person_tracked_middlepoint is not None:
            self.find_bounding_box_middlepoint()
            self.person_tracked_msg.middle_point = self.denormalize()
            self.publisher_to_track.publish(self.person_tracked_msg) 
            print("\nNow we know the person to track. midpoint is ", self.person_tracked_middlepoint,"\n")

    
    def denormalize(self)->PointMsg():
        """Function to denormalize the corrdinates of the midpoint"""
        result = PointMsg()
        result.x = self.person_tracked_middlepoint.x * self.image_width
        result.y = self.person_tracked_middlepoint.y * self.image_height
        return result

    def find_bounding_box_of_tracked_person(self, left_hand, right_hand, boxes):
        """Finds the bounding box around the person who did the triggering move"""
        for box in boxes.bounding_boxes:
            top_left_x = box.top_left.x
            top_left_y = box.top_left.y
            bottom_right_x = box.bottom_right.x
            bottom_right_y = box.bottom_right.y

            left_hand_x = left_hand.x
            left_hand_y = left_hand.y
            right_hand_x = right_hand.x
            right_hand_y = right_hand.y

            if top_left_x <= left_hand_x and top_left_x <= right_hand_x:
                if bottom_right_x >= left_hand_x and bottom_right_x >= right_hand_x:
                    if top_left_y <= left_hand_y and top_left_y <= right_hand_y:
                        if bottom_right_y >= left_hand_y and bottom_right_y >= right_hand_y:
                            self.person_tracked_middlepoint.x = top_left_x/ 2 + bottom_right_x/2 
                            self.person_tracked_middlepoint.y = top_left_y/2 + bottom_right_y/2
        #print("No bounding box found")
        #return None
    
    def find_bounding_box_middlepoint(self):
        """Find the nearest bounding box containing the middlepoint and upadates the middlepoint"""
        smallest_error_margin = -1
        for box in self.boxes.bounding_boxes:
            top_left_x = box.top_left.x
            top_left_y = box.top_left.y
            bottom_right_x = box.bottom_right.x
            bottom_right_y = box.bottom_right.y

            midpoint_x = self.person_tracked_middlepoint.x
            midpoint_y = self.person_tracked_middlepoint.y

            if top_left_x <= midpoint_x and top_left_y <= midpoint_y and bottom_right_x >= midpoint_x and bottom_right_y >= midpoint_y:
                new_midpoint_x = top_left_x/ 2 + bottom_right_x/2
                new_midpoint_y = top_left_y/2 + bottom_right_y/2
                error_margin = self.euclidean_distance(midpoint_x,midpoint_y,new_midpoint_x,new_midpoint_y)
                
                if (smallest_error_margin == -1) or (error_margin < smallest_error_margin):
                    self.person_tracked_middlepoint.x = new_midpoint_x
                    self.person_tracked_middlepoint.y = new_midpoint_y
                    smallest_error_margin = error_margin
                


    def euclidean_distance(self,x1,y1,x2,y2):
        """Calculates the eucliedean distance between two points (x1,y1) and (x2,y2)"""
        return (x2-x1)**2 + (y2-y1)**2

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
