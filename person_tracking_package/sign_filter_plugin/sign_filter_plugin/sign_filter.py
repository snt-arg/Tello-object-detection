import rclpy
from hand_gestures_msgs.msg import Landmarks
from typing import Optional, Any
from std_msgs.msg import String,Bool 
import json

from plugin_base.plugin_base import PluginNode, NodeState
# custom message containing the midpoint of the bounding box surrounding the tracked person
from person_tracking_msgs.msg import  Box
from person_tracking_helpers.helpers import extract_box_msg, extract_point_msg, convert_Landmarks_to_dict


class LandmarkFilter(PluginNode):
   
    person_tracked_topic = "/person_tracked" # carries the position of the tracked person (midpoint and box)
    landmarks_topic = "/hand/landmarks"
    landmarks_from_pilot_topic = "/hand/landmarks_from_pilot"
    hand_tracking_signal_topic = "/tracking_signal_gesture"
    tracking_status_topic = "/tracking_status"

    # trigger gestures 
    right_hand_gesture_trigger = "Open_Palm"
    left_hand_gesture_trigger = "Open_Palm"

    right_hand_gesture_stop= "Closed_Fist"
    left_hand_gesture_stop = "Closed_Fist"

    # publisher
    publisher_landmarks_from_pilot = None
    publisher_tracking_signal = None

    # subscriber
    sub_person_tracked = None
    sub_landmarks = None
    sub_tracking_status = None


    def __init__(self,name):
        super().__init__(name)

        #init topic names
        self._init_parameters()

        #init subscribers
        self._init_subscriptions()
        
        #init publishers
        self._init_publishers()
    

        # is true if someone is being tracked, and false otherwise
        self.tracking = False

        # contains the position of the target person, along with his/her bounding box
        self.pilot_person = None



    def _init_parameters(self)->None:
        """Method to initialize parameters such as ROS topics' names """

        #Topic names
        self.declare_parameter("person_tracked_topic",self.person_tracked_topic) 
        self.declare_parameter("landmarks_from_pilot_topic", self.landmarks_from_pilot_topic)
        self.declare_parameter("landmarks_topic",self.landmarks_topic)
        self.declare_parameter("hand_tracking_signal_topic",self.hand_tracking_signal_topic)
        self.declare_parameter("tracking_status_topic",self.tracking_status_topic)
        self.declare_parameter("right_hand_gesture_trigger",self.right_hand_gesture_trigger)
        self.declare_parameter("left_hand_gesture_trigger",self.left_hand_gesture_trigger)
        self.declare_parameter("right_hand_gesture_stop",self.right_hand_gesture_stop)
        self.declare_parameter("left_hand_gesture_stop",self.left_hand_gesture_stop)


        self.person_tracked_topic = (
        self.get_parameter("person_tracked_topic").get_parameter_value().string_value
        )
        self.landmarks_from_pilot_topic = (
        self.get_parameter("landmarks_from_pilot_topic").get_parameter_value().string_value
        )
        self.landmarks_topic = (
        self.get_parameter("landmarks_topic").get_parameter_value().string_value
        )

        self.hand_tracking_signal_topic = (
        self.get_parameter("hand_tracking_signal_topic").get_parameter_value().string_value
        )
        
        self.tracking_status_topic = (
        self.get_parameter("tracking_status_topic").get_parameter_value().string_value
        )

        self.right_hand_gesture_trigger = (
        self.get_parameter("right_hand_gesture_trigger").get_parameter_value().string_value
        )

        self.left_hand_gesture_trigger = (
        self.get_parameter("left_hand_gesture_trigger").get_parameter_value().string_value
        ) 
        self.right_hand_gesture_stop = (
        self.get_parameter("right_hand_gesture_stop").get_parameter_value().string_value
        )

        self.left_hand_gesture_stop = (
        self.get_parameter("left_hand_gesture_stop").get_parameter_value().string_value
        ) 
      


    def _init_publishers(self)->None:
        """Method to initialize publishers"""
        self.publisher_landmarks_from_pilot = self.create_publisher(Landmarks, self.landmarks_from_pilot_topic, 10)
        self.publisher_tracking_signal = self.create_publisher(String, self.hand_tracking_signal_topic, 10)
        
        
        

    def _init_subscriptions(self)->None:
        """Method to initialize subscriptions"""
        self.sub_person_tracked = self.create_subscription(Box,self.person_tracked_topic, self.person_tracked_callback,5)
        self.sub_landmarks = self.create_subscription(Landmarks, self.landmarks_topic, self.landmarks_callback, 10)
        self.sub_tracking_status = self.create_subscription(Bool, self.tracking_status_topic, self.tracking_status_callback,10)
        

    def person_tracked_callback(self,msg)->None:
        """Receives the target person's bounding box, class and id as a Box message"""
        #print(f"\n\n received a message : {msg}") #for debugging
        self.pilot_person = msg

    

    def landmarks_callback(self, msg: Landmarks):
        num_hands = 0
        if msg.right_hand.handedness != "":
            num_hands += 1
        if msg.left_hand.handedness != "":
            num_hands += 1

        if num_hands == 1 and msg.right_hand.gesture == "ILoveYou":
            self.publisher_landmarks_from_pilot.publish(msg)

        if num_hands == 2:
            if self.tracking and self.pilot_person is not None:
                if self.check_gesture(msg,False):
                    infodict = convert_Landmarks_to_dict(msg)
                    infodict["action"] = "stop_tracking"
                    self.publisher_tracking_signal.publish(json.dumps(infodict))

                if self.is_gesture_from_pilot(msg):
                    self.get_logger().info(f"\n\n\n\n\n\n##########################################\nPilot person did a gesture !! It is {msg.right_hand.gesture} and {msg.left_hand.gesture}")
                    self.publisher_landmarks_from_pilot.publish(msg)
                    self.get_logger().debug("Publishing the gestures from the pilot")
                else:
                    self.get_logger().info("Received some landmarks but they aren't from the pilot person. Not publishing the landmarks")

            elif self.tracking and self.pilot_person is None:
                self.get_logger().error("For some reason, we are tracking, but the pilot's position is not known")

            else: # here, self.tracking == False
                self.publisher_landmarks_from_pilot.publish(msg)
                self.get_logger().debug("Publishing the landmarks as is")
                if self.check_gesture(msg, True):
                    infodict = convert_Landmarks_to_dict(msg)
                    infodict["action"] = "tracking"
                    tracking_signal_msg = String()
                    tracking_signal_msg.data = json.dumps(infodict)
                    self.publisher_tracking_signal.publish(tracking_signal_msg)
     
    def tracking_status_callback(self, msg):
        """Method to receive the tracking status"""
        self.tracking = msg.data
   
        
   
    def is_gesture_from_pilot(self, landmarks: Landmarks):
        """Function to determine if the gesture was made by the pilot person the hands are in the box on the pilot.
        !!!Pre-protocol : self.tracking is True, and self.pilot_person is not None!!!
        """
        
        person_tracked_left_hand_point = landmarks.left_hand.normalized_landmarks[0]
        person_tracked_right_hand_point = landmarks.right_hand.normalized_landmarks[0]
        self.get_logger().debug(f"{landmarks.right_hand.gesture} {landmarks.left_hand.gesture}")

        top_left_x , top_left_y, bottom_right_x, bottom_right_y, _, _ = extract_box_msg(self.pilot_person)
        
        #Extracting the normalized coordinates of the left and right hand wrists' center points
        left_hand_x, left_hand_y = extract_point_msg(person_tracked_left_hand_point)
        right_hand_x, right_hand_y = extract_point_msg(person_tracked_right_hand_point)
        

        #If both wrist' center points are in the bounding box, we return True because it is the pilot person who did the gestures
        if top_left_x <= left_hand_x and top_left_x <= right_hand_x:
            if bottom_right_x >= left_hand_x and bottom_right_x >= right_hand_x:
                if top_left_y <= left_hand_y and top_left_y <= right_hand_y:
                    if bottom_right_y >= left_hand_y and bottom_right_y >= right_hand_y:
                        return True

        
    def check_gesture(self,landmarks:Landmarks, trigger:bool):
        """Function used to check if the trigger gesture was done by someone.
        Returns True if someone did the gesture and False if not
        Parameter trigger is used to specify whether the function is used to spot the trigger move (trigger == True)
        Or the gesture prompting to stop the tracking (trigger == False)"""

        if trigger and landmarks is not None and landmarks.right_hand.gesture == self.right_hand_gesture_trigger and landmarks.left_hand.gesture == self.left_hand_gesture_trigger:
            return True
        elif not trigger and landmarks is not None and landmarks.right_hand.gesture == self.right_hand_gesture_stop and landmarks.left_hand.gesture == self.left_hand_gesture_stop:
            return True
        else:
            return False

    def tick(self,blackboard: Optional[dict["str", Any]] = None) -> NodeState:
        """This method is a mandatory for PluginBase node. It defines what we want our node to do.
        It gets called 20 times a second if state=RUNNING
        Here we call callback functions to publish a detection frame and the list of bounding boxes.
        """
        #print("\n\n Sign filter Ticked \n\n Sign filter Ticked \n\n Sign filter Ticked \n\n")
        
        return NodeState.SUCCESS

def main():
    rclpy.init()
    node = LandmarkFilter("sign_filter_node")
    rclpy.spin(node)
    node.land()
    rclpy.shutdown()


if __name__ == "__main__":
    main()