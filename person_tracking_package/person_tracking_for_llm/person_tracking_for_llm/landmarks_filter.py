import rclpy
from rclpy.node import Node
from hand_gestures_msgs.msg import Landmarks
from geometry_msgs.msg import Twist
from std_msgs.msg import Empty
from tello_msgs.msg import FlipControl
# custom message containing the midpoint of the bounding box surrounding the tracked person
from person_tracking_msgs.msg import PersonTracked, PointMsg, Box
from person_tracking_for_llm.helpers import extract_box_msg, extract_point_msg
import random



class LandmarkFilter(Node):

   
   
    person_tracked_topic = "/person_tracked" # carries the position of the tracked person (midpoint and box)
    landmarks_topic = "/hand/landmarks"
    landmarks_from_pilot_topic = "/hand/landmarks_from_pilot"

    # publisher
    landmarks_from_pilot_pub = None

    # subscriber
    sub_person_tracked = None
    sub_landmarks = None


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

        self.person_tracked_topic = (
        self.get_parameter("person_tracked_topic").get_parameter_value().string_value
        )
        self.landmarks_from_pilot_topic = (
        self.get_parameter("landmarks_from_pilot_topic").get_parameter_value().string_value
        )
        self.landmarks_topic = (
        self.get_parameter("landmarks_topic").get_parameter_value().string_value
        )
        
      


    def _init_publishers(self)->None:
        """Method to initialize publishers"""
        self.landmarks_from_pilot_pub = self.create_publisher(Landmarks, self.landmarks_from_pilot_topic, 10)
        
        
        

    def _init_subscriptions(self)->None:
        """Method to initialize subscriptions"""
        self.sub_person_tracked = self.create_subscription(PersonTracked,self.person_tracked_topic, self.listener_callback,5)
        self.sub_landmarks = self.create_subscription(Landmarks, self.landmarks_topic, self.landmarks_callback, 10)
        

    def listener_callback(self,msg)->None:
        """Receives the target person's midpoint and bounding box as a PersonTracked message"""
        print(f"\n\n received a message : {msg.midpoint}")
        if not(self.empty_midpoint(msg.midpoint)) and not(self.stop_tracking_signal_midpoint(msg.midpoint)):
            self.pilot_person = msg
            self.tracking = True

        elif self.stop_tracking_signal_midpoint(msg.midpoint):
            self.tracking = False
            self.get_logger().info("Stopped the tracking")
        elif self.empty_midpoint(msg.midpoint):
            self.get_logger().info(f"Empty midpoint received")
        else:
            self.get_logger().info("Else just in case")



    def landmarks_callback(self, msg: Landmarks):
        num_hands = 0
        if msg.right_hand.handedness != "":
            num_hands += 1
        if msg.left_hand.handedness != "":
            num_hands += 1

        if num_hands == 1 and msg.right_hand.gesture == "ILoveYou":
            self.landmarks_from_pilot_pub.publish(msg)

        if num_hands == 2:
            if self.tracking and self.pilot_person is not None:
                if self.is_gesture_from_pilot(msg):
                    print(f"\n\n\n\n\n\n##########################################\nPilot person did a gesture !! It is {msg.right_hand.gesture} and {msg.left_hand.gesture}")
                    self.landmarks_from_pilot_pub.publish(msg)
                    self.get_logger().info("Publishing the gestures from the pilot")
                else:
                    self.get_logger().info("Received some landmarks but they aren't from the pilot person. Not publishing the landmarks")

            elif not(self.tracking) and self.pilot_person is not None:
                self.landmarks_from_pilot_pub.publish(msg)
                self.get_logger().info("Publishing the landmarks")

            else:
                self.landmarks_from_pilot_pub.publish(msg)
                self.get_logger().info("Publishing the landmarks")

    

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
        
    def equal_point_msg(self, p1, p2)->bool:
        """function to compare to point messages (PointMsg).
        Returns True if they have the same coordinates, and False else"""
        if isinstance(p1, PointMsg) and isinstance(p2, PointMsg):
            return p1.x == p2.x and p1.y == p2.y
        else:
            raise TypeError("Error in function equal_point_msg, tried to compare two objects that are not of type PointMsg")
        
    def equal_box_msg(self, b1, b2)->bool:
        """function to compare to box messages (Box).
        Returns True if they have the same coordinates, and False else"""
        if isinstance(b1, Box) and isinstance(b2, Box):
            return self.equal_point_msg(b1.top_left, b2.top_left) and self.equal_point_msg(b1.bottom_right,b2.bottom_right)
        else:
            raise TypeError("Error in function equal_box_msg, tried to compare two objects that are not of type Box")
        
    def equal_personTracked_msg(self, pers1:PersonTracked, pers2:PersonTracked)->bool:
        """Compares two instance of PersonTracked. 
        Returns True if their midpoint and bounding boxes match"""
        if isinstance(pers1, PersonTracked) and isinstance(pers2, PersonTracked):
            return self.equal_point_msg(pers1.midpoint,pers2.midpoint) and self.equal_box_msg(pers1.bounding_box, pers2.bounding_box)
    
        else:
            raise TypeError("Error in function equal_personTracker_msg, tried to compare two objects that are not of type PersonTracked")
        
    def is_gesture_from_pilot(self, landmarks: Landmarks):
        """Fucntion to determine if the gesture was made by the pilot person the hands are in the box on the pilot.
        !!!Pre-protocol : self.tracking is True, and self.pilot_person is not None!!!
        """
        if self.tracking and self.pilot_person is not None:
            person_tracked_left_hand_point = landmarks.left_hand.normalized_landmarks[0]
            person_tracked_right_hand_point = landmarks.right_hand.normalized_landmarks[0]
            self.get_logger().info(f"{landmarks.right_hand.gesture} {landmarks.left_hand.gesture}")

            top_left_x , top_left_y, bottom_right_x, bottom_right_y = extract_box_msg(self.pilot_person.bounding_box)
            
            #Extracting the normalized coordinates of the left and right hand wrists' center points
            left_hand_x, left_hand_y = extract_point_msg(person_tracked_left_hand_point)
            right_hand_x, right_hand_y = extract_point_msg(person_tracked_right_hand_point)
            

            #If both wrist' center points are in the bounding box, we return True because it is the pilot person who did the gestures
            if top_left_x <= left_hand_x and top_left_x <= right_hand_x:
                if bottom_right_x >= left_hand_x and bottom_right_x >= right_hand_x:
                    if top_left_y <= left_hand_y and top_left_y <= right_hand_y:
                        if bottom_right_y >= left_hand_y and bottom_right_y >= right_hand_y:
                            return True

        elif not(self.tracking) and self.pilot_person is not None:
            self.get_logger().info(f"Can't map the landmarks to a person because we aren't tracking. All gestures are accepted")
            return True
        
        elif self.pilot_person is None:
            self.get_logger().info(f"Pilot person is undefined for some reason. All gestures are accepted")
            return True
        


def main():
    rclpy.init()
    node = LandmarkFilter("landmark_filter_for_pilot")
    rclpy.spin(node)
    node.land()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
