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



class PersonTrackingSignInterpreter(Node):

    # topics
    flip_topic = "/flip"  # carries key pressed on pygame interface
    person_tracked_topic = "/person_tracked" # carries the position of the tracked person (midpoint and box)
    commands_topic = "/cmd_vel" #carries Twist msgs
    land_topic = "/land" #carries Empty msgs
    takeoff_topic = "/takeoff" # carries Empty msgs 

    # publishers
    takeoff_pub = None
    land_pub = None
    flip_pub = None
    cmd_vel_pub = None

    # subscriber
    sub_person_tracked = None
    sub_landmarks = None

    # Other parameters
    publishing_rate = 0.1
    linear_speed = 0.3
    angular_speed = 0.4

    def __init__(self,name):
        super().__init__(name)

        #init topic names
        self._init_parameters()

        #init subscribers
        self._init_subscriptions()
        
        #init publishers
        self._init_publishers()
    

        self.cmd_vel = Twist()

        # is true if someone is being tracked, and false otherwise
        self.tracking = False

        # contains the position of the target person, along with his/her bounding box
        self.pilot_person = None

        # boolean variable. Is True when we receive a different position of the pilot_person
        self.updated = False

        self.stopped = False


    def _init_parameters(self)->None:
        """Method to initialize parameters such as ROS topics' names """

        #Topic names
        self.declare_parameter("flip_topic",self.flip_topic) 
        self.declare_parameter("person_tracked_topic",self.person_tracked_topic) 
        self.declare_parameter("commands_topic", self.commands_topic)
        self.declare_parameter("land_topic",self.land_topic)
        self.declare_parameter("takeoff_topic",self.takeoff_topic)
        
        self.declare_parameter("publishing_rate", self.publishing_rate)
        self.declare_parameter("linear_speed", self.linear_speed)
        self.declare_parameter("angular_speed", self.angular_speed)

        self.flip_topic = (
        self.get_parameter("flip_topic").get_parameter_value().string_value
        )
        self.person_tracked_topic = (
        self.get_parameter("person_tracked_topic").get_parameter_value().string_value
        )
        self.commands_topic = (
        self.get_parameter("commands_topic").get_parameter_value().string_value
        )
        self.land_topic = (
        self.get_parameter("land_topic").get_parameter_value().string_value
        )
        self.takeoff_topic = (
        self.get_parameter("takeoff_topic").get_parameter_value().string_value
        ) 
        self.publishing_rate = (
        self.get_parameter("publishing_rate").get_parameter_value().double_value
        )
        self.linear_speed = (
        self.get_parameter("linear_speed").get_parameter_value().double_value
        ) 
        self.angular_speed = (
        self.get_parameter("angular_speed").get_parameter_value().double_value
        ) 


    def _init_publishers(self)->None:
        """Method to initialize publishers"""
        
        #publishers
        self.takeoff_pub = self.create_publisher(Empty, self.takeoff_topic, 1)
        self.land_pub = self.create_publisher(Empty, self.land_topic, 1)
        self.flip_pub = self.create_publisher(FlipControl, self.flip_topic, 1)
        self.cmd_vel_pub = self.create_publisher(Twist, self.commands_topic, 10)

        self.timer = self.create_timer(self.publishing_rate, self.timer_callback)
        
        

    def _init_subscriptions(self)->None:
        """Method to initialize subscriptions"""
        self.sub_person_tracked = self.create_subscription(PersonTracked,self.person_tracked_topic, self.listener_callback,5)
        self.sub_landmarks = self.create_subscription(Landmarks, "/hand/landmarks", self.landmarks_callback, 10)
        

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
            self.takeoff()
            self.stopped = False

        if num_hands == 2:
            if self.is_gesture_from_pilot(msg):
                print(f"\n\n\n\n\n\n##########################################\nPilot person did a gesture !! It is {msg.right_hand.gesture} and {msg.left_hand.gesture}")
                self.process_gestures(msg.right_hand.gesture, msg.left_hand.gesture)
                self.stopped = False
            else:
                self.get_logger().info("Received some landmarks but they aren't from the pilot person.")
        else:
            self.stop()

    def process_gestures(self, right_gesture: str, left_gesture: str):
        if right_gesture == "None" or left_gesture == "None":
            self.stop()
        # Forward/backward
        if left_gesture == "Open_Palm":
            if right_gesture == "Pointing_Up":
                self.move_forward()
            elif right_gesture == "Victory":
                self.move_backward()

        # Left/right
        if left_gesture == "Pointing_Up" and right_gesture == "Closed_Fist":
            self.move_right()
        elif left_gesture == "Closed_Fist" and right_gesture == "Pointing_Up":
            self.move_left()

        # Up/Down
        if left_gesture == "Thumb_Up" and right_gesture == "Thumb_Up":
            self.move_up()
        elif left_gesture == "Thumb_Down" and right_gesture == "Thumb_Down":
            self.move_down()

        # Rotation left/right
        if left_gesture == "Thumb_Up" and right_gesture == "Closed_Fist":
            self.rotate_right()
        elif left_gesture == "Closed_Fist" and right_gesture == "Thumb_Up":
            self.rotate_left()

        if left_gesture == "Open_Palm" and right_gesture == "Thumb_Down":
            self.land()

        if left_gesture == "ILoveYou" and right_gesture == "ILoveYou":
            self.flip()

    def timer_callback(self):
        if not(self.stopped):
            self.cmd_vel_pub.publish(self.cmd_vel)

    def move_forward(self):
        self.get_logger().debug("Moving forward")
        self.cmd_vel.linear.x = self.linear_speed

    def move_backward(self):
        self.get_logger().debug("Moving backward")
        self.cmd_vel.linear.x = -self.linear_speed

    def move_left(self):
        self.get_logger().debug("Moving left")
        self.cmd_vel.linear.y = self.linear_speed

    def move_right(self):
        self.get_logger().debug("Moving right")
        self.cmd_vel.linear.y = -self.linear_speed

    def move_up(self):
        self.get_logger().debug("Moving up")
        self.cmd_vel.linear.z = self.linear_speed + 0.2

    def move_down(self):
        self.get_logger().debug("Moving down")
        self.cmd_vel.linear.z = -(self.linear_speed + 0.2)

    def rotate_left(self):
        self.get_logger().debug("Rotating left")
        self.cmd_vel.angular.z = self.angular_speed

    def rotate_right(self):
        self.get_logger().debug("Rotating right")
        self.cmd_vel.angular.z = -self.angular_speed

    def takeoff(self):
        self.get_logger().debug("Standing up")
        self.takeoff_pub.publish(Empty())

    def land(self):
        self.get_logger().debug("Sitting down")
        self.land_pub.publish(Empty())

    def flip(self):
        msg = FlipControl()

        choice = random.choice(["left", "right", "back"])

        if choice == "left":
            msg.flip_left = True
        elif choice == "right":
            msg.flip_right = True
        elif choice == "forward":
            msg.flip_forward = True
        elif choice == "back":
            msg.flip_backward = True

        self.flip_pub.publish(msg)

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
        """Fucntion to determine if the gesture was made by the pilot person the hands are in the box on the pilot."""
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

        elif not(self.tracking):
            self.get_logger().info(f"Can't map the landmarks to a person because we aren't tracking")
            return False
        elif self.pilot_person is None:
            self.get_logger().info(f"Pilot person is undefined for some reason")
            return False
        else:
            return False
            




    def stop(self):
        self.get_logger().debug("Stopping")
        
        self.cmd_vel.linear.x = 0.0
        self.cmd_vel.linear.y = 0.0
        self.cmd_vel.linear.z = 0.0
        self.cmd_vel.angular.z = 0.0
        
        self.stopped = True

def main():
    rclpy.init()
    node = PersonTrackingSignInterpreter("Sign_interpreter_for_llm")
    rclpy.spin(node)
    node.land()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
