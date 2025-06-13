import rclpy
from rclpy.node import Node
from hand_gestures_msgs.msg import Landmarks
from geometry_msgs.msg import Twist
from std_msgs.msg import Empty
from tello_msgs.msg import FlipControl
import random


class TelloSignInterpreterNode(Node):
    def __init__(self):
        super().__init__("tello_sign_interpreter")
        self.create_subscription(
            Landmarks, "/hand/landmarks_from_pilot", self.landmarks_callback, 10
        )

        self.takeoff_pub = self.create_publisher(Empty, "/takeoff", 1)
        self.land_pub = self.create_publisher(Empty, "/land", 1)
        self.flip_pub = self.create_publisher(FlipControl, "/flip", 1)
        self.cmd_vel_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.timer = self.create_timer(1 / 20, self.timer_callback)

        self.linear_speed = 0.3
        self.angular_speed = 0.4

        self.cmd_vel = Twist()

    def landmarks_callback(self, msg: Landmarks):
        num_hands = 0
        if msg.right_hand.handedness != "":
            num_hands += 1
        if msg.left_hand.handedness != "":
            num_hands += 1

        if num_hands == 1 and msg.right_hand.gesture == "ILoveYou":
            self.takeoff()

        if num_hands == 2:
            self.process_gestures(msg.right_hand.gesture, msg.left_hand.gesture)
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

    def stop(self):
        self.get_logger().debug("Stopping")
        self.cmd_vel.linear.x = 0.0
        self.cmd_vel.linear.y = 0.0
        self.cmd_vel.linear.z = 0.0
        self.cmd_vel.angular.z = 0.0


def main():
    rclpy.init()
    node = TelloSignInterpreterNode()
    rclpy.spin(node)
    node.land()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
