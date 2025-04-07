from person_tracking_msgs.msg import Box, PointMsg
import math

def yolo_box_to_Box_msg(yolo_box):
    """Function to convert a yolo box (in list format) to a Box message"""
    box_msg = Box()
    box_msg.top_left.x = yolo_box[0]
    box_msg.top_left.y = yolo_box[1]
    box_msg.bottom_right.x = yolo_box[2]
    box_msg.bottom_right.y = yolo_box[3]

    return box_msg

def extract_box_msg(box_msg):
    """Given a Box message (defined in person_tracking_msgs),
    this function returns a tuple (top_left_x, top_left_y, bottom_right_x, bottom_right_y)
    This function also works for any object possessing top_left.x, top_left.y, bottom_right.x , bott"""

    top_left_x = box_msg.top_left.x
    top_left_y = box_msg.top_left.y
    bottom_right_x = box_msg.bottom_right.x
    bottom_right_y = box_msg.bottom_right.y

    return (top_left_x, top_left_y, bottom_right_x, bottom_right_y)


def extract_point_msg(point_msg):
    """Given a Box message (defined in person_tracking_msgs),
    this function returns a tuple (x, y)"""
    if  hasattr(point_msg, 'x') and hasattr(point_msg, 'y'):
        x = point_msg.x
        y = point_msg.y
        return (x,y)
    else:
        raise TypeError("The provided point is not an instance of PointMsg")


def euclidean_distance_squared(x1,y1,x2,y2):
    """Calculates the eucliedean distance between two points (x1,y1) and (x2,y2)"""
    return (x2-x1)**2 + (y2-y1)**2

def calculate_midpoint(p1,p2):
    """Function to calculate the midpoint of two points, (x1,y1) and (x2,y2)"""
    x1,y1 = extract_point_msg(p1)
    x2,y2 = extract_point_msg(p2)
    return ((x1 + x2) / 2 , (y1 + y2) / 2)

def calculate_box_size(box):
    """Calculates the size of a box. The size is the length of the diagonal.
     This function takes a custom Box message"""
    top_left_x, top_left_y, bottom_right_x, bottom_right_y = extract_box_msg(box)

    return math.sqrt((top_left_x- bottom_right_x)**2 +(top_left_y- bottom_right_y)**2)

def person_lost(empty_midpoint_count, max_empty_midpoint_before_lost):
    """Function to call when someone is lost.
    Returns true when the person is lost and False else."""  
    if empty_midpoint_count >= max_empty_midpoint_before_lost:
        return True
    else: 
        return False  
    
def equal_point_msg(p1, p2)->bool:
    """function to compare to point messages (PointMsg).
    Returns True if they have the same coordinates, and False else"""
    if isinstance(p1, PointMsg) and isinstance(p2, PointMsg):
        return p1.x == p2.x and p1.y == p2.y
    else:
        raise TypeError("Error in function equal_point_msg, tried to compare two objects that are not of type PointMsg")


