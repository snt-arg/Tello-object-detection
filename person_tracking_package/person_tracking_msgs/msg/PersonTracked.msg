# Middle of the box around the person to track (between 0 and 1)
person_tracking_msgs/PointMsg middle_point    

#coordinates of the bounding box containing the target person
person_tracking_msgs/Box bounding_box

# Normalized value of the diagonal between the top left and bottom right corner of the bounding box around the target individual
#float32 bounding_box_size    
           
#all_bounding_boxes_msg/PointMsg left_hand_point   #point at the middle of the fist of the left hand
#all_bounding_boxes_msg/PointMsg right_hand_point  #point at the middle of the fist of the right hand
#float32 radius                        #Error margin around the middlepoint

