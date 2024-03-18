#this is module to apply object detection on a single class, applying filtration.

#import of libraries
from ultralytics import YOLO #for object detection
import cv2 #for video manipulation

#load the model
model = YOLO('yolov8n.pt')
#dictionnary containing the classes
classes = model.names

def filter_detection(classes_needed, video_path):
    """This function takes the a list of the desired classes to filtrate, and the path to the video,
       and performs a detection of only the objects from these classes in the video"""
#definition of the miniml probability required to detect an object    
    minimum_prob = 0.4     
#mapping class  names with their integer ID in the classes dictionary. We do that because the model.track method takes integer ID to know classes.
    classes_ID = [k for k,v in classes.items() if v in classes_needed]
   
#loading the video using cv2, in order to capture each frame afterwards

    cap = cv2.VideoCapture(video_path)

#test to verify if the video has been opened properly
    if not cap.isOpened():
        print("could not open :",video_path)
#length of the video

    length = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))

#width , height and number of frames per second of a frame

    width  = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    fps    = cap.get(cv2.CAP_PROP_FPS)

#ret is a boolean value that we will use to verify if we successfully read a frame.
#We will alos use it for the while loop
    ret = True

#encoding format of the output video. Here, mp4
    cv2_fourcc = cv2.VideoWriter_fourcc(*'mp4v')

#creating a VideoWriter object to contain the output video where objects have been detetcted
    video = cv2.VideoWriter('./skate_detected_only_person_and_cars.mp4',cv2_fourcc,fps,(width,height))

#while loop to read each frame of the input video, detect objects, and write back the new frame in the output video
    while ret:
       #read a frame in the input video
        ret, frame = cap.read()

        if ret:
            results = model.track(frame, persist=True, classes=classes_ID, conf=minimum_prob)
            frame_ = results[0].plot()
            video.write(frame_)
            if cv2.waitKey(25) & 0xFF == ord('q'):
                break
        else:
            break
    cap.release()
    video.release()
    cv2.destroyAllWindows()

filter_detection(["person","car"],"./skate_true.mp4")
