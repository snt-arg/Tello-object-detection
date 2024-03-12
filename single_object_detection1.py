##This is a code to detect objects in an input video, and  output a video were objects are detected using yolov8

#import of libraries
from ultralytics import YOLO #for object detection
import cv2 #for video manipulation

#load the model
model = YOLO('yolov8n.pt')

#path to the input video
video_path = './skate_true.mp4'
#loading the video using cv2, in order to capture each frame afterwards
cap = cv2.VideoCapture(video_path)
#test to verify if the video has been opened properly
if not cap.isOpened(): 
    print("could not open :",video_path)

#length of the video   
#length = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))

#width , height and number of frames per second of a frame
width  = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
fps    = cap.get(cv2.CAP_PROP_FPS)

#print(length,width,height,fps)

#ret is a boolean value that we will use to verify if we successfully read a frame.
#We will alos use it for the while loop
ret = True

#encoding format of the output video. Here, mp4
cv2_fourcc = cv2.VideoWriter_fourcc(*'mp4v')

#creating a VideoWriter object to contain the output video where objects have been detetcted
video = cv2.VideoWriter('./skate_detected.mp4',cv2_fourcc,fps,(width,height))

#while loop to read each frame of the input video, detect objects, and write back the new frame in the output video
while ret:
    #read a frame in the input video
    ret, frame = cap.read()

    if ret:	
        results = model.track(frame, persist=True, classes=0)#classes=0 for person
        frame_ = results[0].plot()
        video.write(frame_)
        if cv2.waitKey(25) & 0xFF == ord('q'):
            break
    else:
        break
cap.release()
video.release()
cv2.destroyAllWindows()
