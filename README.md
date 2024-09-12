# 👩‍💻📸 Object detection and Person tracking with Tello drone

This repository contains two ROS packages. The first one, [tello](https://github.com/snt-arg/Tello-object-detection/tree/main/ros_workspace/tello), performs object detection using a Tello drone's camera by leveraging YOLOv8, a popular framework for object detection, object classification, and more, developed by [Ultralytics](https://github.com/ultralytics). The second one [person_tracking](https://github.com/snt-arg/Tello-object-detection/tree/main/person_tracking) implements drone person tracking. To run the second package, the [tello ros driver](https://github.com/snt-arg/tello_ros_driver) and [hand gesture detector](https://github.com/snt-arg/hand_gestures_plugin) plugins are required.

## 🔍 Table Of Contents

- [🛠 Installation](#installation)
- [🧑‍💻️🏃 Run code](#run)
- [👨🏻‍💻📝Implementation](#implementation)
- [🧾 License](#license)

## 🛠️ Installation <a id="installation"></a>
We strongly advise running this project on [Ubuntu 22.04](https://ubuntu.com/tutorials/install-ubuntu-desktop#1-overview) to make sure it is compatible with the [ROS distribution](https://docs.ros.org/en/humble/index.html) we used.

Also, make sure that the system used allows for connection to the drone (We encountered problems while trying to connect the drone to [WSL](https://learn.microsoft.com/en-us/windows/wsl/about).)

To settle the environment to run this project on your side, you must:
- Clone this repository
  ```sh
  git clone https://github.com/snt-arg/Tello-object-detection.git
  ```
  You need to have [git](https://git-scm.com/book/en/v2/Getting-Started-Installing-Git) installed.
  
- Install the requirements specified in [requirements.txt](https://github.com/snt-arg/Tello-object-detection/blob/main/requirements.txt)\
  If you possess, [pip](https://pypi.org/project/pip/), you can run the command
  ```sh
  pip install -r requirements.txt
  ``` 
  Even if you do not want to install the packages by using this file, we still recommend you to have a look on the versions of the packages we used, to avoid incompatibility errors.

- Install ROS 2:\
  For this project, we used ROS 2 Humble Hawksbill. The platforms supported are Ubuntu 22.04, RHEL-8, Windows 10, and macOS(building from source).
  To install on Ubuntu 22.04 from binary packages(recommended), follow [this tutorial](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html).
  For the other platforms, more information is found [here](https://docs.ros.org/en/humble/Installation.html).
  
- Install Tello driver and hand gesture packages:\
  These are two ROS packages mandatory to possess to run the [person_tracking](https://github.com/snt-arg/Tello-object-detection/tree/main/person_tracking) package.
  You must clone them in the same ROS workspace as [person_tracking](https://github.com/snt-arg/Tello-object-detection/tree/main/person_tracking).\
  
  Tello driver:
  ```sh
  git clone https://github.com/snt-arg/tello_ros_driver.git
  ```
  Hand gesture plugin:
  ```sh
  git clone https://github.com/snt-arg/hand_gestures_plugin.git
  ```

  NB:\
   * YOLOv8 :\
        YOLOv8 should be installed if you installed all packages in [requirements.txt](https://github.com/snt-arg/Tello-object-detection/blob/main/requirements.txt).\
      If not, you can run
      ```sh
      pip install ultralytics
      ```
      However, we recommend that you install all the packages in the requirement file to avoid errors.\
        Please note that to run a code that uses YOLOv8, you should possess a Python version >=3.8, and PyTorch >= 1.8.\
        [For more information on how to install YOLOv8](https://github.com/ultralytics/ultralytics?tab=readme-ov-file#documentation).
    
    * DJITelloPy:\
       It should be installed if you installed packages in [requirements.txt](https://github.com/snt-arg/Tello-object-detection/blob/main/requirements.txt). The [tello](https://github.com/snt-arg/Tello-object-detection/tree/main/ros_workspace/tello) package requires [DJITelloPy](https://github.com/damiafuentes/DJITelloPy) to establish the connection to the drone.
      It can be installed with
      ```sh
      pip install djitellopy
      ```
      Again, it is advised to install all requirements from the requirement file.
  

## 🧑‍💻️🏃 Run code
This repository is organized into 4 repertories:
- *config* where all configuration files are kept.
- *media* containing some videos on which we ran our object detection module to test its functioning.
- *ros_workspace* contains a robotic package ([tello](https://github.com/snt-arg/Tello-object-detection/tree/main/ros_workspace/tello)) we implemented to perform object detection with a DJI Tello drone.
  To run the *tello* package, you have to fulfill all installation requirements listed above.
  Then in a terminal,
  - Move to the ros_workspace directory after cloning the repository
  - Source your ROS distribution using
  - Build the Tello package
  - Check your WIFI connection to make sure that your computer is connected to the drone.
  - Open three other terminals and source the ROS distribution and the Tello package in all of them
      * In the first one, run the drone's camera publisher node using
        ```sh
        ros2 run tello camera_pub
        ```
      * In the second one, run the object detection subscriber and republisher node using
        ```sh
        ros2 run tello detected_pub
        ```
      * In the third terminal, you can use visualization tools such as rqt_image_view to view the image messages published on each topic (*image_raw* for raw images coming from the drone, and *image_detected* for images on which object detection was performed. )
    

## 👨🏻‍💻📝Implementation <a id="implementation"></a>
This project uses a publisher/subscriber architecture to detect objects on a drone's camera video.
The publisher node [camera_publisher.py](https://github.com/snt-arg/Tello-object-detection/blob/main/ros_workspace/tello/tello/camera_publisher.py) is in charge of connecting to the drone (using [DJITelloPy](https://github.com/damiafuentes/DJITelloPy)) and reading video frames from the drone's camera.
These frames are then published on a topic name *image_raw*.
The subscriber node [sub1.py](https://github.com/snt-arg/Tello-object-detection/blob/main/ros_workspace/tello/tello/sub1.py) receives frames published on *image_raw*, performs object detection on them, and republishes the new frames (with bounding boxes around objects) on *image_detected*.
## 🧾 License <a id="license"></a>



