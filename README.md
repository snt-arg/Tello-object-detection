# 👩‍💻📸 Object detection using YOLOv8

For my Bachelor Semester Project 2, I chose to focus on object detection, by leveraging on YOLOv8, 
a popular framework for object detection, object classification and more, developed by [Ultralytics](https://github.com/ultralytics).

GitHub repository for YOLOv8 : (https://github.com/ultralytics/ultralytics)

The project is divised into two parts:
* Object detection Python application
* Implementation on a robot using [ROS](https://www.ros.org/)

## 🔍 Table Of Contents

- [🛠 Installation](#installation)
- [👨🏻‍💻📝Implementation](#implementation)
  - [🐍👩🏻‍💻Python application](#application)
  - [🤖ROS (Robot Operating System)](#ROS) 
- [🧾 License](#license)

## 🛠️ Installation <a id="installation"></a>
To run this project on your side, follow these steps:
- Clone this repository
  ```sh
  git clone https://github.com/maeri18/YOLOV8-BSP2.git
  ```
  You need to have [git](https://git-scm.com/book/en/v2/Getting-Started-Installing-Git) installed
- Install the requirements specified in [requirements.txt](https://github.com/maeri18/YOLOV8-BSP2/blob/main/requirements.txt)\
  If you possess, [pip](https://pypi.org/project/pip/), you can run the command
  ```sh
  pip install -r requirements.txt
  ```
  Even if you do not want to install the packages by using this file, we still recommend you to have a look on the versions of the packages we used, in order to avoid incompatibility errors.
- Install YOLOv8 :\
    YOLOv8 should be installed if you installed all packages in [requirements.txt](https://github.com/maeri18/YOLOV8-BSP2/blob/main/requirements.txt).\
  If not, you can run
  ```sh
  pip install ultralytics
  ```
  However, we recommend that you install all the packages in the requirement file to avoid errors.\
    Please note that to run a code that uses YOLOv8, you should possess a Python version >=3.8, and PyTorch >= 1.8.\
    [For more information on how to install YOLOv8](https://github.com/ultralytics/ultralytics?tab=readme-ov-file#documentation)
- Run the file [filter_class.py](https://github.com/maeri18/YOLOV8-BSP2/blob/main/object_detection/filter_class.py) on one of your video to check if everything works fine.

## 🧾 License <a id="license"></a>



