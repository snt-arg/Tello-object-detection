# 
# 👋 Object detection using YOLOv8

For my Bachelor Semester Project 2, I chose to focus on object detection, by leveraging on YOLOv8, 
a popular framework for object detection, object classification and more, developed by [Ultralytics](https://github.com/ultralytics).

GitHub repository for YOLOv8 : (https://github.com/ultralytics/ultralytics)

The project is divised into two parts:
* Object detection Python application
* Implementation on a robot using [ROS](https://www.ros.org/)

## 🔍 Table Of Contents

- [🛠️ Installation](#installation)
  - [🧩 Dependencies](#dependencies)
- [🚀 Usage](#usage)
- [🏗️ Architecture](#architecture)
- [🤖 ROS Related](#ros-related)
  - [📥 Subscribed Topics](#subscribed-topics)
  - [📤 Published Topics](#published-topics)
  - [⚙️ Parameters](#parameters)
- [🔑 License](#license)

## 🛠️ Installation <a id="installation"></a>

### 🧩 Dependencies <a id="dependencies"></a>

- [`tello_plugin_utils`](https://github.com/snt-arg/tello_plugin_utils).

1. Clone this repo

```sh
git clone git@github.com:snt-arg/hand_gestures_plugin.git && cd hand_gestures_plugin
```

2. Clone `tello_plugin_utils`

```sh
git clone git@github.com:snt-arg/tello_plugin_utils.git
```

3. Build

```sh
colcon build --symlink-install && source install/setup.bash # or .zsh
```

## 🚀 Usage <a id="usage"></a>

> [!NOTE]
> Each command needs to be executed in 2 different terminals and you must source the `install/setup.bash` which was created in the section above.

1. Launch the detector node in **standalone** mode

```sh
 ros2 run hand_gestures landmark_detector_node --ros-args -p standalone:=true
```

2. Launch the annotator node in **standalone** mode

```sh
 ros2 run hand_gestures landmark_annotator_node --ros-args -p standalone:=true
```

## 🏗️ Architecture <a id="architecture"></a>

<p align="center">
    <img src="./assets/Architecture.png" width="100%"/>
</p>

The main idea is to split work into multiple nodes. We first have a `detector` node, which only goal is to detect the hand landmarks and publish them to a topic `/hand/landmarks`.

In addition, we have another node `annotator` which receives the same images which are fed into the detector node and also the landmarks produced by the detector node. Then, it will draw the detected landmarks into the image and publish the annotated image to the topic `/hand/annotated/image`.

To conclude the hand gesture package, we have the `sign_classifier` node. It's goal is to receive the hand landmarks and interpret those into hand gestures.

In order to support multiple robots, it is needed to create an extra node which will be depended to the robot itself. This node takes in the interpreted gestures and then translates them into velocity commands and others, like stand up, take-off, etc.

## 🤖 ROS Related <a id="ros-related"></a>

### 📥 Subscribed Topics <a id="subscribed-topics"></a>

#### 🌐 Landmark Detector Node

| Topic Name          | Message Type                                                                             | Description                     |
| ------------------- | ---------------------------------------------------------------------------------------- | ------------------------------- |
| `/camera/image_raw` | [sensor_msgs/Image](http://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/Image.html)) | Input image for hand detection. |

#### 🌐 Landmark Annotator Node

| Topic Name          | Message Type                                                                                                                    | Description                         |
| ------------------- | ------------------------------------------------------------------------------------------------------------------------------- | ----------------------------------- |
| `/camera/image_raw` | [sensor_msgs/Image](http://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/Image.html))                                        | Input image use for hand detection. |
| `/hand/landmarks`   | [hand_gestures_msgs/Landmakrs](https://github.com/snt-arg/hand_gestures_plugin/blob/main/hand_gestures_msgs/msg/Landmarks.msg)) | Landmarks detected.                 |

### 📤 Published Topics <a id="published-topics"></a>

#### 🌐 Landmark Detector Node

| Topic Name        | Message Type                                                                                                                    | Description                                |
| ----------------- | ------------------------------------------------------------------------------------------------------------------------------- | ------------------------------------------ |
| `/hand/landmarks` | [hand_gestures_msgs/Landmakrs](https://github.com/snt-arg/hand_gestures_plugin/blob/main/hand_gestures_msgs/msg/Landmarks.msg)) | Output landmarks topic for detected hands. |

#### 🌐 Landmark Annotator Node

| Topic Name              | Message Type                                                                             | Description                                        |
| ----------------------- | ---------------------------------------------------------------------------------------- | -------------------------------------------------- |
| `/hand/annotated/image` | [sensor_msgs/Image](http://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/Image.html)) | Annotated image with landmarks for detected hands. |

### ⚙️ Parameters <a id="parameters"></a>

#### 🌐 Landmark Detector Node

| Parameter Name             | Description                          | Default             |
| -------------------------- | ------------------------------------ | ------------------- |
| `img_input_topic`          | Input image topic name.              | `/camera/image_raw` |
| `landmarks_topic`          | Detected landmarks on image.         | `hand/landmakrs`    |
| `num_hands`                | Maximum number of hands to detect.   | 2                   |
| `min_detection_confidence` | Minimum confidence to detect a hand. | 0.5                 |
| `min_tracking_confidence`  | Minimum confidence to track a hand.  | 0.5                 |

#### 🌐 Landmark Annotator Node

| Parameter Name        | Description                                | Default             |
| --------------------- | ------------------------------------------ | ------------------- |
| `img_input_topic`     | Input image topic name.                    | `/camera/image_raw` |
| `annotated_img_topic` | Annotated image with landmarks topic name. | `/camera/image_raw` |
| `landmarks_topic`     | Detected landmarks on image.               | `hand/landmakrs`    |

## 🔑 License <a id="license"></a>

This project is licensed under the GPL-3.0 license - see the [LICENSE](https://github.com/snt-arg/hand_gestures_plugin/blob/main/LICENSE) for more details.

