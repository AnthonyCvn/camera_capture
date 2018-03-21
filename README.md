# Package Name

## Overview

ROS package to capture frames from any UVC camera. The frame are shared using image_transport package.
Camera settings (height, width and frequency) can be changed dynamically with the ROS service "set_camera".

**Keywords:** camera, capture, uvc

### License

The source code is released under a [BSD 2-Clause license](https://github.com/AnthonyCvn/camera_capture/blob/master/LICENSE).

**Author: Anthony Cavin**

The camera_capture package has been tested under [ROS] kinetic and Ubuntu 16.04.

## Installation

Reference page to install ROS kinetic on Ubuntu:

[Robot Operating System (ROS)] http://wiki.ros.org/kinetic/Installation/Ubuntu

#### Dependencies

- [Robot Operating System (ROS)](http://wiki.ros.org) (middleware for robotics),
- [image_transport](http://wiki.ros.org/image_transport) (ROS package),
- [OpenCV](https://opencv.org/) (library for computer vision)

#### Building

To build from source, clone the latest version from this repository into your catkin workspace and compile the package using

	cd catkin_workspace/src
	git clone https://github.com/AnthonyCvn/camera_capture.git
	cd ../
	catkin build camera_capture

### Unit Tests

Run the unit tests with

	catkin run_tests camera_capture

## Usage

Run the main node with

	roslaunch camera_capture camera_capture.launch

Use the ROS service to change the camera settings:

	rosservice call /camera_capture/set_camera 
		"{height: 1080, width: 1920, fps: 30, conf_camera: true, run_camera: true}"

## Launch files

* **camera_capture.launch:** Launch one camera.
    
    Argument set 1

    - **`is_running`** Run (true) or stop (false) the camera. Default: `true`.

    Argument set 2
	
     - **`camera_num`** Camera number to run. Default: `0`.

    Argument set 3
	
     - **`is_stereo`** Set true for stereo camera. Default: `true`.

    Argument set 4
	
     - **`height`** Resolution in pixel. Default: `720`.

    Argument set 5
	
     - **`width`** Resolution in pixel. Default: `2560`.

    Argument set 6
	
     - **`fps`** Frame per second to capture from the camera. Default: `30`.

## Nodes

### camera_capture

Open the UVC camera and share the frames using image_transport package.

#### Published Topics

* **`/image`** ([sensor_msgs/Image])

	The images captured with the camera in raw format.

* **`/image/theora`** ([theora_image_transport/Packet])

	The images captured with the camera compressed with the Theora codec.


#### Services

* **`set_camera`** ([camera_capture/SetCamera])

	Set dynamically camera's settings.

		rosservice call /camera_capture/set_camera "{height: 1080, width: 1920, fps: 30, conf_camera: true, run_camera: true}"


## Bugs & Feature Requests

Please report bugs and request features using the [Issue Tracker](https://github.com/AnthonyCvn/camera_capture/issues).


[ROS]: http://www.ros.org
[rviz]: http://wiki.ros.org/rviz
[sensor_msgs/Image]: http://docs.ros.org/api/sensor_msgs/html/msg/Image.html
[theora_image_transport/Packet]: http://docs.ros.org/api/theora_image_transport/html/msg/Packet.html
