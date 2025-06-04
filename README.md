# ROB499 Project

This folder contains the nodes for the final project of ROB 499. Each package has a README if you 
are curious about parameters or what they do in detail. 

This project was built using the ROS2 Jazzy distribution and requires the following Python packages:

opencv-python
numpy

To install the dependencies, run:

pip install opencv-python numpy

--------------------------------------------------------------------------------------------------

**Package Launch:**  Provides a single launch file dedicated to launching nodes and modifying params.

**Lifecycle Coordinator:** A node that can coordinate multiple lifecycle enabled nodes at a time, bringing
  them up or down in correct sequence.

**Project Interfaces:** Contains custom service definitions.

**Camera Driver:** Captures images from a UVC compliant camera and publishes them on an image topic.

**Periodic Screenshot:** Subscribes to an image topic and save an image to disc at a specified rate.

**Ring Buffer Recorder:** Buffers image frames from a subscribed topic and saves a video to disc upon 
  a service call.

**Bag Recorder:** Records a bag from a list of topics that you can pass in as parameters. Has a 
  parameter that lets you turn on and off bag saving but is not a dynamic parameter. It's just 
  meant to allow me to turn off bag saving if I don't want it on while keeping the structure of the
  launch file. If you want to record with it you need to set true and relaunch.

**Sobel Filter:** A node that takes in an image topic and republishes the x, y and xy sobel filtered
  variants of the image. A lifecycle enabled node

**Strain Recorder:** A node that subscribes to a OpenCV canny edge image and calculates the distance
  between contours. Publishes an image of the region of interest and the strain, given the subjects
  original length.

--------------------------------------------------------------------------------------------------

# Usage:

colcon build
source install/setup.bash

# Check device (sometimes need to change id in launch file)
For camera device enumeration and configuration, v4l2-ctl (part of the Video4Linux2 utilities) is used. You can install it with sudo apt install v4l-utils if it's not already available.

v4l2-ctl --list-devices


# Launch the whole suite of nodes:
ros2 launch package_launch projectlaunch.py

# Use the coordinator to bring up and shutdown nodes
ros2 service call /lifecycle_control project_interfaces/srv/ChangeStateSrv "{ command: 'bringup' }"

These are the different commands you can pass in the service. Each will move through the states in sequence.The coordinator node will send a command out of sequence if you tell it to and everything will crash. Might be 
nice to add protection against that in the future, but for now just be careful.

'bringup': configure -> activate
'pause' : deactivate
'unpause' : activate
'shutdown' : deactivate -> cleanup -> shutdown


Pause can only be run while active and unpause only while paused. Shutdown can only be run while active, not paused.

# Service call to save video to file
The service call takes no request right now, just needs a trigger.

ros2 service call /save_request project_interfaces/srv/SaveSrv "{}"





