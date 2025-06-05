# ROB499 Project

This folder contains the nodes for the final project of ROB 499. Each package has a README if you 
are curious about parameters or what they do in detail. 

This project was built using the ROS2 Jazzy distribution and requires the following Python packages:

opencv-python
numpy

To install the dependencies, run:

pip install opencv-python numpy

--------------------------------------------------------------------------------------------------

**Package Launch:**  Provides a single launch file dedicated to launching nodes and modifying parameters.

**Lifecycle Coordinator:** A node that can coordinate multiple lifecycle enabled nodes at a time, bringing
them up or down in correct sequence. A list of nodes are passed in as a parameter at launch time.

**Project Interfaces:** Contains custom service definitions.

**Camera Driver:** Captures images from a UVC compliant camera and publishes them on an image topic.
Has parameters for device ID as well as encoding and resolution.

**Periodic Screenshot:** Subscribes to an image topic and saves an image to disc at a specified rate 
as determined by parameters.

**Ring Buffer Recorder:** Buffers image frames from a subscribed topic and saves a video to disc upon 
a service call. Has parameters for video length and encoding.

**Bag Recorder:** Records a bag from a list of topics that you can pass in as parameters. 
Has a parameter that lets you turn on and off bag saving but is not a dynamic parameter.

**Sobel Filter:** A node that takes in an image topic and republishes the x, y and xy sobel filtered 
variants of the image as well as a OpenCV Canny Edge detected image. 

**Strain Logger:** A node that subscribes to a OpenCV canny edge image and calculates the distance between
contours. Publishes an image of the region of interest and the strain, given the subject’s original length.

**Event Trigger:** A node that triggers the Ring Buffer Recorder node on a service call.

**CSV Writer:** A node that subscribes to a Float32 topic, creates and appends data to a CSV file saved on 
the computer. 

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





