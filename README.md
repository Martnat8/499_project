# ROB499 Project

This folder contains the nodes for the final project of ROB 499. Each package has a README if you 
are curious about parameters or what they do in detail.

--------------------------------------------------------------------------------------------------

**Package Launch:**  Provides a single launch file dedicated to launching nodes and modifying params.

**Lifecycle Coordinator:** A node that can coordinate multiple lifecycle enabled nodes at a time, bringing
  them up or down in correct sequence.

**Project Interfaces:** Contains custom service definitions.

**Camera Driver:** Captures images from a UVC compliant camera and publishes them on an image topic.

**Periodic Screenshot:** Subscribes to an image topic and save an image to disc at a specified rate.

**Ring Buffer Recorder:** Buffers image frames from a subscribed topic and saves a video to disc upon 
  a service call.

--------------------------------------------------------------------------------------------------

# Usage:

colcon build
source install/setup.bash

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



