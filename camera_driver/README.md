# ROB 499 Robot Software Frameworks Project

This package contains a lifecycle controlled camera driver node. 

Once launched can be controlled with Lifecycle Commands:

# List all lifecycle nodes
ros2 lifecycle list

# Check current state of camera_driver
ros2 lifecycle get /camera_driver

# Configure it
ros2 lifecycle set /camera_driver configure

# Activate it (starts publishing /raw_image_out)
ros2 lifecycle set /camera_driver activate

# Deactivate it (pauses publishing)
ros2 lifecycle set /camera_driver deactivate

# Cleanup resources (back to “unconfigured”)
ros2 lifecycle set /camera_driver cleanup

# Shutdown permanently (moves to “finalized”)
ros2 lifecycle set /camera_driver shutdown

_____________________________________________________________________________________
_____________________________________________________________________________________
Maintainer - Nathan Martin - martnat8@oregonstate.edu
_____________________________________________________________________________________
License - BSD 3-Clause
_____________________________________________________________________________________


