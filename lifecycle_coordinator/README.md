# ROB 499 Robot Software Frameworks Project

This package contains a lifecycle coordinator node. 

Once launched can be controlled with service calls:

# Service call syntax
ros2 service call /lifecycle_control project_interfaces/srv/ChangeStateSrv "{ command: 'bringup' }"

These are the different commands you can pass in the service. Each will move through the states in sequence.

'bringup': configure -> activate
'pause' : deactivate
'unpause' : activate
'shutdown' : deactivate -> cleanup -> shutdown

_____________________________________________________________________________________
_____________________________________________________________________________________
Maintainer - Nathan Martin - martnat8@oregonstate.edu
_____________________________________________________________________________________
License - BSD 3-Clause
_____________________________________________________________________________________


