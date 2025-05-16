# ROB 499 Robot Software Frameworks Project

This package contains a lifecycle coordinator node. 

Once launched can be controlled with service calls:

# Service call syntax
ros2 service call /transition project_interfaces/srv/ChangeStateSrv "{transition: {id: 1}}"

Configure: id=1
Cleanup: id=2
Activate: id=3
Deactivate: id=4
Shutdown: id=5

You can only transition in a certain order here in order:

configure (1) -> activate (3) -> deactivate (4) -> cleanup (2) -> shutdown (5)
_____________________________________________________________________________________
_____________________________________________________________________________________
Maintainer - Nathan Martin - martnat8@oregonstate.edu
_____________________________________________________________________________________
License - BSD 3-Clause
_____________________________________________________________________________________


