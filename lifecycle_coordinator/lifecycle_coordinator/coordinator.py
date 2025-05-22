#!/usr/bin/env python3

# This node is a lifecycle coordinator. 
#
# lifecycle_coordinator.py
#
# Nathan Martin
#



# Import the usual ROS stuff
import rclpy
from rclpy.node import Node

# We need the ChangeState service type declaration and the Transition message
from lifecycle_msgs.srv import ChangeState
from lifecycle_msgs.msg import Transition

# Import service
from project_interfaces.srv import ChangeStateSrv



class LifecycleCoordinator(Node):
	def __init__(self):
		super().__init__('coordinator')

		# Parameter is an list of lifecycle enabled nodes.
		self.declare_parameter('node_names', ['camera_driver'])
		node_names = self.get_parameter('node_names').value

		# Make a service clients for each of the nodes we want to coordinate.
		self.client_list = [self.create_client(ChangeState, f'{name}/change_state') for name in node_names]

		# Wait until the service is available and the client is connected.
		for client in self.client_list:
			while not client.wait_for_service(timeout_sec=1):
				self.get_logger().info('waiting for lifecycle node to become available')

		# Create subscriber so that we can control lifecycle nodes
		self.service = self.create_service(ChangeStateSrv, 'lifecycle_control', self.service_callback)

		self.step_index = 0
		self.current_sequence = []
		self.responses: list = []

		# Command sequences so we can go through sequences more naturally
		self.command_sequence = {
			'bringup': ['configure', 'activate'],
			'pause' : ['deactivate'],
			'unpause' : ['activate'],
			'shutdown' : ['deactivate', 'shutdown'],
		}

		self.transition_map = {
			'configure' : Transition.TRANSITION_CONFIGURE,
			'activate' : Transition.TRANSITION_ACTIVATE,
			'deactivate' : Transition.TRANSITION_DEACTIVATE,
			'cleanup' : Transition.TRANSITION_CLEANUP,
			'shutdown' : Transition.TRANSITION_INACTIVE_SHUTDOWN,
		}


	def service_callback(self, request, response):

		# Check to see if we're already running a transition
		if self.step_index < len(self.current_sequence):
			response.success = False
			response.result = "Another transition already in progress"
			return response

		# Check to see if the input string matches a known command
		if request.command not in self.command_sequence:
			response.success = False
			response.result  = f"Unknown command '{request.command}'"
			return response

		# Grab the command sequence and initialize a fresh count 
		self.current_sequence = self.command_sequence[request.command]
		self.step_index = 0

		# launch the first transition
		self._launch_step()

		# Fill Response and send
		response.success = True
		response.result  = f"Started '{request.command}' sequence"
		return response



	# Grabs the state id, sends the request and attaches callbacks
	def _launch_step(self):

		# Step index is used to know if we've completed all commands
		if self.step_index >= len(self.current_sequence):
			self.get_logger().info("All steps done")
			return

		# Grab the state and transition id, log the request
		state = self.current_sequence[self.step_index]
		transition_id   = self.transition_map[state]
		self.get_logger().info(f"Requesting '{state}' (id {transition_id})")

		# Futures assigned to the responses of change_state's use of call_async
		futures = self.change_state(transition_id)
		for fut in futures:
			fut.add_done_callback(self._on_step_done)

	# Checks transition success and launches next step or drops out if needed
	def _on_step_done(self, fut):

		# ignore any calls once we’ve advanced past the last step
		if self.step_index >= len(self.current_sequence):
			return

		# Silently bail out if all of the callbacks are not finished
		if not all(f.done() for f in self.responses):
			return  

		# Check success
		if not all(f.result().success for f in self.responses):
			self.get_logger().error(f"Step {self.current_sequence[self.step_index]} failed")
			return

		# Move to the next step
		self.get_logger().info(f"Step {self.current_sequence[self.step_index]} succeeded")
		self.step_index += 1
		self._launch_step()


	# Make a call for each state in the client list and assign futures.
	def change_state(self, state):
		request = ChangeState.Request()
		request.transition.id = state

		# Make the calls, one to each node, and store the futures in a list.
		self.responses = [client.call_async(request) for client in self.client_list]

		return self.responses



def main(args=None):
	
	# Initialize rclpy.  We should do this every time.
	rclpy.init(args=args)

	# Make a node class.
	coordinator = LifecycleCoordinator()

	# Handover to ROS2
	rclpy.spin(coordinator)

	# Make sure we shutdown everything cleanly.	
	rclpy.shutdown()



if __name__ == '__main__':
	main()
