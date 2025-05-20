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

		
		self.TRANSITION_MAP = {
			'configure' : Transition.TRANSITION_CONFIGURE,
			'activate' : Transition.TRANSITION_ACTIVATE,
			'deactivate' : Transition.TRANSITION_DEACTIVATE,
			'cleanup' : Transition.TRANSITION_CLEANUP,
			'shutdown' : Transition.TRANSITION_INACTIVE_SHUTDOWN,
		}

		# Command sequences so we can go through sequences more naturally
		self.COMMAND_SEQUENCE = {
			'bringup': ['configure', 'activate'],
			'pause' : ['deactivate'],
			'unpause' : ['activate'],
			'shutdown' : ['deactivate', 'cleanup', 'shutdown'],
		}
		

	def service_callback(self, request, response):

		command = request.command

		# Make sure the input is a known string
		if command not in self.COMMAND_SEQUENCE:
			response.success = False
			response.result = f"'{command}' is an unknown request"
			return response


		# Iterate through the state sequence
		for state in self.COMMAND_SEQUENCE[command]:

			transition_id = self.TRANSITION_MAP[state]

			self.get_logger().info(f"Requesting '{state}' transition (id {transition_id})")

			self.change_state(transition_id)

			# Spin until transitions are complete
			while rclpy.ok() and not self.requests_done():
				rclpy.spin_once(self, timeout_sec=0.1)

		response.success = True
		response.result = f"Lifecycle command: {command} completed successfully."
		return response

	# This wraps up the state change request.
	def change_state(self, state):
		request = ChangeState.Request()
		request.transition.id = state

		# Make the calls, one to each node, and store the futures in a list.
		self.responses = [client.call_async(request) for client in self.client_list]

	# Are all of the responses done?  If any of them are not done, then return False.
	# Otherwise, return True.
	def requests_done(self):
		for response in self.responses:
			if not response.done():
				return False

		return True



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
