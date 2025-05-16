#!/usr/bin/env python3

# This node is a lifecycle coordinator. 
#
# lifecycle_coordinator.py
#
# Nathan Martin


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
		self.service = self.create_service(ChangeStateSrv, 'transition', self.service_callback)


	def service_callback(self, request, response):

		state = request.transition.id


		self.get_logger().info(f'Requesting transition to {state}')
		futures = self.change_state(state)

		for fut in futures:
			fut.add_done_callback(self._on_transition_done)

		response.success = True
		response.result = "Transition in progress"
		return response

	# This wraps up the state change request.
	def change_state(self, state):

		request = ChangeState.Request()
		request.transition.id = state

		# Make the calls, one to each node, and store the futures in a list.
		self.responses = [client.call_async(request) for client in self.client_list]

		return self.responses

	# This reports on the futures as transitions succeed
	def _on_transition_done(self, fut):

		try:
			result = fut.result()
			self.get_logger().info(f'Transition succeeded: {result}')

		except Exception as e:
			self.get_logger().error(f'Transition failed: {e}')


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
