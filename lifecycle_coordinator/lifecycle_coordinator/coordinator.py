#!/usr/bin/env python3

# Lifecycle coordinator for a series of nodes for ROB 499 Final Project
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
	def __init__(self, node_names):
		super().__init__('coordinator')

		self.declare_parameter('node_names', 'camera_driver')
		self.node_names = self.get_parameter('node_names').get_parameter_value().string_array_value

		# Make a service clients for each of the nodes we want to coordinate.
		self.client_list = [self.create_client(ChangeState, f'{name}/change_state') for name in node_names]

		# Wait until the service is available and the client is connected.
		for client in self.client_list:
			while not client.wait_for_service(timeout_sec=1):
				self.get_logger().info('waiting for lifecycle node to become available')

		# Create subscriber so that we can control lifecycle nodes
		self.service = self.create_service(ChangeStateSrv, 'transition', self.service_callback)

		self.manager = LifecycleCoordinator(self.node_names)

	def service_callback(self, msg):


		new_state = msg.transition
		request = ChangeState.Request()
		request.transition.id = new_state




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
