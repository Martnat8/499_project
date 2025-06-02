#!/usr/bin/env python3


# This is a ROS2 node that on a trigger, currently a timer, can envoke the
# service calls of other nodes. For now this only triggers bag recorder.
#
# event_trigger.py
#
# Nathan Martin



import rclpy
from rclpy.lifecycle import LifecycleNode, TransitionCallbackReturn
from project_interfaces.srv import SaveSrv


class EventTrigger(LifecycleNode):
	def __init__(self):

		# Initialize the parent class 
		super().__init__('event_trigger')

		# Declaring parameters
		self.declare_parameter('timer_interval', 15.0)


	def on_configure(self, previous_state):
		
		self.get_logger().info('Configuring Event Trigger')

		# Pull out parameters
		timer_interval = self.get_parameter('timer_interval').value

		# Create a timer using the frequency parameter
		self.timer = self.create_timer(timer_interval, self.timer_callback, autostart=False)

		# Create the client for the service call, watch for failure
		self.client = self.create_client(SaveSrv, '/ring_buffer_recorder/save_request')

		if not self.client.wait_for_service(timeout_sec=5.0):
			self.get_logger().error('Service not available: /ring_buffer_recorder/save_request')
			return TransitionCallbackReturn.FAILURE

		return TransitionCallbackReturn.SUCCESS


	def on_activate(self, previous_state):
		self.get_logger().info('Activating Event Trigger')

		# Start timer for callback
		self.timer.reset()

		return super().on_activate(previous_state)
	

	def on_deactivate(self, previous_state):
		self.get_logger().info('Deactivating Event Trigger')

		# Pause the timer
		self.timer.cancel()

		return super().on_deactivate(previous_state)
	
	
	def on_cleanup(self, previous_state):
		self.get_logger().info('Cleaning up')

		# Get rid of the timer
		self.destroy_timer(self.timer)

		return TransitionCallbackReturn.SUCCESS
	

	def on_shutdown(self, previous_state):
		self.get_logger().info('Shutting down')

		# Get rid of the timer
		self.destroy_timer(self.timer)

		return TransitionCallbackReturn.SUCCESS


	def on_error(self, previous_state):
		self.get_logger().error('ERROR!')

		return TransitionCallbackReturn.ERROR
	
	# Callback for the timer
	def callback(self):

		request = SaveSrv.Request()

		request.save_request = True

		future = self.client.call_async(request)

		return

def main(args=None):
	
	rclpy.init(args=args)

	trigger = EventTrigger()

	rclpy.spin(trigger)

	rclpy.shutdown()

if __name__ == '__main__':
	
	main()