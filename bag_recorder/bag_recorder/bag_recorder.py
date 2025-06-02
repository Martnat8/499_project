#!/usr/bin/env python3


# This is a ROS2 bag recorder node. It takes in a list of topics as
# a parameter and saves them all as a bag. On pause makes a new bag
#
# bag_recorder.py
#
# Nathan Martin



import os
import rclpy
import subprocess
from rclpy.lifecycle import LifecycleNode, TransitionCallbackReturn

# Import service
from project_interfaces.srv import SaveSrv


class BagRecorder(LifecycleNode):
	def __init__(self):

		# Initialize the parent class 
		super().__init__('bag_recorder')

		# Declaring parameters
		self.declare_parameter('topics', ['/raw_image_out'])
		self.declare_parameter('bag_name', 'test_bag')
		self.declare_parameter('save_directory', '.')
		self.declare_parameter('is_saving', False)

		self.bag_process = None


	def on_configure(self, previous_state):
		
		self.get_logger().info('Configuring Bag Recorder')

		# Pull out parameters
		self.topics = self.get_parameter('topics').value
		self.bag_name = self.get_parameter('bag_name').value
		self.save_directory = self.get_parameter('save_directory').value
		self.is_saving = self.get_parameter('is_saving').value

		if self.is_saving:

			# Establish filepaths
			os.makedirs(self.save_directory, exist_ok=True)

			# Counter to make sure we don't override bags if paused.
			self.counter = 1

		return TransitionCallbackReturn.SUCCESS

	def on_activate(self, previous_state):

		if self.is_saving:

			self.get_logger().info('Activating ROS2 bag recording')

			file_name = f'{self.bag_name}_{self.counter}'

			self.bag_path = os.path.join(self.save_directory, file_name)

			# Create the command and use Popen to run it
			cmd = ['ros2', 'bag', 'record', '-o', self.bag_path] + self.topics
			self.get_logger().info(f"Running: {' '.join(cmd)}")

			try:
				self.bag_process = subprocess.Popen(cmd)
			except Exception as e:
				self.get_logger().error(f"Failed to start bag recording :{e}")
				return TransitionCallbackReturn.FAILURE
			
		else:
			self.get_logger().info('Bag recording skipped (is_saving set to False)')

	
		return super().on_activate(previous_state)
	

	def on_deactivate(self, previous_state):
		self.get_logger().info('Deactivating ROS2 bag recording')

		if self.is_saving:

			if self.bag_process:
				self.bag_process.terminate()
				self.bag_process.wait()
				self.bag_process = None

			# Increment counter for next recording name
			self.counter += 1

		return super().on_deactivate(previous_state)
	
	
	def on_cleanup(self, previous_state):
		self.get_logger().info('Cleaning up')

		if self.is_saving:

			if self.bag_process:
				self.bag_process.terminate()
				self.bag_process.wait()

		return TransitionCallbackReturn.SUCCESS

	
	def on_shutdown(self, previous_state):
		self.get_logger().info('Shutting down')
		
		if self.is_saving:

			if self.bag_process:
				self.bag_process.terminate()
				self.bag_process.wait()

		return TransitionCallbackReturn.SUCCESS


	def on_error(self, previous_state):
		self.get_logger().error('ERROR!')

		return TransitionCallbackReturn.ERROR


def main(args=None):
	
	rclpy.init(args=args)

	bag = BagRecorder()

	rclpy.spin(bag)

	rclpy.shutdown()

if __name__ == '__main__':
	
	main()