#!/usr/bin/env python3


# Subscribes to an image topic and saves an image to disk at a parameterized
# rate and quality
#
# camera_driver.py
#
# Nathan Martin


import cv2
import os
import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
from rclpy.lifecycle import LifecycleNode, TransitionCallbackReturn
from sensor_msgs.msg import Image



class PeriodicScreenshot(LifecycleNode):
	def __init__(self):

		# Initialize the parent class of name oscope
		super().__init__('periodic_screenshot')

		# Declaring parameters
		self.declare_parameter('topic_name', '/raw_image_out')
		self.declare_parameter('timer_interval', 30)
		self.declare_parameter('save_name', 'test_img')
		self.declare_parameter('save_directory', '.')

	def on_configure(self, previous_state):
		
		self.get_logger().info('Configuring')

		# Use CV Bridge to convert between ROS 2 and OpenCV images
		self.bridge = CvBridge()

		# Pull out parameters to determine capture parameters
		topic = self.get_parameter('topic_name').value
		timer_interval = self.get_parameter('timer_interval').value
		self.save_name = self.get_parameter('save_name').value
		self.save_directory = self.get_parameter('save_directory').value

		# Create the subscriber
		self.sub = self.create_subscription(Image, topic, self.callback, 10)

		# Counter to track saved files
		self.counter = 0

		# Variable to hold previous image message
		self.last_img = None

		# Create a timer using the frequency parameter
		self.timer = self.create_timer(timer_interval, self.timer_callback, autostart=False)

		# Establish filepaths
		os.makedirs(self.save_directory, exist_ok=True)

		return TransitionCallbackReturn.SUCCESS



	def on_activate(self, previous_state):
		self.get_logger().info('Activating')

		# Start timer for callback
		self.timer.reset()

		return super().on_activate(previous_state)
	

	def on_deactivate(self, previous_state):
		self.get_logger().info('Deactivating')

		# Pause the timer
		self.timer.cancel()

		return super().on_deactivate(previous_state)
	
	
	def on_cleanup(self, previous_state):
		self.get_logger().info('Cleaning up')

		# Get rid of the timer
		self.destroy_timer(self.timer)

		# Get rid of the subscription
		self.destroy_subscription(self.sub)

		return TransitionCallbackReturn.SUCCESS

	
	def on_shutdown(self, previous_state):
		self.get_logger().info('Shutting down')
		
		# Get rid of the timer
		self.destroy_timer(self.timer)

		# Get rid of the subscription
		self.destroy_subscription(self.sub)

		return TransitionCallbackReturn.SUCCESS


	def on_error(self, previous_state):
		self.get_logger().error('ERROR!')

		return TransitionCallbackReturn.ERROR

	# This callback will be called whenever there's a new image on the topic.
	def callback(self, msg):

		self.last_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')


	# This callback will be called every time the timer fires.
	def timer_callback(self):
		
		if self.last_img is None:
			return

		self.counter += 1

		file_name = f'{self.save_name}_{self.counter}.png'

		full_path = os.path.join(self.save_directory, file_name)

		ok = cv2.imwrite(full_path, self.last_img)

		if not ok:
			self.get_logger().error(f"Failed to save image to {file_name}")

# Basic ROS2 Setup function
def main(args=None):
	
	# Initialize rclpy.  We should do this every time.
	rclpy.init(args=args)

	# Make a node class.
	subscriber = PeriodicScreenshot()

	# Handover to ROS2
	rclpy.spin(subscriber)

	# Make sure we shutdown everything cleanly.	
	rclpy.shutdown()


# If we run the node as a script, then we're going to start here.
if __name__ == '__main__':
	
	main()