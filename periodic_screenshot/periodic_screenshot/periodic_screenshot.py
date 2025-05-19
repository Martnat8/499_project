#!/usr/bin/env python3


# Subscribes to an image topic and saves an image to disk at a parameterized
# rate and quality
#
# camera_driver.py
#
# Nathan Martin

import rclpy
import cv2
from rclpy.node import Node
from cv_bridge import CvBridge
from rclpy.lifecycle import LifecycleNode, TransitionCallbackReturn
from sensor_msgs.msg import Image



class PeriodicScreenshot(LifecycleNode):
	def __init__(self):

		# Initialize the parent class of name oscope
		super().__init__('periodic_screenshot')

	def on_configure(self, previous_state):
		
		self.get_logger().info('Configuring')

		# Use CV Bridge to convert between ROS 2 and OpenCV images
		self.bridge = CvBridge()

		# Declaring parameters
		self.declare_parameter('rate_hz', 1280)
		self.declare_parameter('frame_width', 1280)
		self.declare_parameter('frame_height', 720)
		self.declare_parameter('topic_name', '/raw_image_out')
		self.declare_parameter('save_path', 720)
		self.declare_parameter('save_name', 'test_img')

		# Pull out parameters to determine capture parameters

		rate_hz = self.get_parameter('rate_hz').get_parameter_value().double_value
		self.width = self.get_parameter('frame_width').get_parameter_value().integer_value
		self.height = self.get_parameter('frame_height').get_parameter_value().integer_value
		self.save_path = self.get_parameter('save_path').get_parameter_value().string_value
		topic = self.get_parameter('topic_name').get_parameter_value().string_value
		save_name = self.get_parameter('save_name').get_parameter_value().string_value

		# Create the subscriber
		self.sub = self.create_subscription(Image, topic, self.callback, 10)

		# Get rate parameter to controll timer
		timer_period = 1 / rate_hz

		# Counter to track saved files
		self.counter = 0

		# Create a timer using the frequency parameter
		self.timer = self.create_timer(timer_period, self.timer_callback, autostart=False)

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

		# Get rid of the publisher.
		self.destroy_lifecycle_publisher(self.pub)

		return TransitionCallbackReturn.SUCCESS
	
	def on_shutdown(self, previous_state):
		self.get_logger().info('Shutting down')
		
		# Get rid of the timer
		self.destroy_timer(self.timer)

		return TransitionCallbackReturn.SUCCESS

	def on_error(self, previous_state):
		self.get_logger().error('ERROR!')

		return TransitionCallbackReturn.ERROR

	# This callback will be called every time the timer fires.
	def timer_callback(self, msg):
		
		cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')

		self.counter += 1

		# Need to concatinate counter and save_name and use that to save 


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