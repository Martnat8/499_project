#!/usr/bin/env python3


# This is a ROS2 node that subscribes to an image topic,
# runs OpenCV sobel filter and then republishes the image.
#
# sobel_filter.py
#
# Nathan Martin



import rclpy
import cv2
import os
import numpy as np
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from rclpy.lifecycle import LifecycleNode, TransitionCallbackReturn


class SobelFilter(LifecycleNode):
	def __init__(self):

		# Initialize the parent class of name oscope
		super().__init__('sobel_filter')

		# Declaring parameters
		self.declare_parameter('topic_in', '/raw_image_out')
		self.declare_parameter('save_name', 'sobel_test_img')
		self.declare_parameter('save_directory', '.')
		self.declare_parameter('save_to_disk', False)

		# Boolean so that subscriber/publisher is controlled by lifecycle
		self.is_active = False

	def on_configure(self, previous_state):
		
		self.get_logger().info('Configuring Sobel Filter')

		# Pull out parameters
		topic_in = self.get_parameter('topic_in').value
		self.save_name = self.get_parameter('save_name').value
		self.save_directory = self.get_parameter('save_directory').value
		self.is_saving = self.get_parameter('save_to_disk').value

		# Create the subscriber and publisher
		self.sub = self.create_subscription(Image, topic_in, self.callback, 10)
		self.pub = self.create_lifecycle_publisher(Image, '/sobelX_out', 10)
		self.pub = self.create_lifecycle_publisher(Image, '/sobely_out', 10)
		self.pub = self.create_lifecycle_publisher(Image, '/sobelxy_out', 10)

		# Establish filepaths
		os.makedirs(self.save_directory, exist_ok=True)

		return TransitionCallbackReturn.SUCCESS

	def on_activate(self, previous_state):
		self.get_logger().info('Activating Sobel Filter')

		self.is_active = True

		return super().on_activate(previous_state)
	

	def on_deactivate(self, previous_state):
		self.get_logger().info('Deactivating Sobel Filter')

		self.is_active = False

		return super().on_deactivate(previous_state)
	
	
	def on_cleanup(self, previous_state):
		self.get_logger().info('Cleaning up')

		# Destroy subscriber and publisher
		self.destroy_subscription(self.sub)
		self.destroy_publisher(self.pub)

		return TransitionCallbackReturn.SUCCESS

	
	def on_shutdown(self, previous_state):
		self.get_logger().info('Shutting down')
		

		# Destroy subscriber and publisher		
		self.destroy_subscription(self.sub)
		self.destroy_publisher(self.pub)

		return TransitionCallbackReturn.SUCCESS


	def on_error(self, previous_state):
		self.get_logger().error('ERROR!')

		return TransitionCallbackReturn.ERROR
	
		# Callback saves each image in memory to save with timer callback later
	def callback(self, msg):

		if self.is_active:

			

			img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')

			if self.is_saving:

				file_name = f'{self.bag_name}_{self.counter}'

				self.bag_path = os.path.join(self.save_directory, file_name)

				# Finish saving functionalty

			self.last_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')



def main(args=None):
	
	rclpy.init(args=args)

	filtered = SobelFilter()

	rclpy.spin(filtered)

	rclpy.shutdown()

if __name__ == '__main__':
	
	main()