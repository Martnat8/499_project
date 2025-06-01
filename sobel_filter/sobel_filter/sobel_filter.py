#!/usr/bin/env python3


# This is a ROS2 node that subscribes to an image topic,
# runs OpenCV sobel filter and then republishes the image.
#
# sobel_filter.py
#
# Nathan Martin



import rclpy
import cv2 as cv
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

		# Boolean so that subscriber/publisher is controlled by lifecycle
		self.is_active = False

	def on_configure(self, previous_state):
		
		self.get_logger().info('Configuring Sobel Filter')

		# Pull out parameters
		topic_in = self.get_parameter('topic_in').value

		# Create the subscriber and publisher
		self.sub = self.create_subscription(Image, topic_in, self.callback, 10)
		self.pubx = self.create_lifecycle_publisher(Image, '/sobelX_out', 10)
		self.puby = self.create_lifecycle_publisher(Image, '/sobely_out', 10)
		self.pubxy = self.create_lifecycle_publisher(Image, '/sobelxy_out', 10)

		# Use CV Bridge to convert between ROS 2 and OpenCV images
		self.bridge = CvBridge()

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
		self.destroy_publisher(self.pubx)
		self.destroy_publisher(self.puby)
		self.destroy_publisher(self.pubxy)

		return TransitionCallbackReturn.SUCCESS

	
	def on_shutdown(self, previous_state):
		self.get_logger().info('Shutting down')
		

		# Destroy subscriber and publisher		
		self.destroy_subscription(self.sub)
		self.destroy_publisher(self.pubx)
		self.destroy_publisher(self.puby)
		self.destroy_publisher(self.pubxy)

		return TransitionCallbackReturn.SUCCESS


	def on_error(self, previous_state):
		self.get_logger().error('ERROR!')

		return TransitionCallbackReturn.ERROR
	
	# Callback for the subscriber
	def callback(self, msg):

		if self.is_active:

			# Convert image message to cv2 image
			img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='mono8')

			# Protect against bad image loads
			if img is None:
				self.get_logger().warning('Error getting image for Sobel filter')
				return

			img = cv.GaussianBlur(img, (3,3), 0)

			gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)

			sobelx = cv.Sobel(gray, cv.CV_16S, 0, 1, ksize=3)
			sobely = cv.Sobel(gray, cv.CV_16S, 1, 0, ksize=3)

			abs_grad_x = cv.convertScaleAbs(sobelx)
			abs_grad_y = cv.convertScaleAbs(sobely)

			grad_xy = cv.addWeighted(abs_grad_x, 0.5, abs_grad_y, 0.5, 0)

			# Convert back to messages to publish

			sobelx_msg = self.bridge.cv2_to_imgmsg(abs_grad_x, encoding= 'mono8')
			sobely_msg = self.bridge.cv2_to_imgmsg(abs_grad_y, encoding= 'mono8')
			sobelxy_msg = self.bridge.cv2_to_imgmsg(grad_xy, encoding= 'mono8')

			# Publish the message
			self.pubx.publish(sobelx_msg)
			self.puby.publish(sobely_msg)
			self.pubxy.publish(sobelxy_msg)



def main(args=None):
	
	rclpy.init(args=args)

	filtered = SobelFilter()

	rclpy.spin(filtered)

	rclpy.shutdown()

if __name__ == '__main__':
	
	main()