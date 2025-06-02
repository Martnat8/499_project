#!/usr/bin/env python3


# This is a ROS2 node that subscribes to an image topic published by an OpenCV
# canny edge publisher. Calculates the distance between two contours and records
# strain to a csv.
#
# strain_logger.py
#
# Nathan Martin



import rclpy
import cv2 as cv
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from rclpy.lifecycle import LifecycleNode, TransitionCallbackReturn


class StrainLogger(LifecycleNode):
	def __init__(self):

		# Initialize the parent class
		super().__init__('strain_logger')

		# Declaring parameters
		self.declare_parameter('topic_in', '/raw_image_out')
		self.declare_parameter('pix_to_mm', 0.1)
		self.declare_parameter('region_of_interest', )

		# Boolean so that subscriber/publisher is controlled by lifecycle
		self.is_active = False

	def on_configure(self, previous_state):
		
		self.get_logger().info('Configuring Strain Logger')

		# Pull out parameters
		topic_in = self.get_parameter('topic_in').value

		# Create the subscriber and publisher
		self.sub = self.create_subscription(Image, topic_in, self.callback, 1)
		self.pub = self.create_lifecycle_publisher(Image, '/strain_roi_image', 10)

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
	
	# Callback for the subscriber
	def callback(self, msg):

		if self.is_active:

		

			# Publish the message
			self.pub.publish(msg)



def main(args=None):
	
	rclpy.init(args=args)

	strain = StrainLogger()

	rclpy.spin(strain)

	rclpy.shutdown()

if __name__ == '__main__':
	
	main()