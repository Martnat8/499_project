#!/usr/bin/env python3


# Publishes camera frames on a topic raw_image_out from a UVC compiant camera.
# Is a lifecycle supported node.
#
# camera_driver.py
#
# Nathan Martin

import rclpy
import cv2
from cv_bridge import CvBridge
from rclpy.lifecycle import LifecycleNode, TransitionCallbackReturn
from sensor_msgs.msg import Image



class CameraDriver(LifecycleNode):
	def __init__(self):

		# Initialize the parent class 
		super().__init__('camera_driver')

		# Declaring parameters
		self.declare_parameter('frame_id', 'camera')	
		self.declare_parameter('device_id', 0)	
		self.declare_parameter('fps', 15.0)		
		self.declare_parameter('encoding', 'bgr8')
		self.declare_parameter('frame_width', 1280)
		self.declare_parameter('frame_height', 720)
		self.declare_parameter('topic_name', '/raw_image_out')

	def on_configure(self, previous_state):
		
		self.get_logger().info('Configuring Camera Driver')

		# Pull out parameters to determine capture parameters
		self.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value
		dev = self.get_parameter('device_id').get_parameter_value().integer_value
		fps = self.get_parameter('fps').get_parameter_value().double_value
		self.encoding_type = self.get_parameter('encoding').get_parameter_value().string_value
		width = self.get_parameter('frame_width').get_parameter_value().integer_value
		height = self.get_parameter('frame_height').get_parameter_value().integer_value
		topic = self.get_parameter('topic_name').get_parameter_value().string_value
		
		# Create a publisher, and assign it to a member variable. 
		self.pub = self.create_lifecycle_publisher(Image, topic, 10)

		# Get FPS parameter to controll timer
		timer_period = 1 / fps

		# Create a VideoCapture object
		self.capture = cv2.VideoCapture(dev, cv2.CAP_V4L2)

		if not self.capture.isOpened():
			self.get_logger().error('Failed to open camera')

			# Destroy publisher if failed
			self.destroy_lifecycle_publisher(self.pub)
		
			return TransitionCallbackReturn.FAILURE
		
		# Set capture parameters
		self.capture.set(cv2.CAP_PROP_FPS, fps)
		self.capture.set(cv2.CAP_PROP_FRAME_WIDTH, width)
		self.capture.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
		self.capture.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))

		# Use CV Bridge to convert between ROS 2 and OpenCV images
		self.bridge = CvBridge()

		# Throw away a few frames so the next happen without delay
		for _ in range(5):
			self.capture.grab()

		# Create a timer using the frequency parameter
		self.timer = self.create_timer(timer_period, self.timer_callback, autostart=False)

		return super().on_configure(previous_state)


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

		# Release the camera
		self.capture.release()

		# Get rid of the publisher.
		self.destroy_lifecycle_publisher(self.pub)

		return TransitionCallbackReturn.SUCCESS

	
	def on_shutdown(self, previous_state):
		self.get_logger().info('Shutting down')
		
		# Get rid of the timer
		self.destroy_timer(self.timer)

		# Release the camera
		self.capture.release()

		# Get rid of the publisher.
		self.destroy_lifecycle_publisher(self.pub)

		return TransitionCallbackReturn.SUCCESS


	def on_error(self, previous_state):
		self.get_logger().error('ERROR!')

		return TransitionCallbackReturn.ERROR

	# This callback will be called every time the timer fires.
	def timer_callback(self):
		
		# Capture frame by frame
		ret, frame = self.capture.read()
	
		if not ret:
			self.get_logger().warn('capture.read() returned False')
			return

		# Make an image message, and fill in the information.
		image_message = self.bridge.cv2_to_imgmsg(frame, encoding= self.encoding_type)
		image_message.header.stamp = self.get_clock().now().to_msg()
		image_message.header.frame_id = self.frame_id

		# Publish the message
		self.pub.publish(image_message)
		



def main(args=None):
	
	rclpy.init(args=args)

	publisher = CameraDriver()

	rclpy.spin(publisher)

	rclpy.shutdown()


if __name__ == '__main__':
	
	main()