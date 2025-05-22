#!/usr/bin/env python3


# Subscribes to an image topic and video to disk of a certain length
# Appends and removes frames from the file on disk so that it's safe
# from the node crashing
#
# camera_driver.py
#
# Nathan Martin


import cv2
import os
import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
from collections import deque
from rclpy.lifecycle import LifecycleNode, TransitionCallbackReturn
from sensor_msgs.msg import Image



class PeriodicScreenshot(LifecycleNode):
	def __init__(self):

		# Initialize the parent class of name oscope
		super().__init__('ring_buffer_recorder')

		# Declaring parameters
		self.declare_parameter('topic_name', '/raw_image_out')
		self.declare_parameter('fps', 15)  # Should match camera_driver output
		self.declare_parameter('video_length_s', 60)
		self.declare_parameter('num_videos', 3)
		self.declare_parameter('codec', 'MJPEG')
		self.declare_parameter('save_name', 'test_video')
		self.declare_parameter('save_directory', '.')

	def on_configure(self, previous_state):
		
		self.get_logger().info('Configuring Ring Buffer Recorder')

		# Use CV Bridge to convert between ROS 2 and OpenCV images
		self.bridge = CvBridge()

		# Pull out parameters to determine capture parameters
		topic = self.get_parameter('topic_name').value
		fps = self.get_parameter('fps').value
		video_length = self.get_parameter('video_length_s').value
		self.num_videos = self.get_parameter('num_videos').value
		self.codec = self.get_parameter('codec').value
		self.save_name = self.get_parameter('save_name').value
		self.save_directory = self.get_parameter('save_directory').value

		self.is_recording = False

		# Num frames is used to control the length of each saved video
		num_frames = fps * video_length 

		# We'll use deque to controll the buffer
		self.video_buffer = deque(maxlen= num_frames)

		# Create the subscriber
		self.sub = self.create_subscription(Image, topic, self.callback, 10)

		# Establish filepaths
		os.makedirs(self.save_directory, exist_ok=True)

		return TransitionCallbackReturn.SUCCESS



	def on_activate(self, previous_state):
		self.get_logger().info('Activating')

		# Turn on video recording to disk
		self.is_recording = True

		return super().on_activate(previous_state)
	

	def on_deactivate(self, previous_state):
		self.get_logger().info('Deactivating')

		# Turn off video recording to disk
		self.is_recording = False

		return super().on_deactivate(previous_state)
	
	
	def on_cleanup(self, previous_state):
		self.get_logger().info('Cleaning up')

		self.destroy_subscription(self.sub)

		return TransitionCallbackReturn.SUCCESS

	
	def on_shutdown(self, previous_state):
		self.get_logger().info('Shutting down')
		
		self.destroy_subscription(self.sub)

		return TransitionCallbackReturn.SUCCESS


	def on_error(self, previous_state):
		self.get_logger().error('ERROR!')

		return TransitionCallbackReturn.ERROR

	
	def callback(self, msg):

		if self.is_recording:

			# Grab the frame from message and convert back to OpenCV format
			frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
			self.buffer.append(frame)



def main(args=None):
	
	rclpy.init(args=args)

	subscriber = PeriodicScreenshot()

	rclpy.spin(subscriber)

	rclpy.shutdown()

if __name__ == '__main__':
	
	main()