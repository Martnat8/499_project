#!/usr/bin/env python3


# This is a ROS2 node that subscribes to an image topic published by an OpenCV
# canny edge publisher. Calculates the distance between two contours and publishes
# the annotated image and the distance value.
#
# strain_logger.py
#
# Nathan Martin



import rclpy
import cv2 as cv
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from std_msgs.msg import Float32
from rclpy.lifecycle import LifecycleNode, TransitionCallbackReturn


class StrainLogger(LifecycleNode):
	def __init__(self):

		# Initialize the parent class
		super().__init__('strain_logger')

		# Declaring parameters
		self.declare_parameter('topic_in', '/canny_edge_out')
		self.declare_parameter('pix_to_mm', 0.1)
		self.declare_parameter('roi_width', 320)
		self.declare_parameter('roi_height', 480)
		self.declare_parameter('original_length', 10.0)


		# Boolean so that subscriber/publisher is controlled by lifecycle
		self.is_active = False

	def on_configure(self, previous_state):
		
		self.get_logger().info('Configuring Strain Logger')

		

		# Pull out parameters
		topic_in = self.get_parameter('topic_in').value
		self.pix_to_mm = self.get_parameter('pix_to_mm').value
		self.w = self.get_parameter('roi_width').value
		self.h = self.get_parameter('roi_height').value
		self.og_length = self.get_parameter('original_length').value

		# Create the subscriber and publisher
		self.sub = self.create_subscription(Image, topic_in, self.callback, 1)
		self.pub = self.create_lifecycle_publisher(Image, '/strain_roi_image', 10)
		self.pubdist = self.create_lifecycle_publisher(Float32, '/measured_strain', 10)

		# Use CV Bridge to convert between ROS 2 and OpenCV images
		self.bridge = CvBridge()

		return TransitionCallbackReturn.SUCCESS

	def on_activate(self, previous_state):
		self.get_logger().info('Activating Strain Logger')

		self.is_active = True

		return super().on_activate(previous_state)
	

	def on_deactivate(self, previous_state):
		self.get_logger().info('Deactivating Strain Logger')

		self.is_active = False

		return super().on_deactivate(previous_state)
	
	
	def on_cleanup(self, previous_state):
		self.get_logger().info('Cleaning up')

		# Destroy subscriber and publishers
		self.destroy_subscription(self.sub)
		self.destroy_publisher(self.pub)
		self.destroy_publisher(self.pubdist)

		return TransitionCallbackReturn.SUCCESS

	
	def on_shutdown(self, previous_state):
		self.get_logger().info('Shutting down')
		

		# Destroy subscriber and publishers		
		self.destroy_subscription(self.sub)
		self.destroy_publisher(self.pub)
		self.destroy_publisher(self.pubdist)

		return TransitionCallbackReturn.SUCCESS


	def on_error(self, previous_state):
		self.get_logger().error('ERROR!')

		return TransitionCallbackReturn.ERROR
	
	# Callback for the subscriber
	def callback(self, msg):

		# Crop incoming image to the desired size
		img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')

		# Center the region of interest
		height, width = img.shape[:2]
		roi_x = (width - self.w) // 2
		roi_y = (height - self.h) // 2

		roi = img[roi_y:roi_y + self.h, roi_x:roi_x + self.w]

		# Dilate edges to make contours more prominent
		dilated = cv.dilate(roi, None, iterations=1)

		# Use opencv find contours function
		contours, _ = cv.findContours(dilated, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
		sorted_contours = sorted(contours, key=lambda c: cv.boundingRect(c)[1])

		# Safety check for enough contours
		if len(sorted_contours) < 2:
			self.get_logger().warn('Not enough contours found.')
			image_message = self.bridge.cv2_to_imgmsg(dilated, encoding= 'mono8')
			self.pub.publish(image_message)
			return

		# Find top and bottom countours
		y_top = cv.boundingRect(sorted_contours[0])[1]
		y_bottom = cv.boundingRect(sorted_contours[-1])[1]
		pixel_distance = abs(y_bottom - y_top)

		# Convert pixels to distance and publish
		msg_dist = Float32()
		msg_dist.data = (pixel_distance * self.pix_to_mm) / self.og_length
		self.pubdist.publish(msg_dist)

		# Draw contours on incoming message and republish
		roi_color = cv.cvtColor(roi, cv.COLOR_GRAY2BGR)
		cv.drawContours(roi_color, sorted_contours, -1, (0, 255, 0), 2) 
		image_message = self.bridge.cv2_to_imgmsg(roi_color, encoding= 'bgr8')
		self.pub.publish(image_message)



def main(args=None):
	
	rclpy.init(args=args)

	strain = StrainLogger()

	rclpy.spin(strain)

	rclpy.shutdown()

if __name__ == '__main__':
	
	main()