#!/usr/bin/env python3


# Subscribes to a topic and records the data to a csv.
#
# csv_writer.py
#
# Nathan Martin


import os
import rclpy
from rclpy.node import Node
from rclpy.lifecycle import LifecycleNode, TransitionCallbackReturn
from std_msgs.msg import Float32
from datetime import datetime



class CSVWriter(LifecycleNode):
	def __init__(self):

		# Initialize the parent class 
		super().__init__('csv_writer')

		# Declaring parameters
		self.declare_parameter('topic_in', '/measured_strain')
		self.declare_parameter('save_name', 'test_csv')
		self.declare_parameter('save_directory', '.')
		self.declare_parameter('save_interval', 1.0)

		self.prev_msg_data = 0.0
		self.counter = 1

	def on_configure(self, previous_state):
		
		self.get_logger().info('Configuring CSV Writer')

		# Pull out parameters to determine capture parameters
		topic = self.get_parameter('topic_in').value
		self.save_name = self.get_parameter('save_name').value
		self.save_directory = self.get_parameter('save_directory').value
		timer_interval = self.get_parameter('save_interval').value

		# Create the subscriber
		self.sub = self.create_subscription(Float32, topic, self.callback, 10)

		# Create a timer using the frequency parameter
		self.timer = self.create_timer(timer_interval, self.timer_callback, autostart=False)

		# Establish filepaths
		os.makedirs(self.save_directory, exist_ok=True)

		return TransitionCallbackReturn.SUCCESS



	def on_activate(self, previous_state):
		self.get_logger().info('Activating CSV Writer')

		# Start timer for callback
		self.timer.reset()

		return super().on_activate(previous_state)
	

	def on_deactivate(self, previous_state):
		self.get_logger().info('Deactivating CSV Writer')

		# Pause the timer
		self.timer.cancel()

		# Increment counter so we can have multiple csv files if we need. 
		self.counter += 1

		return super().on_deactivate(previous_state)
	
	
	def on_cleanup(self, previous_state):
		self.get_logger().info('Cleaning up CSV Writer')

		# Get rid of the timer
		self.timer.cancel()
		self.destroy_timer(self.timer)

		# Get rid of the subscription
		self.destroy_subscription(self.sub)

		return TransitionCallbackReturn.SUCCESS

	
	def on_shutdown(self, previous_state):
		self.get_logger().info('Shutting down CSV Writer')
		
		# Get rid of the timer
		self.timer.cancel()
		self.destroy_timer(self.timer)

		# Get rid of the subscription
		self.destroy_subscription(self.sub)

		return TransitionCallbackReturn.SUCCESS


	def on_error(self, previous_state):
		self.get_logger().error('ERROR!')

		return TransitionCallbackReturn.ERROR

	# Callback appends to a file on callback
	def callback(self, msg):

		self.prev_msg_data = msg.data

	def timer_callback(self):

		file_name = f'{self.save_name}_{self.counter}.csv'

		full_path = os.path.join(self.save_directory, file_name)

		time_at_write = datetime.now().time().isoformat()

		with open(full_path, "a") as file:
			file.write(f"{time_at_write},{self.prev_msg_data}\n")



def main(args=None):
	
	rclpy.init(args=args)

	writer = CSVWriter()

	rclpy.spin(writer)

	rclpy.shutdown()

if __name__ == '__main__':
	
	main()