# The launch file for camera_driver
#
# cameralaunch.py
#
# Nathan Martin


# We need to import the launch system modules.  There's a generic launch
# system, in launch, and some ROS-specific stuff, in launch_ros.]
import os
import launch
import launch_ros.actions



# You need to define this function, which is loaded by the launch system.  The function
# returns a list of nodes that you want to run.
def generate_launch_description():
    return launch.LaunchDescription([

        launch_ros.actions.Node(
            package='lifecycle_coordinator',
            executable='lifecycle_coordinator',
            parameters= [{
                # Here you can add the node names you want launched with the coordinator
                'node_names': [
                    'camera_driver',
                    'periodic_screenshot',
                    'ring_buffer_recorder',
                    ],
            }],          
        ),

        # Launch and adjust parameters for Camera_Driver
        launch_ros.actions.Node(
            package='camera_driver',
            executable='camera_driver',
            parameters= [{
                'fps': 15.0,
                'encoding': 'bgr8',
                'frame_width': 1280,
                'frame_height': 720,
                'device_id': 0,
                'frame_id' : 'camera',
                'topic_name': '/raw_image_out'
            }],          
        ),

        # Launch and adjust parameters for Ring_Buffer_Recorder
        launch_ros.actions.Node(
            package='ring_buffer_recorder',
            executable='ring_buffer_recorder',
            parameters= [{
                'topic_name': '/raw_image_out',
                'fps': 15,
                'video_length_s': 60, 
                'codec': 'MJPG',
                'save_name': 'test_video',
                'save_directory': os.path.expanduser('~/junk_imgs'), 
            }],          
        ),

        # Launch and adjust parameters for Periodic_Screenshot
        launch_ros.actions.Node(
            package='periodic_screenshot',
            executable='periodic_screenshot',
            parameters= [{
                'topic_name': '/raw_image_out',
                'timer_interval': 10,
                'save_name': 'test_img',
                'save_directory': os.path.expanduser('~/junk_imgs'),
            }],          
        ),

        launch_ros.actions.Node(
            package='rviz2',
            executable='rviz2',         
        ),

        ])
