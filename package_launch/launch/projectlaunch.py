# The launch file for ROB499 Final Project
#
# projectlaunch.py
#
# Nathan Martin



import os
import launch
import launch_ros.actions




def generate_launch_description():
    return launch.LaunchDescription([

        # Launch and adjust parameters for Lifecycle_Coordinator
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
                'fps': 15.0,
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
