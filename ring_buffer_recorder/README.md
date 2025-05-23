# Ring Buffer Recorder

A **Ring Buffer Recorder** node that subscribes to a raw image topic and keeps the last _N_ seconds of video frames in memory.  When triggered via its `/save_request` service, it dumps the buffered frames to disk as a video file.

## Parameters

| Name              | Type   | Default           | Description                                      |
|-------------------|--------|-------------------|--------------------------------------------------|
| `topic_name`      | string | `/raw_image_out`  | ROS image topic to subscribe for raw frames.     |
| `fps`             | int    | `15`              | Expected frame rate (frames per second).         |
| `video_length_s`  | int    | `60`              | Length of the in-memory buffer in seconds.       |
| `codec`           | string | `MJPG`            | FourCC code for the output video file.           |
| `save_name`       | string | `test_video`      | Base name for dumped video files.                |
| `save_directory`  | string | `.`               | Directory path where video files will be saved.  |


# Service Trigger

To trigger the service that saves video to disk use the following command:

ros2 service call /save_request project_interfaces/srv/SaveSrv "{}"
_____________________________________________________________________________________
_____________________________________________________________________________________
Maintainer - Nathan Martin - martnat8@oregonstate.edu
_____________________________________________________________________________________
License - BSD 3-Clause
_____________________________________________________________________________________
References:
https://stackoverflow.com/questions/29317262/opencv-video-saving-in-python
https://www.geeksforgeeks.org/python-opencv-capture-video-from-camera/
https://docs.opencv.org/4.x/dd/d43/tutorial_py_video_display.html
https://docs.python.org/3/library/collections.html#collections.deque
Lifecycle node structure influenced largly from class supplied code
ChatGPT for debugging and syntax help