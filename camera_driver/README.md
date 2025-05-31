# ROB 499 Robot Software Frameworks Project

This package contains a lifecycle controlled camera driver node inteded to be used 
with a UVC compliant camera.
_____________________________________________________________________________________

# Parameters and their defaults:
| Parameter      | Type    | Default           | Description                                       |
| -------------- | ------- | ----------------- | ------------------------------------------------- |
| `frame_id`     | string  | `"camera"`        | TF frame ID to stamp on each published image.     |
| `device_id`    | integer | `0`               | Index of the V4L2 camera device (`/dev/videoN`).  |
| `fps`          | float   | `15.0`            | Capture rate in frames per second.                |
| `encoding`     | string  | `"bgr8"`          | OpenCV/ROS color encoding for image messages.     |
| `frame_width`  | integer | `1280`            | Width of the captured frames in pixels.           |
| `frame_height` | integer | `720`             | Height of the captured frames in pixels.          |
| `topic_name`   | string  | `"/raw_image_out"`| ROS topic on which to publish raw images.         |

_____________________________________________________________________________________
Maintainer - Nathan Martin - martnat8@oregonstate.edu
_____________________________________________________________________________________
License - BSD 3-Clause
_____________________________________________________________________________________
References:
https://docs.opencv.org/4.x/dd/d43/tutorial_py_video_display.html
https://docs.opencv.org/3.4/d8/dfe/classcv_1_1VideoCapture.html

Lifecycle node structure influenced largly from class supplied code


