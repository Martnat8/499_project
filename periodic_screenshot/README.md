# Periodic Screenshot

A **Periodic Screenshot** node that subscribes to a ROS image topic and saves the latest frame to disk at a configurable time interval. Useful for capturing snapshots over long runs without storing full video.

## Parameters

| Name              | Type    | Default             | Description                                        |
|-------------------|---------|---------------------|----------------------------------------------------|
| `topic_name`      | string  | `/raw_image_out`    | Image topic to subscribe for screenshots.          |
| `timer_interval`  | int     | `30`                | Time between saves, in seconds.                    |
| `save_name`       | string  | `test_img`          | Base filename for saved images (suffix `_N.png`).  |
| `save_directory`  | string  | `.`                 | Directory where screenshot files will be written.  |
_____________________________________________________________________________________
_____________________________________________________________________________________
Maintainer - Nathan Martin - martnat8@oregonstate.edu
_____________________________________________________________________________________
License - BSD 3-Clause
_____________________________________________________________________________________
References:
https://www.geeksforgeeks.org/python-os-path-join-method/
Lifecycle node structure influenced largly from class supplied code