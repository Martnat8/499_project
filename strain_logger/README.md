# Strain Logger

A **Strain Logger** node that subscribes to an image topic from a OpenCV Canny Edge Detection node. Calculates
the distance between two prominant contours. And given the subjects original length publishes the strain of 
the specimen.

This node is mostly specific to the usecase that I have with my material testing platform. There is an expectation
of a workspace with minimal visual noise and two prominant horizontal edges.

## Parameters


| Name              | Type    | Default             | Description                                                  |
|-------------------|---------|---------------------|--------------------------------------------------------------|
| `topic_in`        | string  | `/canny_edge_out`   | The input image topic to process                             |
| `pix_to_mm`       | double  | `0.1`               | Conversion factor from pixels to millimeters                 |
| `roi_width`       | int     | `320`               | Width of the region of interest (ROI) in pixels              |
| `roi_height`      | int     | `480`               | Height of the region of interest (ROI) in pixels             |
| `original_length` | double  | `10.0`              | Original length of the measured feature, used for strain calc|


_____________________________________________________________________________________
_____________________________________________________________________________________
Maintainer - Nathan Martin - martnat8@oregonstate.edu
_____________________________________________________________________________________
License - BSD 3-Clause
_____________________________________________________________________________________
References:
https://docs.opencv.org/4.x/da/d22/tutorial_py_canny.html
https://docs.opencv.org/3.4/d4/d73/tutorial_py_contours_begin.html
https://docs.opencv.org/3.4/d3/dc0/group__imgproc__shape.html#ga819779b9857cc2f8601e6526a3a5bc71
Lifecycle node structure influenced largely from class supplied code