# Sobel Filter

A **Sobel Filter** node that takes in an image topic and republishes the sobel x, y and xy filtered images.

## Parameters

| Name              | Type    | Default             | Description                                        |
|-------------------|---------|---------------------|----------------------------------------------------|
| `topic_in`        | string  | `/raw_image_out`    | A topic to run the filter on                       |

_____________________________________________________________________________________
_____________________________________________________________________________________
Maintainer - Nathan Martin - martnat8@oregonstate.edu
_____________________________________________________________________________________
License - BSD 3-Clause
_____________________________________________________________________________________
References:
https://docs.opencv.org/4.x/d2/d2c/tutorial_sobel_derivatives.html
Lifecycle node structure influenced largly from class supplied code