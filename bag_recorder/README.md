# Bag Recorder

A **Bag Recorder** node that can take in a list of topics and record a bag from those topics. Is lifecycle
supported. If you pause the lifecycle node it will increment the bag file name so that they won't override.

## Parameters

| Name              | Type    | Default             | Description                                        |
|-------------------|---------|---------------------|----------------------------------------------------|
| `topics`          | list    | [`/raw_image_out`]  | List of topics to record a bag of                  |
| `bag_name      `  | string  | `test_bag`          | Base name for the bag file                         |
| `save_directory`  | string  | `.`                 | Directory where bag files will be stored.          |
_____________________________________________________________________________________
_____________________________________________________________________________________
Maintainer - Nathan Martin - martnat8@oregonstate.edu
_____________________________________________________________________________________
License - BSD 3-Clause
_____________________________________________________________________________________
References:

Lifecycle node structure influenced largly from class supplied code