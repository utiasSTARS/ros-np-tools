# ros-np-tools

A set of reusable functions and classes for reducing boilerplate ROS and numpy code.

In particular, can be useful for converting between ROS and numpy types,  converting between different ROS types, and updating ros transforms.

This is a living library, so the signatures of existing functions will not be changed, but new functions and/or modules will be periodically added.

## Installation
Ensure that a system-wide ros installation already exists. Then,
```
pip install -e .
```

## Usage
Each file is named based on the output of each of the functions it contains. For example, to convert from a 4x4 transform matrix to a Transform msg, you would call `ros_np_tools.tf_msg.mat_to_tf_msg`.