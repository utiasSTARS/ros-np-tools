# ros-np-tools

A set of reusable functions and classes for reducing boilerplate ROS and numpy code.

In particular, can be useful for converting between ROS and numpy types,  converting between different ROS types, and updating ros transforms.

This is a living library, so the signatures of existing functions will not be changed, but new functions and/or modules will be periodically added.

## Installation
Ensure that a system-wide ros installation already exists. Then,
```
pip install -e .
```

### For virtualenv/conda
If you're using virtualenv/conda for python, you should be okay as long as the following considerations are met:

- Your `PYTHONPATH` should contain `/opt/ros/ros_dist/lib/python3/dist-packages`, by default this is done when you call `source /opt/ros/ros_dist/setup.bash`, which should be in your `.bashrc` after installing ROS.
- Some python packages are not installed in `/opt/ros/...`, but rather `/usr/lib/python3/dist-packages`. To access these libraries in your virtual/conda env, you need to install them manually (since adding `/usr/lib/python3/dist-packages` to your `PYTHONPATH` doesn't work, nor should it). Most are installed automatically via the `setup.py` requirements, but unfortunately the version of `PyKDL` automatically installed via `pip` is the wrong one, so you must install a custom wheel as follows:
    - Install the custom wheel for your machine and python version from [here](https://rospypi.github.io/simple/pykdl/). Download the `.whl` file and install with `pip install .`. Thanks to [@otamachan](https://github.com/otamachan) for setting this up.

## Usage
Each file is named based on the output of each of the functions it contains. For example, to convert from a 4x4 transform matrix to a Transform msg, you would call `ros_np_tools.tf_msg.mat_to_tf_msg`.