""" Some utility functions for getting quaternions as outputs """
# ros
import tf2_ros
import tf.transformations as tf_trans
import tf_conversions
from geometry_msgs.msg import (Pose, Transform,
                               TransformStamped, PoseStamped,
                               WrenchStamped, Quaternion)

# others
import numpy as np


def euler_to_ros_quat(e, axes='sxyz'):
    # get a ROS XYZW quaternion from an SXYZ euler
    np_q = tf_conversions.transformations.quaternion_from_euler(*e, axes)
    return np_quat_to_ros_quat(np_q)

def ros_quat_to_np_quat(q):
    q = np.array([q.x, q.y, q.z, q.w])
    return q

def np_quat_to_ros_quat(np_q, quat_order_in='xyzw'):
    q = Quaternion()
    q.x = np_q[quat_order_in.find('x')]
    q.y = np_q[quat_order_in.find('y')]
    q.z = np_q[quat_order_in.find('z')]
    q.w = np_q[quat_order_in.find('w')]
    return q