""" Tools for getting euler angles as output """
# ros
import tf2_ros
import tf.transformations as tf_trans
import tf_conversions
from geometry_msgs.msg import (Pose, Transform,
                               TransformStamped, PoseStamped,
                               WrenchStamped, Quaternion)

# others
import numpy as np


def ros_quat_to_euler(q, axes='sxyz'):
    # get the SXYZ euler angles from a ros XYZW quaternion
    np_q = np.array([q.x, q.y, q.z, q.w])
    return tf_conversions.transformations.euler_from_quaternion(np_q, axes)

def tf_msg_to_eul(tf_msg, axes='sxyz'):
    if hasattr(tf_msg, 'header'):
        rot = tf_msg.transform.rotation
    else:
        rot = tf_msg.rotation
    return tf_trans.euler_from_quaternion([rot.x, rot.y, rot.z, rot.w], axes=axes)