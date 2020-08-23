""" Some utility functions for getting vector numpy arrays as outputs. """

# ros
import tf2_ros
import tf.transformations as tf_trans
from geometry_msgs.msg import (Pose, Transform,
                               TransformStamped, PoseStamped,
                               WrenchStamped, Quaternion)

# others
import numpy as np


def tf_msg_to_pos_quat(tf_msg, quat_order='xyzw'):
    return np.array([tf_msg.transform.translation.x,
                     tf_msg.transform.translation.y,
                     tf_msg.transform.translation.z,
                     getattr(tf_msg.transform.rotation, quat_order[0]),
                     getattr(tf_msg.transform.rotation, quat_order[1]),
                     getattr(tf_msg.transform.rotation, quat_order[2]),
                     getattr(tf_msg.transform.rotation, quat_order[3])
    ])

def mat_to_pos_quat(mat, quat_order='xyzw'):
    pos = mat[:3, 3]
    quat = tf_trans.quaternion_from_matrix(mat)

    # tf_trans outputs xyzw order
    if quat_order != 'xyzw':
        quat_fixed = np.zeros(4)
        quat_fixed[quat_order.find('x')] = quat[0]
        quat_fixed[quat_order.find('y')] = quat[1]
        quat_fixed[quat_order.find('z')] = quat[2]
        quat_fixed[quat_order.find('w')] = quat[3]
    else:
        quat_fixed = quat

    return np.array([*pos, *quat_fixed])