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
    if hasattr(tf_msg, 'header'):  # check to see if stamped or not
        return np.array([tf_msg.transform.translation.x,
                         tf_msg.transform.translation.y,
                         tf_msg.transform.translation.z,
                         getattr(tf_msg.transform.rotation, quat_order[0]),
                         getattr(tf_msg.transform.rotation, quat_order[1]),
                         getattr(tf_msg.transform.rotation, quat_order[2]),
                         getattr(tf_msg.transform.rotation, quat_order[3])
        ])
    else:
        return np.array([tf_msg.translation.x,
                         tf_msg.translation.y,
                         tf_msg.translation.z,
                         getattr(tf_msg.rotation, quat_order[0]),
                         getattr(tf_msg.rotation, quat_order[1]),
                         getattr(tf_msg.rotation, quat_order[2]),
                         getattr(tf_msg.rotation, quat_order[3])
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

def get_trans_rot_dist(pos_quat_a, pos_quat_b):
    trans_dist = np.linalg.norm(pos_quat_a[:3] - pos_quat_b[:3])
    # rot distance, see http://www.boris-belousov.net/2016/12/01/quat-dist/ and
    # Metrics for 3D Rotations: Comparison and Analysis by Huynh
    qa = pos_quat_a[3:]
    qb = pos_quat_b[3:]
    rot_dist = 2 * np.arccos(np.clip(
        np.abs(qa[0] * qb[0] + qa[1] * qb[1] + qa[2] * qb[2] + qa[3] * qb[3]), -1.0, 1.0
    ))
    return trans_dist, rot_dist