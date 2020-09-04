""" Tools involving 4x4 homogenous transform matrices """

# ros
import tf2_ros
import tf.transformations as tf_trans
from geometry_msgs.msg import (Pose, Transform,
                               TransformStamped, PoseStamped,
                               WrenchStamped, Quaternion)

# others
import numpy as np

# internal
from ros_np_tools.pose_msg import tf_msg_to_pose_msg


def pose_msg_to_mat(pose):
    if hasattr(pose, 'header'):  # in case pose is a stamped msg
        pose = pose.pose
    T = np.zeros([4, 4])
    T[3, 3] = 1.0
    T[0, 3] = pose.position.x
    T[1, 3] = pose.position.y
    T[2, 3] = pose.position.z
    quat = np.ndarray([4, ])
    quat[0] = pose.orientation.x
    quat[1] = pose.orientation.y
    quat[2] = pose.orientation.z
    quat[3] = pose.orientation.w
    rot = tf_trans.quaternion_matrix(quat)
    T[:3, :3] = rot[:3, :3]
    return T

def invertTransform(T_in):
    """return inverse transform of T_in, assuming T_in is affine 4x4 transformation"""
    T_out = np.eye(4)
    C_out_inv = T_in[:3, :3].T # exploiting that C^T = C^-1 for rotations
    T_out[:3, :3] = C_out_inv
    T_out[:3, 3] = -C_out_inv.dot(T_in[:3, 3])
    return T_out

def tf_msg_to_mat(tf_msg):
    if hasattr(tf_msg, 'header'):
        return pose_msg_to_mat(tf_msg_to_pose_msg(tf_msg).pose)
    else:
        return pose_msg_to_mat(tf_msg_to_pose_msg(tf_msg))

def pos_quat_to_mat(pos_quat):
    mat = np.eye(4)
    mat[:3, 3] = pos_quat[:3]
    mat[:3, :3] = tf_trans.quaternion_matrix(pos_quat[3:])[:3, :3]
    return mat

def pos_eul_to_mat(pos_eul, eul_axes='sxyz'):
    mat = np.eye(4)
    mat[:3, 3] = pos_eul[:3]
    mat[:3, :3] = tf_trans.euler_matrix(*pos_eul[3:], axes=eul_axes)[:3, :3]
    return mat