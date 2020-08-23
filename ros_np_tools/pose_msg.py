""" Some utility functions for getting ros poses as outputs. """

# ros
import tf2_ros
import tf.transformations as tf_trans
from geometry_msgs.msg import (Pose, Transform,
                               TransformStamped, PoseStamped,
                               WrenchStamped, Quaternion)

# others
import numpy as np


def mat_to_pose_msg(mat):
    p = Pose()
    p.position.x = mat[0, 3]
    p.position.y = mat[1, 3]
    p.position.z = mat[2, 3]
    quat = tf_trans.quaternion_from_matrix(mat)
    p.orientation.x = quat[0]
    p.orientation.y = quat[1]
    p.orientation.z = quat[2]
    p.orientation.w = quat[3]
    return p

def tf_msg_to_pose_msg(tf_msg):
    if hasattr(tf_msg, 'header'):  # check to see if stamped or not
        p = PoseStamped()
        p.header = tf_msg.header
        p.pose.position.x = tf_msg.transform.translation.x
        p.pose.position.y = tf_msg.transform.translation.y
        p.pose.position.z = tf_msg.transform.translation.z
        p.pose.orientation.x = tf_msg.transform.rotation.x
        p.pose.orientation.y = tf_msg.transform.rotation.y
        p.pose.orientation.z = tf_msg.transform.rotation.z
        p.pose.orientation.w = tf_msg.transform.rotation.w
    else:
        p = Pose()
        p.position.x = tf_msg.translation.x
        p.position.y = tf_msg.translation.y
        p.position.z = tf_msg.translation.z
        p.orientation.x = tf_msg.rotation.x
        p.orientation.y = tf_msg.rotation.y
        p.orientation.z = tf_msg.rotation.z
        p.orientation.w = tf_msg.rotation.w
    return p