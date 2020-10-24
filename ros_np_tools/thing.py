""" Generic ros tools specifically for use with the Thing at UTIAS. """

# ros
import rospy
import tf
import tf.transformations as tf_trans
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import Transform, PoseStamped
from nav_msgs.msg import Path

# other
import numpy as np
from ros_np_tools import pos_quat_np, tf_mat, pose_msg


def get_servo_msg(mat=None, tf_msg=None, base_mat=None, base_tf_msg=None):
    """ Prepare a message for the thing servoCallback method based on desired poses.
    All poses are relative to the odom frame, as designed in thing_control. """
    assert ((mat is not None or tf_msg is not None) and (base_mat is not None or base_tf_msg is not None)), \
        "One of mat and tf_msg as well as base_mat and base_tf_msg must be set."
    assert ((mat is None or tf_msg is None) and (base_mat is None or base_tf_msg is None)), \
        "Only one of mat and tf_msg as well as base_mat and base_tf_msg can be set"

    servo_msg = Float64MultiArray()
    if mat is not None:
        arr = pos_quat_np.mat_to_pos_quat(mat)
    elif tf_msg is not None:
        arr = pos_quat_np.tf_msg_to_pos_quat(tf_msg)
    servo_msg.data.extend(arr)

    if base_mat is not None:
        base_xy = base_mat[:2, 3]
        base_theta = tf_trans.euler_from_matrix(base_mat)[2]
    elif base_tf_msg is not None:
        base_arr = pos_quat_np.tf_msg_to_pos_quat(base_tf_msg)
        base_xy = base_arr[:2]
        base_theta = tf_trans.euler_from_quaternion(base_arr[3:])[2]
    servo_msg.data.extend([*base_xy, base_theta])

    return servo_msg


def gen_no_movement_base_path(arm_path, odom_base_mat):
    """ Generate a base path that corresponds to an arm path, ensuring that the base does not move. """
    des_base_pose = PoseStamped()
    des_base_pose.pose = pose_msg.mat_to_pose_msg(odom_base_mat)
    base_path = Path()
    base_path.poses = [des_base_pose] * len(arm_path.poses)
    return base_path


def gen_no_movement_arm_path(base_path, base_tool_mat):
    """ Generate an arm path that corresponds to a base path ensuring a constant
    transform between the base and the arm. As with other thing functions, assumes that IK
    is done relative to odom. """
    T_rt = base_tool_mat
    arm_path = Path()
    for pose in base_path.poses:
        base_pose_mat = tf_mat.pose_msg_to_mat(pose.pose)
        arm_pose = PoseStamped()
        arm_pose.pose = pose_msg.mat_to_pose_msg(np.dot(base_pose_mat, T_rt))
        arm_path.poses.append(arm_pose)

    return arm_path