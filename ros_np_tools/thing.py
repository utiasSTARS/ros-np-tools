""" Generic ros tools specifically for use with the Thing at UTIAS. """

# ros
import rospy
import tf
import tf.transformations as tf_trans
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import Transform

# other
import numpy as np
from ros_np_tools import pos_quat_np


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
