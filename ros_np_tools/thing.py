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
from numpy.linalg import norm, inv
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


def get_position_impedance_control_action(T_des, K, D, ft, f_max, t_max, T_ft_ref_to_des_ref, T_ft_to_tool,
                                          T_tool_to_des, vel):
    """ Generate a modified action using a simple position impedance control scheme with a wrist-mounted ft sensor.

    # TODO add damping and mass with vel and accel terms.

    :param T_des:              Desired pose given as 4x4 transform matrix
    :param K:                    Stiffness matrix. Should be 6x6, PD, symmetric. Can be diagonal.
    :param D:                    Damping matrix. Should be 6x6, PD, symmetric. Can be diagonal.
    :param ft:                   6x1 Force torque sensor reading.
    :param f_max:                Maximum norm force allowed before modified T positions are output.
    :param t_max:                Maximum norm torque allowed before modified T rotations are output.
    :param T_ft_ref_to_des_ref:  T matrix from ref frame of ft to ref frame of mat_des.
    :param T_ft_to_tool:         fixed T matrix from ft to tool.
    :param vel:                 6 DOF velocity.
    """
    force_ref = T_ft_ref_to_des_ref[:3, :3].T.dot(ft[:3])
    torque_ref = T_ft_ref_to_des_ref[:3, :3].dot(ft[3:])

    t_norm = norm(torque_ref)
    if t_norm > t_max:

        T_des_ref_to_ft = tf_mat.invert_transform(T_ft_ref_to_des_ref)
        new_torque = inv(K[3:, 3:]).dot(ft[3:])
        new_t_norm = norm(new_torque)
        new_t_max = 1 / K[3, 3] * t_max
        R_t_ext = tf_trans.rotation_matrix(new_t_norm - new_t_max, new_torque / new_t_norm)[:3, :3]
        T_R_mod = np.eye(4)
        T_R_mod[:3, :3] = R_t_ext
        T_mod = T_des_ref_to_ft.dot(T_R_mod).dot(T_ft_to_tool).dot(T_tool_to_des)

    else:
        T_mod = T_des

    # eqn: F_ext - F_max = K (p_mod - p_des) + D (v_act - 0) --> K^-1 (F_ext - F_max - D(v_act)) + p_des = p_mod
    f_norm = norm(ft[:3])
    if f_norm > f_max:
        new_force = force_ref - (force_ref / f_norm).dot(f_max)
        # new_force = force_ref - (force_ref / f_norm).dot(f_max) - D[:3, :3].dot(vel[:3])
        # norm_new_force = norm(new_force)
        # vel_term = D[:3, :3].dot(vel[:3])
        # norm_vel_term = norm(vel_term)
        # if norm_new_force > norm_vel_term:
        #     new_force = new_force - vel_term
        # print('old new force: ', force_ref - (force_ref / f_norm).dot(f_max))
        # print('new new force: ', new_force)
        # new_force_mag = norm(new_force)
        # print(new_force_mag)
        # if new_force_mag > 5:
        #     new_force = new_force / new_force_mag * 20
        T_mod[:3, 3] = inv(K[:3, :3]).dot(new_force) + T_mod[:3, 3]

    # T_mod = np.eye(4)
    # T_mod[:3, 3] = p_mod
    # T_mod[:3, :3] = R_mod

    return T_mod