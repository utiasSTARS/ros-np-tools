""" Utilities for creating smooth paths. """

# ros
import rospy
from geometry_msgs.msg import Quaternion, PoseStamped
from nav_msgs.msg import Path

# other
import numpy as np

# own
from ros_np_tools.pose_msg import tf_msg_to_pose_msg

def quat_slerp(qa, qb, t):
    """given two quaternions, output the interpolation at t, assuming qa is at t=0 and qb is at t=1.
    Note: all quaternions in this function assumed to be geometry_msgs.msg Quaternions"""
    # source: http://www.euclideanspace.com/maths/algebra/realNormedAlgebra/quaternions/slerp/

    qm = Quaternion()

    # interested to know where the math for this comes from
    cos_half_theta = qa.w * qb.w + qa.x * qb.x + qa.y * qb.y + qa.z * qb.z

    # if qa = qb or qa = -qb, theta = 0 (cos(half_theta) = 1), so return qa
    if np.abs(cos_half_theta) >= 1.0:
        qm.w = qa.w
        qm.x = qa.x
        qm.y = qa.y
        qm.z = qa.z
        return qm

    # note: this will change the actual qb object that is passed in, which should be fine, but is worth mentioning
    if cos_half_theta < 0:
        qb.w = -qb.w
        qb.x = -qb.x
        qb.y = -qb.y
        qb.z = -qb.z
        cos_half_theta = -cos_half_theta

    half_theta = np.arccos(cos_half_theta)
    sin_half_theta = np.sqrt(1.0 - cos_half_theta * cos_half_theta)

    # if theta = 180, result is not fully defined, can rotate about any axis normal to qa or qb
    if np.fabs(sin_half_theta) < 0.001:
        qm.w = qa.w * 0.5 + qb.w * 0.5
        qm.x = qa.x * 0.5 + qb.x * 0.5
        qm.y = qa.y * 0.5 + qb.y * 0.5
        qm.z = qa.z * 0.5 + qb.z * 0.5
        return qm

    ratio_a = np.sin((1 - t) * half_theta) / sin_half_theta
    ratio_b = np.sin(t * half_theta) / sin_half_theta

    qm.w = qa.w * ratio_a + qb.w * ratio_b
    qm.x = qa.x * ratio_a + qb.x * ratio_b
    qm.y = qa.y * ratio_a + qb.y * ratio_b
    qm.z = qa.z * ratio_a + qb.z * ratio_b

    return qm


def lerp(va, vb, t):
    """given 2 3-D vectors, output interpolation at t, assuming va is at t=0 and vb is at t=1
    Note: all vectors will be (3,1) numpy arrays"""
    return va * (1 - t) + vb * t

def generate_smooth_path(first_frame, last_frame, trans_velocity, rot_velocity, time_btwn_poses):
    """ Given two Transform msgs from ros, generate a smooth trajectory
    (as a Path nav_msg) between them. Both transform msgs should be in the same relative frame.

    The trans_velocity and rot_velocity are both considered maximums, so only one will be the true constraint.
    trans_velocity is m/s, rot_velocity is rad/s.

    Note that time_btwn_poses is currently hardcoded in thing_control, and should
    be the value from there, or you won't get the desired velocity."""

    va = np.array([first_frame.translation.x,
                   first_frame.translation.y,
                   first_frame.translation.z]).reshape(3, 1)
    vb = np.array([last_frame.translation.x,
                   last_frame.translation.y,
                   last_frame.translation.z]).reshape(3, 1)
    qa = Quaternion(first_frame.rotation.x,
                    first_frame.rotation.y,
                    first_frame.rotation.z,
                    first_frame.rotation.w)
    qb = Quaternion(last_frame.rotation.x,
                    last_frame.rotation.y,
                    last_frame.rotation.z,
                    last_frame.rotation.w)

    trans_dist = np.linalg.norm(va - vb)

    # rot distance, see http://www.boris-belousov.net/2016/12/01/quat-dist/ and
    # Metrics for 3D Rotations: Comparison and Analysis by Huynh
    rot_dist = 2 * np.arccos(np.clip(
        np.abs(qa.w * qb.w + qa.x * qb.x + qa.y * qb.y + qa.z * qb.z), -1.0, 1.0
    ))

    distance_btwn_poses = trans_velocity * time_btwn_poses
    distance_btwn_poses_rot = rot_velocity * time_btwn_poses
    min_num_pts_trans = int(np.round(trans_dist / distance_btwn_poses))
    min_num_pts_rot = int(np.round(rot_dist / distance_btwn_poses_rot))
    ret_path = Path()

    num_pts = max(min_num_pts_rot, min_num_pts_trans)
    for i in range(num_pts):
        # lerp and slerp for next point
        t = float(i) / num_pts
        trans_interp = lerp(va, vb, t)
        rot_interp = quat_slerp(qa, qb, t)

        # append results to path
        p = PoseStamped()
        p.pose.position.x = trans_interp[0]
        p.pose.position.y = trans_interp[1]
        p.pose.position.z = trans_interp[2]
        p.pose.orientation.w = rot_interp.w
        p.pose.orientation.x = rot_interp.x
        p.pose.orientation.y = rot_interp.y
        p.pose.orientation.z = rot_interp.z
        ret_path.poses.append(p)

    final_pose = PoseStamped()
    final_pose.pose = tf_msg_to_pose_msg(last_frame)
    ret_path.poses.append(final_pose)
    return ret_path