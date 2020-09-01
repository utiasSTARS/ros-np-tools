""" Some utility functions getting ros TF msgs as output. """

# built-ins
import time

# ros
import tf2_ros
import rospy
import tf.transformations as tf_trans
from geometry_msgs.msg import (Pose, Transform,
                               TransformStamped, PoseStamped,
                               WrenchStamped, Quaternion)

# others
import numpy as np

# internal
from ros_np_tools.pose_msg import mat_to_pose_msg
from ros_np_tools.tf_mat import tf_msg_to_mat
from ros_np_tools.pos_quat_np import tf_msg_to_pos_quat

class TransformWithUpdate(TransformStamped):
    def __init__(self, tf_buffer, target_frame=None, source_frame=None, tf_stamped_msg=None):
        super(TransformWithUpdate, self).__init__()
        self.tf_buffer = tf_buffer
        if target_frame is not None and source_frame is not None:
            self.header.frame_id = target_frame
            self.child_frame_id = source_frame
            self.update_mandatory()
        elif tf_stamped_msg is not None:
            self.set_tf_attributes(tf_stamped_msg.header,
                                   tf_stamped_msg.child_frame_id, tf_stamped_msg.transform)

    def set_tf_attributes(self, h, cfi, t):
        self.header = h
        self.child_frame_id = cfi
        self.transform = t

    def update(self):
        try:
            tf_stamped = self.tf_buffer.lookup_transform(
                self.header.frame_id, self.child_frame_id, rospy.Time(0))
            self.set_tf_attributes(tf_stamped.header, tf_stamped.child_frame_id, tf_stamped.transform)
        except tf2_ros.TransformException:
            print("careful, tf lookup failed for target: %s, source: %s, leaving it as is."
                  % (self.header.frame_id, self.child_frame_id))

    def update_mandatory(self):
        tf_stamped = None
        while tf_stamped is None:
            try:
                tf_stamped = self.tf_buffer.lookup_transform(
                    self.header.frame_id, self.child_frame_id, rospy.Time(0))
                self.set_tf_attributes(tf_stamped.header, tf_stamped.child_frame_id, tf_stamped.transform)
            except tf2_ros.TransformException:
                print("tf not yet ready for frames %s and %s, trying again in .2 seconds..."
                      % (self.header.frame_id, self.child_frame_id))
                time.sleep(.2)

    def as_mat(self):
        return tf_msg_to_mat(self.transform)

    def as_pos_quat(self, quat_order='xyzw'):
        return tf_msg_to_pos_quat(self.transform, quat_order)

    def as_pos_eul(self, eul_axes='sxyz'):
        pos_quat = self.as_pos_quat()
        eul = tf_trans.euler_from_quaternion(pos_quat[3:], eul_axes)
        return np.concatenate([pos_quat[:3], eul])

def pose_msg_to_tf_msg(p_msg):
    if hasattr(p_msg, 'header'):  # check to see if stamped or not
        tf_msg = TransformStamped()
        tf_msg.header = p_msg.header
        tf_msg.transform.translation.x = p_msg.pose.position.x
        tf_msg.transform.translation.y = p_msg.pose.position.y
        tf_msg.transform.translation.z = p_msg.pose.position.z
        tf_msg.transform.rotation.x = p_msg.pose.orientation.x
        tf_msg.transform.rotation.y = p_msg.pose.orientation.y
        tf_msg.transform.rotation.z = p_msg.pose.orientation.z
        tf_msg.transform.rotation.w = p_msg.pose.orientation.w
    else:
        tf_msg = Transform()
        tf_msg.translation.x = p_msg.position.x
        tf_msg.translation.y = p_msg.position.y
        tf_msg.translation.z = p_msg.position.z
        tf_msg.rotation.x = p_msg.orientation.x
        tf_msg.rotation.y = p_msg.orientation.y
        tf_msg.rotation.z = p_msg.orientation.z
        tf_msg.rotation.w = p_msg.orientation.w
    return tf_msg

def mat_to_tf_msg(mat):
    return pose_msg_to_tf_msg(mat_to_pose_msg(mat))

def pos_quat_to_tf_msg(pos_quat, quat_order='xyzw'):
    tf_msg = Transform()
    tf_msg.translation.x = pos_quat[0]
    tf_msg.translation.y = pos_quat[1]
    tf_msg.translation.z = pos_quat[2]
    setattr(tf_msg.rotation, quat_order[0], pos_quat[3])
    setattr(tf_msg.rotation, quat_order[1], pos_quat[4])
    setattr(tf_msg.rotation, quat_order[2], pos_quat[5])
    setattr(tf_msg.rotation, quat_order[3], pos_quat[6])
    return tf_msg

def pos_eul_to_tf_msg(pos_eul, eul_axes='sxyz'):
    tf_msg = Transform()
    tf_msg.translation.x = pos_eul[0]
    tf_msg.translation.y = pos_eul[1]
    tf_msg.translation.z = pos_eul[2]
    quat = tf_trans.quaternion_from_euler(*pos_eul[3:], axes=eul_axes)
    tf_msg.rotation.x = quat[0]
    tf_msg.rotation.y = quat[1]
    tf_msg.rotation.z = quat[2]
    tf_msg.rotation.w = quat[3]
    return tf_msg