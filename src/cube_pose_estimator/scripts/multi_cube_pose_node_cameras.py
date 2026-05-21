#!/usr/bin/env python3

import math
import numpy as np
import rospy

from geometry_msgs.msg import PoseStamped, PoseArray, TransformStamped
from std_msgs.msg import Int32MultiArray

from apriltag_ros.msg import AprilTagDetectionArray


import tf2_ros
from tf.transformations import quaternion_matrix, quaternion_from_matrix


def make_transform(R, t):
    """Build 4x4 homogeneous transform from rotation matrix and translation."""
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = t
    return T


def invert_transform(T):
    """Invert 4x4 homogeneous transform."""
    R = T[:3, :3]
    t = T[:3, 3]
    T_inv = np.eye(4)
    T_inv[:3, :3] = R.T
    T_inv[:3, 3] = -R.T @ t
    return T_inv


def pose_to_matrix(pose_msg):
    """Convert geometry_msgs/Pose to 4x4 matrix."""
    q = pose_msg.orientation
    t = pose_msg.position

    T = quaternion_matrix([q.x, q.y, q.z, q.w])
    T[0, 3] = t.x
    T[1, 3] = t.y
    T[2, 3] = t.z
    return T


def matrix_to_pose_stamped(T, header, child_frame_id=None):
    """Convert 4x4 transform to PoseStamped."""
    pose_msg = PoseStamped()
    pose_msg.header = header

    pose_msg.pose.position.x = T[0, 3]
    pose_msg.pose.position.y = T[1, 3]
    pose_msg.pose.position.z = T[2, 3]

    q = quaternion_from_matrix(T)
    pose_msg.pose.orientation.x = q[0]
    pose_msg.pose.orientation.y = q[1]
    pose_msg.pose.orientation.z = q[2]
    pose_msg.pose.orientation.w = q[3]

    return pose_msg


def rot_x(theta):
    c, s = math.cos(theta), math.sin(theta)
    return np.array([
        [1, 0, 0],
        [0, c, -s],
        [0, s, c]
    ])


def rot_y(theta):
    c, s = math.cos(theta), math.sin(theta)
    return np.array([
        [c, 0, s],
        [0, 1, 0],
        [-s, 0, c]
    ])


def rot_z(theta):
    c, s = math.cos(theta), math.sin(theta)
    return np.array([
        [c, -s, 0],
        [s,  c, 0],
        [0,  0, 1]
    ])


class CubePoseEstimator:
    def __init__(self):
        rospy.init_node("cube_pose_node")

        self.pose_pubs = {
            10: rospy.Publisher("/cube_pose/tag10", PoseStamped, queue_size=10),
            11: rospy.Publisher("/cube_pose/tag11", PoseStamped, queue_size=10),
            12: rospy.Publisher("/cube_pose/tag12", PoseStamped, queue_size=10),
            13: rospy.Publisher("/cube_pose/tag13", PoseStamped, queue_size=10),
            14: rospy.Publisher("/cube_pose/tag14", PoseStamped, queue_size=10),
        }
        
        self.pose_array_pub = rospy.Publisher("/cube_pose/all", PoseArray, queue_size=10)
        self.pose_ids_pub = rospy.Publisher("/cube_pose/all_ids", Int32MultiArray, queue_size=10)
        
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()

        # Cube geometry
        self.L = 0.150
        self.half_L = self.L / 2.0

        self.R_user_cam = np.array([
            [0.0,  0.0,  1.0],
            [-1.0, 0.0,  0.0],
            [0.0, -1.0,  0.0]
        ])

        # Static transforms: cube -> tag_i
        # Cube frame:
        #   x forward
        #   y left
        #   z up
        #
        # Tag ordering:
        #   tag10 = first face
        #   tag11 = +90 deg around z from tag10
        #   tag12 = +90 deg around z from tag11
        #   tag13 = +90 deg around z from tag12
        #   tag14 = upper face

        self.T_cube_tag = {}

        # IMPORTANT:
        # We start with the nominal face rotations.
        # If the sign convention of the detected tag frame differs from your expectation,
        # we will correct it after the first bench test.
        self.T_cube_tag[10] = make_transform(rot_z(0.0),             np.array([ self.half_L, 0.0,          0.0]))
        self.T_cube_tag[11] = make_transform(rot_z(math.pi / 2.0),   np.array([ 0.0,         self.half_L,  0.0]))
        self.T_cube_tag[12] = make_transform(rot_z(math.pi),         np.array([-self.half_L, 0.0,          0.0]))
        self.T_cube_tag[13] = make_transform(rot_z(3.0 * math.pi/2), np.array([ 0.0,        -self.half_L,  0.0]))
        self.T_cube_tag[14] = make_transform(rot_y(-math.pi / 2.0),  np.array([ 0.0,         0.0,          self.half_L]))

        # Precompute inverses: tag_i -> cube
        self.T_tag_cube = {tag_id: invert_transform(T) for tag_id, T in self.T_cube_tag.items()}

        self.sub = rospy.Subscriber("/tag_detections", AprilTagDetectionArray, self.detections_callback, queue_size=1)

        rospy.loginfo("cube_pose_node started.")
        rospy.loginfo("Listening on /tag_detections and publishing /cube_pose/tagXX for all visible tags")


    def detections_callback(self, msg):
        if not msg.detections:
            return

        pose_array_msg = PoseArray()
        pose_array_msg.header = msg.header
        ids_msg = Int32MultiArray()

        R_user_cam = np.array([
            [0.0,  0.0,  1.0],
            [-1.0, 0.0,  0.0],
            [0.0, -1.0,  0.0]
        ])

        valid_count = 0

        for det in msg.detections:
            if len(det.id) == 0:
                continue

            tag_id = det.id[0]

            if tag_id not in self.T_tag_cube:
                continue

            # apriltag_ros pose is nested: detection.pose.pose.pose
            tag_pose = det.pose.pose.pose

            # Tag pose in camera frame
            T_cam_tag = pose_to_matrix(tag_pose)

            # Cube pose in camera frame
            T_cam_cube = T_cam_tag @ self.T_tag_cube[tag_id]

            # Convert to user frame: x forward, y left, z up
            R_cam_cube = T_cam_cube[:3, :3]
            t_cam_cube = T_cam_cube[:3, 3]

            R_user_cube = self.R_user_cam @ R_cam_cube
            t_user_cube = self.R_user_cam @ t_cam_cube

            T_user_cube = np.eye(4)
            T_user_cube[:3, :3] = R_user_cube
            T_user_cube[:3, 3] = t_user_cube

            # Publish per-tag pose
            cube_pose_msg = matrix_to_pose_stamped(T_user_cube, msg.header)
            self.pose_pubs[tag_id].publish(cube_pose_msg)
            
            pose_array_msg.poses.append(cube_pose_msg.pose)
            ids_msg.data.append(tag_id)

            # Publish TF per tag
            tf_msg = TransformStamped()
            tf_msg.header = msg.header
            tf_msg.child_frame_id = f"cube_from_tag{tag_id}"

            tf_msg.transform.translation.x = T_user_cube[0, 3]
            tf_msg.transform.translation.y = T_user_cube[1, 3]
            tf_msg.transform.translation.z = T_user_cube[2, 3]

            q = quaternion_from_matrix(T_user_cube)
            tf_msg.transform.rotation.x = q[0]
            tf_msg.transform.rotation.y = q[1]
            tf_msg.transform.rotation.z = q[2]
            tf_msg.transform.rotation.w = q[3]

            self.tf_broadcaster.sendTransform(tf_msg)

            rospy.loginfo_throttle(
                0.1,
                "Tag %d -> cube USER position = [%.3f, %.3f, %.3f] m",
                tag_id,
                T_user_cube[0, 3],
                T_user_cube[1, 3],
                T_user_cube[2, 3]
            )

            """
            
            # PUBLISH CUBE POSE IN THE CAMERA OPTICAL FRAME 

            # Publish PoseStamped
            cube_pose_msg = matrix_to_pose_stamped(T_cam_cube, msg.header)
            self.pose_pub.publish(cube_pose_msg)

            # Publish TF
            tf_msg = TransformStamped()
            tf_msg.header = msg.header
            tf_msg.child_frame_id = "cube"

            tf_msg.transform.translation.x = T_cam_cube[0, 3]
            tf_msg.transform.translation.y = T_cam_cube[1, 3]
            tf_msg.transform.translation.z = T_cam_cube[2, 3]

            q = quaternion_from_matrix(T_cam_cube)
            tf_msg.transform.rotation.x = q[0]
            tf_msg.transform.rotation.y = q[1]
            tf_msg.transform.rotation.z = q[2]
            tf_msg.transform.rotation.w = q[3]

            self.tf_broadcaster.sendTransform(tf_msg)
            

            rospy.loginfo_throttle(
                1.0,
                "Using tag %d -> cube position = [%.3f, %.3f, %.3f] m",
                chosen_id,
                T_cam_cube[0, 3],
                T_cam_cube[1, 3],
                T_cam_cube[2, 3]
            )
            """
            
            valid_count += 1
        
        if len(pose_array_msg.poses) > 0:
            self.pose_array_pub.publish(pose_array_msg)
            self.pose_ids_pub.publish(ids_msg)


if __name__ == "__main__":
    try:
        CubePoseEstimator()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass