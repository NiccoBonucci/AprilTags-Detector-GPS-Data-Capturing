#!/usr/bin/env python3


import math
import numpy as np
import rospy

from geometry_msgs.msg import PoseStamped, TransformStamped
from std_msgs.msg import Float64
from apriltag_ros.msg import AprilTagDetectionArray
import tf2_ros
from tf.transformations import quaternion_matrix, quaternion_from_matrix



R_ROBOT_CAM = np.array([
    [ 0.0,  0.0,  1.0],
    [-1.0,  0.0,  0.0],
    [ 0.0, -1.0,  0.0],
])


def pose_to_matrix(pose_msg):
    """geometry_msgs/Pose → 4x4 homogeneous matrix."""
    q = pose_msg.orientation
    T = quaternion_matrix([q.x, q.y, q.z, q.w])
    T[0, 3] = pose_msg.position.x
    T[1, 3] = pose_msg.position.y
    T[2, 3] = pose_msg.position.z
    return T


def matrix_to_pose_stamped(T, header):
    """4x4 matrix → PoseStamped."""
    ps = PoseStamped()
    ps.header = header
    ps.pose.position.x = T[0, 3]
    ps.pose.position.y = T[1, 3]
    ps.pose.position.z = T[2, 3]
    q = quaternion_from_matrix(T)
    ps.pose.orientation.x = q[0]
    ps.pose.orientation.y = q[1]
    ps.pose.orientation.z = q[2]
    ps.pose.orientation.w = q[3]
    return ps


def cam_to_robot(T_cam):
    """
    Convert any 4x4 transform expressed in camera optical frame
    to robot frame.  Works for both poses and directions.
    """
    T_robot = np.eye(4)
    T_robot[:3, :3] = R_ROBOT_CAM @ T_cam[:3, :3]
    T_robot[:3, 3]  = R_ROBOT_CAM @ T_cam[:3, 3]
    return T_robot


# ---------------------------------------------------------------------------
# Cube geometry
# ---------------------------------------------------------------------------
CUBE_SIDE = 0.150          # metres
H = CUBE_SIDE / 2.0        # half-side


R_tag_at_px = np.array([
    [  0.0,   0.0,    1.0 ],   # cube_x component  ← sign flipped
    [  1.0,   0.0,    0.0 ],   # cube_y component
    [  0.0,  -1.0,    0.0 ],   # cube_z component
])

# tag10: +x face.  No additional rotation needed.
# tag11: +y face = tag10 rotated +90° around cube_z
# tag12: -x face = tag10 rotated 180° around cube_z
# tag13: -y face = tag10 rotated -90° around cube_z


def rot_z(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c,-s,0],[s,c,0],[0,0,1]])


def make_T(R, t):
    T = np.eye(4)
    T[:3,:3] = R
    T[:3, 3] = t
    return T


def invert_T(T):
    R, t = T[:3,:3], T[:3,3]
    Ti = np.eye(4)
    Ti[:3,:3] = R.T
    Ti[:3, 3] = -R.T @ t
    return Ti


# Build T_cube_tag  (cube ← tag)
# The translation is the tag face centre in cube coords.
_T_cube_tag = {
    10: make_T(rot_z(0.0)             @ R_tag_at_px,  np.array([ H,  0,  0])),
    11: make_T(rot_z( math.pi/2)      @ R_tag_at_px,  np.array([ 0,  H,  0])),
    12: make_T(rot_z( math.pi)        @ R_tag_at_px,  np.array([-H,  0,  0])),
    13: make_T(rot_z(-math.pi/2)      @ R_tag_at_px,  np.array([ 0, -H,  0])),
    14: make_T(np.array([[1,0,0],[0,-1,0],[0,0,-1]], dtype=float), np.array([0, 0, H])),
}
# Precompute tag ← cube  (needed to compute cam position in cube frame)
_T_tag_cube = {k: invert_T(v) for k, v in _T_cube_tag.items()}


class CubePoseNode:
    def __init__(self):
        rospy.init_node("cube_pose_node")

        # --- publishers ---
        self.pub_tag_robot  = {
            tid: rospy.Publisher(f"/cube_pose/tag{tid}",
                                 PoseStamped, queue_size=5)
            for tid in _T_cube_tag
        }
        self.pub_cube_robot = rospy.Publisher("/cube_pose/fused_pose",
                                              PoseStamped, queue_size=5)
        self.pub_azimuth    = rospy.Publisher("/cube_pose/azimuth_deg",
                                              Float64, queue_size=5)

        self.tf_br = tf2_ros.TransformBroadcaster()

        # azimuth state
        self.first      = True
        self.az_start   = 0.0
        self.az_last    = 0.0

        rospy.Subscriber("/tag_detections", AprilTagDetectionArray,
                         self._cb, queue_size=1)
        rospy.loginfo("cube_pose_node ready.")

    # ------------------------------------------------------------------
    def _broadcast_tf(self, T, header, child):
        tf = TransformStamped()
        tf.header = header
        tf.child_frame_id = child
        tf.transform.translation.x = T[0,3]
        tf.transform.translation.y = T[1,3]
        tf.transform.translation.z = T[2,3]
        q = quaternion_from_matrix(T)
        tf.transform.rotation.x = q[0]
        tf.transform.rotation.y = q[1]
        tf.transform.rotation.z = q[2]
        tf.transform.rotation.w = q[3]
        self.tf_br.sendTransform(tf)

    # ------------------------------------------------------------------
    def _cb(self, msg):
        if not msg.detections:
            return

        cam_positions_in_cube = []   # for azimuth
        cube_positions_in_robot = [] # for position fusion
        weights = []

        for det in msg.detections:
            if not det.id:
                continue
            tid = det.id[0]
            if tid not in _T_cube_tag:
                continue

            # ---- T_cam_tag: tag pose in camera optical frame ----
            T_cam_tag = pose_to_matrix(det.pose.pose.pose)

            # ---- T_robot_tag: tag pose in robot frame -----------
            T_robot_tag = cam_to_robot(T_cam_tag)

            # Publish tag pose in robot frame (Phase 1 verification)
            self.pub_tag_robot[tid].publish(
                matrix_to_pose_stamped(T_robot_tag, msg.header))
            self._broadcast_tf(T_robot_tag, msg.header, f"tag{tid}_robot")

            # ---- T_robot_cube from this tag ---------------------
            # T_robot_cube = T_robot_tag  @  T_tag_cube
            T_robot_cube = T_robot_tag @ _T_tag_cube[tid]
            cube_positions_in_robot.append(T_robot_cube[:3, 3])

            # ---- camera position in cube frame (for azimuth) ----
            # camera origin in tag frame = -(R_cam_tag.T @ t_cam_tag)
            R_ct = T_cam_tag[:3,:3]
            t_ct = T_cam_tag[:3, 3]
            cam_in_tag = -R_ct.T @ t_ct

            # rotate + translate into cube frame
            T_ctag = _T_cube_tag[tid]
            cam_in_cube = T_ctag[:3,:3] @ cam_in_tag + T_ctag[:3, 3]

            if tid != 14:   # top face is degenerate for yaw
                cam_positions_in_cube.append(cam_in_cube)
                dist = np.linalg.norm(t_ct)
                weights.append(1.0 / (dist + 0.01))

            # Diagnostic log (throttled) — remove once verified
            rospy.loginfo_throttle(1.0,
                "tag%d | robot pos=[%.3f, %.3f, %.3f] | "
                "tag_z_in_robot=[%.3f, %.3f, %.3f] | "
                "cam_in_cube=[%.3f, %.3f, %.3f]",
                tid,
                T_robot_tag[0,3], T_robot_tag[1,3], T_robot_tag[2,3],
                T_robot_tag[0,2], T_robot_tag[1,2], T_robot_tag[2,2],
                cam_in_cube[0],   cam_in_cube[1],   cam_in_cube[2],
            )

        # ---- fused cube position in robot frame -----------------
        if not cube_positions_in_robot:
            return

        fused_pos = np.mean(cube_positions_in_robot, axis=0)

        # Publish a minimal cube pose (position only for now; orientation TBD)
        T_cube_pub = np.eye(4)
        T_cube_pub[:3, 3] = fused_pos
        self.pub_cube_robot.publish(
            matrix_to_pose_stamped(T_cube_pub, msg.header))

        # ---- azimuth from cam position in cube frame ------------
        if not cam_positions_in_cube:
            return

        w = np.array(weights)
        w /= w.sum()
        cam_cube = np.average(cam_positions_in_cube, axis=0, weights=w)

        # azimuth = angle in cube XY plane from cube +x (tag10 normal)
        # 0° = camera in front of tag10
        # +90° = camera on tag11 side
        azimuth_rad = math.atan2(cam_cube[1], cam_cube[0])

        if self.first:
            self.az_last = azimuth_rad
            self.first   = False

        delta = azimuth_rad - self.az_last
        delta = (delta + math.pi) % (2 * math.pi) - math.pi
        self.az_last += delta

        azimuth_deg = math.degrees(self.az_last)  

        """
        if self.first:
            self.az_start = azimuth_rad
            self.az_last  = azimuth_rad
            self.first    = False

        delta = azimuth_rad - self.az_last
        delta = (delta + math.pi) % (2*math.pi) - math.pi
        self.az_last += delta

        azimuth_deg = math.degrees(self.az_last - self.az_start)
        """


        self.pub_azimuth.publish(Float64(data=azimuth_deg))

        rospy.loginfo_throttle(0.02,
            "cube in robot: [%.3f, %.3f, %.3f]  azimuth=%.2f deg  "
            "(cam_in_cube=[%.3f, %.3f, %.3f])",
            fused_pos[0], fused_pos[1], fused_pos[2],
            azimuth_deg,
            cam_cube[0], cam_cube[1], cam_cube[2],
        )


if __name__ == "__main__":
    try:
        CubePoseNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
