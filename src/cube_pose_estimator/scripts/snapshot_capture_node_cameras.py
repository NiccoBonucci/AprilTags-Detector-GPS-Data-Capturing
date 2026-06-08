#!/usr/bin/env python3

import os
import sys
import cv2
import yaml
import rospy
import rosbag
import select
import termios
import tty
from datetime import datetime

from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CameraInfo, NavSatFix
from apriltag_ros.msg import AprilTagDetectionArray

from geometry_msgs.msg import PoseStamped
from cube_pose_estimator.msg import CubePoseArray
from std_msgs.msg import Float64



class SnapshotCaptureNode:
    def __init__(self):
        rospy.init_node("snapshot_capture_node")

        self.bridge = CvBridge()
        self.latest_msgs = {}
        self.capture_requested = False
        self.capture_in_progress = False
        self.capture_t_ref = None

        self.output_root = rospy.get_param(
            "~output_root",
            os.path.expanduser("~/apriltag_snapshots")
        )

        self.auto_capture_rate = rospy.get_param("~auto_capture_rate", 0.0)  # Hz, 0 = disabled
        self.last_auto_capture_time = rospy.Time(0)

        self.topics = {
            "/cam_up/color/image_raw": Image,
            "/cam_up/color/camera_info": CameraInfo,
            "/cam_up/depth/image_rect_raw": Image,
            "/cam_up/depth/camera_info": CameraInfo,
            "/tag_detections": AprilTagDetectionArray,
            "/cube_pose/tag10": PoseStamped,
            "/cube_pose/tag11": PoseStamped,
            "/cube_pose/tag12": PoseStamped,
            "/cube_pose/tag13": PoseStamped,
            "/cube_pose/tag14": PoseStamped,
            "/cube_pose/current": CubePoseArray,
            "/cube_pose/fused": PoseStamped,
            "/cube_pose/azimuth_deg": Float64,
            "/fix": NavSatFix,
        }

        self.subs = []
        for topic, msg_type in self.topics.items():
            self.subs.append(
                rospy.Subscriber(topic, msg_type, self.generic_callback, callback_args=topic, queue_size=1)
            )

        os.makedirs(self.output_root, exist_ok=True)

        rospy.loginfo("snapshot_capture_node started.")

        if self.auto_capture_rate > 0:
            rospy.loginfo("Auto-capture enabled at %.2f Hz. Press 'q' to quit.", self.auto_capture_rate)
        else:
            rospy.loginfo("Press 'c' to capture snapshot, 'q' to quit.")

    def generic_callback(self, msg, topic_name):
        self.latest_msgs[topic_name] = msg

        # If user requested a capture, anchor it on the NEXT /cube_pose/all frame
        if topic_name == "/cube_pose/current" and self.capture_requested and not self.capture_in_progress:
            self.capture_in_progress = True
            self.capture_t_ref = msg.header.stamp
            self.capture_snapshot()
            self.capture_requested = False
            self.capture_in_progress = False

    def is_close_in_time(self, msg, t_ref, tol=0.05):
        if not hasattr(msg, "header"):
            return False
        dt = abs((msg.header.stamp - t_ref).to_sec())
        return dt <= tol

    def msg_to_yaml_file(self, msg, filepath):
        with open(filepath, "w") as f:
            f.write(str(msg))

    def write_combined_cube_pose_file(self, snapshot_dir, cube_pose_msg):
        combined = {
            "header": {
                "seq": cube_pose_msg.header.seq,
                "stamp": {
                    "secs": cube_pose_msg.header.stamp.secs,
                    "nsecs": cube_pose_msg.header.stamp.nsecs,
                },
                "frame_id": cube_pose_msg.header.frame_id,
            },
            "num_ids": len(cube_pose_msg.ids),
            "num_poses": len(cube_pose_msg.poses),
            "ids": list(cube_pose_msg.ids),
            "poses": [],
            "id_pose_pairs": [],
        }

        for pose in cube_pose_msg.poses:
            pose_dict = {
                "position": {
                    "x": pose.position.x,
                    "y": pose.position.y,
                    "z": pose.position.z,
                },
                "orientation": {
                    "x": pose.orientation.x,
                    "y": pose.orientation.y,
                    "z": pose.orientation.z,
                    "w": pose.orientation.w,
                },
            }
            combined["poses"].append(pose_dict)

        n = min(len(cube_pose_msg.ids), len(cube_pose_msg.poses))
        for i in range(n):
            pose = cube_pose_msg.poses[i]
            combined["id_pose_pairs"].append({
                "id": int(cube_pose_msg.ids[i]),
                "pose": {
                    "position": {
                        "x": pose.position.x,
                        "y": pose.position.y,
                        "z": pose.position.z,
                    },
                    "orientation": {
                        "x": pose.orientation.x,
                        "y": pose.orientation.y,
                        "z": pose.orientation.z,
                        "w": pose.orientation.w,
                    },
                }
            })

        filepath = os.path.join(snapshot_dir, "cube_pose__current.yaml")
        with open(filepath, "w") as f:
            yaml.safe_dump(combined, f, sort_keys=False)
            
    def write_metadata_yaml(self, snapshot_dir, timestamp, msgs_to_save):
        metadata = {
            "snapshot_timestamp": timestamp,
            "ros_time_now": rospy.Time.now().to_sec(),
            "available_topics": sorted(list(msgs_to_save.keys())),
            "visible_tag_ids": [],
            "num_visible_tags": 0,
            "num_cube_poses": 0,
            "color_image": {},
            "depth_image": {},
            "camera_info": {},
            "gps": {},
            "fused_pose": {},
            "azimuth_deg": None,
        }

        # Visible tag IDs from the synchronized cube pose message, if present
        if "/cube_pose/current" in msgs_to_save:
            cube_pose_msg = msgs_to_save["/cube_pose/current"]
            metadata["visible_tag_ids"] = list(cube_pose_msg.ids)

        elif "/tag_detections" in msgs_to_save:
            det_msg = msgs_to_save["/tag_detections"]
            ids = []
            for det in det_msg.detections:
                if len(det.id) > 0:
                    ids.append(int(det.id[0]))
            metadata["visible_tag_ids"] = ids

        metadata["num_visible_tags"] = len(metadata["visible_tag_ids"])

        if "/cube_pose/current" in msgs_to_save:
            cube_pose_msg = msgs_to_save["/cube_pose/current"]
            metadata["num_cube_poses"] = len(cube_pose_msg.poses)

        if "/cube_pose/fused" in msgs_to_save:
            fused_msg = msgs_to_save["/cube_pose/fused"]
            metadata["fused_pose"] = {
                "frame_id": fused_msg.header.frame_id,
                "stamp": fused_msg.header.stamp.to_sec(),
                "position": {
                    "x": fused_msg.pose.position.x,
                    "y": fused_msg.pose.position.y,
                    "z": fused_msg.pose.position.z,
                },
                "orientation": {
                    "x": fused_msg.pose.orientation.x,
                    "y": fused_msg.pose.orientation.y,
                    "z": fused_msg.pose.orientation.z,
                    "w": fused_msg.pose.orientation.w,
                },
            }

        if "/cube_pose/azimuth_deg" in msgs_to_save:
            metadata["azimuth_deg"] = msgs_to_save["/cube_pose/azimuth_deg"].data


        # Color image info
        if "/cam_up/color/image_raw" in msgs_to_save:
            img_msg = msgs_to_save["/cam_up/color/image_raw"]
            metadata["color_image"] = {
                "width": img_msg.width,
                "height": img_msg.height,
                "encoding": img_msg.encoding,
                "frame_id": img_msg.header.frame_id,
                "stamp": img_msg.header.stamp.to_sec(),
            }

        # Depth image info
        if "/cam_up/depth/image_rect_raw" in msgs_to_save:
            depth_msg = msgs_to_save["/cam_up/depth/image_rect_raw"]
            metadata["depth_image"] = {
                "width": depth_msg.width,
                "height": depth_msg.height,
                "encoding": depth_msg.encoding,
                "frame_id": depth_msg.header.frame_id,
                "stamp": depth_msg.header.stamp.to_sec(),
            }

        # Camera info
        if "/cam_up/color/camera_info" in msgs_to_save:
            cam_info = msgs_to_save["/cam_up/color/camera_info"]
            metadata["camera_info"]["color"] = {
                "frame_id": cam_info.header.frame_id,
                "stamp": cam_info.header.stamp.to_sec(),
                "width": cam_info.width,
                "height": cam_info.height,
                "K": list(cam_info.K),
                "D": list(cam_info.D),
                "distortion_model": cam_info.distortion_model,
            }

        if "/cam_up/depth/camera_info" in msgs_to_save:
            cam_info = msgs_to_save["/cam_up/depth/camera_info"]
            metadata["camera_info"]["depth"] = {
                "frame_id": cam_info.header.frame_id,
                "stamp": cam_info.header.stamp.to_sec(),
                "width": cam_info.width,
                "height": cam_info.height,
                "K": list(cam_info.K),
                "D": list(cam_info.D),
                "distortion_model": cam_info.distortion_model,
            }
        
        if "/fix" in msgs_to_save:
            gps_msg = msgs_to_save["/fix"]
            metadata["gps"] = {
                "frame_id": gps_msg.header.frame_id,
                "stamp": gps_msg.header.stamp.to_sec(),
                "status": gps_msg.status.status,
                "service": gps_msg.status.service,
                "latitude": gps_msg.latitude,
                "longitude": gps_msg.longitude,
                "altitude": gps_msg.altitude,
                "position_covariance": list(gps_msg.position_covariance),
                "position_covariance_type": gps_msg.position_covariance_type,
            }

        metadata_path = os.path.join(snapshot_dir, "metadata.yaml")
        with open(metadata_path, "w") as f:
            yaml.safe_dump(metadata, f, sort_keys=False)

    def save_color_image(self, msg, filepath):
        cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        cv2.imwrite(filepath, cv_img)

    def save_depth_image(self, msg, filepath_png, filepath_npy):
        cv_depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        cv2.imwrite(filepath_png, cv_depth)
        try:
            import numpy as np
            np.save(filepath_npy, cv_depth)
        except Exception as e:
            rospy.logwarn("Could not save depth .npy: %s", str(e))

    def capture_snapshot(self):
        if self.capture_t_ref is None:
            rospy.logwarn("No reference timestamp available for snapshot.")
            return

        t_ref = self.capture_t_ref
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        snapshot_dir = os.path.join(self.output_root, f"snapshot_{timestamp}")
        os.makedirs(snapshot_dir, exist_ok=True)

        bag_path = os.path.join(snapshot_dir, f"snapshot_{timestamp}.bag")

        # Build a synchronized subset of topics to save
        msgs_to_save = {}

        # 1) Always anchor on /cube_pose/current
        if "/cube_pose/current" in self.latest_msgs:
            cube_pose_current_msg = self.latest_msgs["/cube_pose/current"]
            if self.is_close_in_time(cube_pose_current_msg, t_ref, tol=0.001):
                msgs_to_save["/cube_pose/current"] = cube_pose_current_msg
                visible_ids = list(cube_pose_current_msg.ids)
            else:
                rospy.logwarn("Current /cube_pose/current is not aligned with t_ref, aborting snapshot.")
                return
        else:
            rospy.logwarn("No /cube_pose/current available, aborting snapshot.")
            return
        
        # 2) Save fused pose if aligned
        if "/cube_pose/fused" in self.latest_msgs:
            fused_msg = self.latest_msgs["/cube_pose/fused"]
            if self.is_close_in_time(fused_msg, t_ref, tol=0.03):
                msgs_to_save["/cube_pose/fused"] = fused_msg
            else:
                rospy.logwarn("Skipping /cube_pose/fused (timestamp too old/new).")

        # 2b) Save azimuth if available
        if "/cube_pose/azimuth_deg" in self.latest_msgs:
            msgs_to_save["/cube_pose/azimuth_deg"] = self.latest_msgs["/cube_pose/azimuth_deg"]

        # 3) Save only the single-tag topics that are CURRENTLY visible and time-aligned
        for tag_id in visible_ids:
            tag_topic = f"/cube_pose/tag{tag_id}"
            if tag_topic in self.latest_msgs:
                tag_msg = self.latest_msgs[tag_topic]
                if self.is_close_in_time(tag_msg, t_ref, tol=0.03):
                    msgs_to_save[tag_topic] = tag_msg
                else:
                    rospy.logwarn("Skipping stale %s (timestamp too old/new).", tag_topic)

        # 4) Save current tag detections if aligned
        if "/tag_detections" in self.latest_msgs:
            det_msg = self.latest_msgs["/tag_detections"]
            if self.is_close_in_time(det_msg, t_ref, tol=0.03):
                msgs_to_save["/tag_detections"] = det_msg

        # 5) Save camera topics closest to this frame
        for cam_topic in [
            "/cam_up/color/image_raw",
            "/cam_up/color/camera_info",
            "/cam_up/depth/image_rect_raw",
            "/cam_up/depth/camera_info",
            "/fix",
        ]:
            if cam_topic in self.latest_msgs:
                msg = self.latest_msgs[cam_topic]

                # More permissive tolerance for sensors
                if hasattr(msg, "header"):
                    tol = 0.10 if "image" in cam_topic or "camera_info" in cam_topic else 0.20
                    if self.is_close_in_time(msg, t_ref, tol=tol):
                        msgs_to_save[cam_topic] = msg
                    else:
                        rospy.logwarn("Skipping %s (not close enough to snapshot time).", cam_topic)
                else:
                    msgs_to_save[cam_topic] = msg

        # Save bag with synchronized messages only
        with rosbag.Bag(bag_path, "w") as bag:
            for topic, msg in msgs_to_save.items():
                if hasattr(msg, "header"):
                    bag.write(topic, msg, msg.header.stamp)
                else:
                    bag.write(topic, msg, rospy.Time.now())

        for topic, msg in msgs_to_save.items():
            safe_name = topic.strip("/").replace("/", "__")

            if topic == "/cam_up/color/image_raw":
                try:
                    self.save_color_image(msg, os.path.join(snapshot_dir, f"{safe_name}.png"))
                except Exception as e:
                    rospy.logwarn("Failed saving color image: %s", str(e))
                    self.msg_to_yaml_file(msg, os.path.join(snapshot_dir, f"{safe_name}.txt"))

            elif topic == "/cam_up/depth/image_rect_raw":
                try:
                    self.save_depth_image(
                        msg,
                        os.path.join(snapshot_dir, f"{safe_name}.png"),
                        os.path.join(snapshot_dir, f"{safe_name}.npy"),
                    )
                except Exception as e:
                    rospy.logwarn("Failed saving depth image: %s", str(e))
                    self.msg_to_yaml_file(msg, os.path.join(snapshot_dir, f"{safe_name}.txt"))

            elif topic == "/cube_pose/current":
                pass
            else:
                self.msg_to_yaml_file(msg, os.path.join(snapshot_dir, f"{safe_name}.txt"))

        if "/cube_pose/current" in msgs_to_save:
            self.write_combined_cube_pose_file(
                snapshot_dir,
                msgs_to_save["/cube_pose/current"]
            )


        self.write_metadata_yaml(snapshot_dir, timestamp,msgs_to_save)

        rospy.loginfo("Synchronized snapshot saved in: %s", snapshot_dir)

    def get_key(self):
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(fd)
            rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
            if rlist:
                return sys.stdin.read(1)
            return None
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)

    def run(self):
        rate = rospy.Rate(200)
        while not rospy.is_shutdown():
            key = self.get_key()

            if key is not None:
                if key.lower() == "c":
                    self.capture_requested = True
                    rospy.loginfo("Capture requested: waiting for next /cube_pose/current frame...")
                elif key.lower() == "q":
                    rospy.loginfo("Exiting snapshot_capture_node.")
                    break

            # Auto-capture logic
            if self.auto_capture_rate > 0 and not self.capture_requested and not self.capture_in_progress:
                interval = rospy.Duration(1.0 / self.auto_capture_rate)
                if (rospy.Time.now() - self.last_auto_capture_time) >= interval:
                    self.capture_requested = True
                    self.last_auto_capture_time = rospy.Time.now()
                    rospy.loginfo("Auto-capture triggered.")

            rate.sleep()


if __name__ == "__main__":
    try:
        node = SnapshotCaptureNode()
        node.run()
    except rospy.ROSInterruptException:
        pass