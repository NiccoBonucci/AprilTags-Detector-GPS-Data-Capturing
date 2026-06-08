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
from pynput import mouse
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CameraInfo, NavSatFix
from apriltag_ros.msg import AprilTagDetectionArray
from geometry_msgs.msg import PoseStamped, PoseArray
from std_msgs.msg import Int32MultiArray
import threading


class SnapshotCaptureNode:
    def __init__(self):
        rospy.init_node("snapshot_capture_node")

        self.snapshot_counter = 1

        self.bridge = CvBridge()
        self.latest_msgs = {}

        self.output_root = rospy.get_param(
            "~output_root",
            os.path.expanduser("~/apriltag_snapshots")
        )

        self.auto_capture_rate = rospy.get_param("~auto_capture_rate", 0.0)  # Hz, 0 = disabled
        self.last_auto_capture_time = rospy.Time(0)

        self.topics = {
            "/camera/color/image_raw": Image,
            "/camera/color/camera_info": CameraInfo,
            "/camera/depth/image_rect_raw": Image,
            "/camera/depth/camera_info": CameraInfo,
            "/tag_detections": AprilTagDetectionArray,
            "/cube_pose/all": PoseArray,
            "/cube_pose/all_ids": Int32MultiArray,
            "/cube_pose/tag10": PoseStamped,
            "/cube_pose/tag11": PoseStamped,
            "/cube_pose/tag12": PoseStamped,
            "/cube_pose/tag13": PoseStamped,
            "/cube_pose/tag14": PoseStamped,
            "/fix": NavSatFix,
        }

        self.subs = []
        for topic, msg_type in self.topics.items():
            self.subs.append(
                rospy.Subscriber(topic, msg_type, self.generic_callback, callback_args=topic, queue_size=1)
            )

        self.mouse_listener = mouse.Listener(
            on_click=self.on_click
        )
        self.mouse_listener.start()

        os.makedirs(self.output_root, exist_ok=True)

        rospy.loginfo("snapshot_capture_node started.")

        if self.auto_capture_rate > 0:
            rospy.loginfo("Auto-capture enabled at %.2f Hz. Press 'q' to quit.", self.auto_capture_rate)
        else:
            rospy.loginfo("Press 'c' to capture snapshot, 'q' to quit.")


    def generic_callback(self, msg, topic_name):
        self.latest_msgs[topic_name] = msg

    def msg_to_yaml_file(self, msg, filepath):
        with open(filepath, "w") as f:
            f.write(str(msg))

    def write_metadata_yaml(self, snapshot_dir, timestamp):
        metadata = {
            "snapshot_timestamp": timestamp,
            "ros_time_now": rospy.Time.now().to_sec(),
            "available_topics": sorted(list(self.latest_msgs.keys())),
            "visible_tag_ids": [],
            "num_visible_tags": 0,
            "num_cube_poses": 0,
            "color_image": {},
            "depth_image": {},
            "camera_info": {},
            "gps": {},
        }

        # Visible tag IDs from your custom topic, if present
        if "/cube_pose/all_ids" in self.latest_msgs:
            ids_msg = self.latest_msgs["/cube_pose/all_ids"]
            metadata["visible_tag_ids"] = list(ids_msg.data)
        elif "/tag_detections" in self.latest_msgs:
            det_msg = self.latest_msgs["/tag_detections"]
            ids = []
            for det in det_msg.detections:
                if len(det.id) > 0:
                    ids.append(int(det.id[0]))
            metadata["visible_tag_ids"] = ids

        metadata["num_visible_tags"] = len(metadata["visible_tag_ids"])

        # Number of cube poses from PoseArray, if present
        if "/cube_pose/all" in self.latest_msgs:
            pose_array_msg = self.latest_msgs["/cube_pose/all"]
            metadata["num_cube_poses"] = len(pose_array_msg.poses)

        # Color image info
        if "/camera/color/image_raw" in self.latest_msgs:
            img_msg = self.latest_msgs["/camera/color/image_raw"]
            metadata["color_image"] = {
                "width": img_msg.width,
                "height": img_msg.height,
                "encoding": img_msg.encoding,
                "frame_id": img_msg.header.frame_id,
                "stamp": img_msg.header.stamp.to_sec(),
            }

        # Depth image info
        if "/camera/depth/image_rect_raw" in self.latest_msgs:
            depth_msg = self.latest_msgs["/camera/depth/image_rect_raw"]
            metadata["depth_image"] = {
                "width": depth_msg.width,
                "height": depth_msg.height,
                "encoding": depth_msg.encoding,
                "frame_id": depth_msg.header.frame_id,
                "stamp": depth_msg.header.stamp.to_sec(),
            }

        # Camera info
        if "/camera/color/camera_info" in self.latest_msgs:
            cam_info = self.latest_msgs["/camera/color/camera_info"]
            metadata["camera_info"]["color"] = {
                "frame_id": cam_info.header.frame_id,
                "stamp": cam_info.header.stamp.to_sec(),
                "width": cam_info.width,
                "height": cam_info.height,
                "K": list(cam_info.K),
                "D": list(cam_info.D),
                "distortion_model": cam_info.distortion_model,
            }

        if "/camera/depth/camera_info" in self.latest_msgs:
            cam_info = self.latest_msgs["/camera/depth/camera_info"]
            metadata["camera_info"]["depth"] = {
                "frame_id": cam_info.header.frame_id,
                "stamp": cam_info.header.stamp.to_sec(),
                "width": cam_info.width,
                "height": cam_info.height,
                "K": list(cam_info.K),
                "D": list(cam_info.D),
                "distortion_model": cam_info.distortion_model,
            }
        
        if "/fix" in self.latest_msgs:
            gps_msg = self.latest_msgs["/fix"]
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
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        snapshot_dir = os.path.join(self.output_root, f"snapshot_{timestamp}")
        os.makedirs(snapshot_dir, exist_ok=True)

        bag_path = os.path.join(snapshot_dir, f"snapshot_{timestamp}.bag")

        # Save bag with latest messages
        with rosbag.Bag(bag_path, "w") as bag:
            for topic, msg in self.latest_msgs.items():
                if hasattr(msg, "header"):
                    bag.write(topic, msg, msg.header.stamp)
                else:
                    bag.write(topic, msg, rospy.Time.now())

        # Save readable files
        for topic, msg in self.latest_msgs.items():
            safe_name = topic.strip("/").replace("/", "__")

            if topic == "/camera/color/image_raw":
                try:
                    self.save_color_image(msg, os.path.join(snapshot_dir, f"{safe_name}.png"))
                except Exception as e:
                    rospy.logwarn("Failed saving color image: %s", str(e))
                    self.msg_to_yaml_file(msg, os.path.join(snapshot_dir, f"{safe_name}.txt"))

            elif topic == "/camera/depth/image_rect_raw":
                try:
                    self.save_depth_image(
                        msg,
                        os.path.join(snapshot_dir, f"{safe_name}.png"),
                        os.path.join(snapshot_dir, f"{safe_name}.npy"),
                    )
                except Exception as e:
                    rospy.logwarn("Failed saving depth image: %s", str(e))
                    self.msg_to_yaml_file(msg, os.path.join(snapshot_dir, f"{safe_name}.txt"))

            else:
                self.msg_to_yaml_file(msg, os.path.join(snapshot_dir, f"{safe_name}.txt"))

        self.write_metadata_yaml(snapshot_dir, timestamp)

        rospy.loginfo("%d: Snapshot saved in: %s", self.snapshot_counter, snapshot_dir)
        self.snapshot_counter+=1

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

    def on_click(self, x, y, button, pressed):
        if pressed:
            threading.Thread(
                target=self.capture_snapshot,
                daemon=True
            ).start()

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
