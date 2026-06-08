# AprilTags-Detector-GPS-Data-Capturing
This is the repository for the AprilTags Detector and GPS Data Capturing system. 

# Setup
1) Clone the repository into your workspace:
   
```bash
git clone https://github.com/NiccoBonucci/AprilTags-Detector-GPS-Data-Capturing.git
cd AprilTags-Detector-GPS-Data-Capturing
```

2) From the root of the workspace (e.g., "/*path_from_home_to_workspace*/catkin_ws") build using:

```bash
cd /*path_from_home_to_workspace*/catkin_ws
catkin build
```

3) Source both the ROS main environment and then the newly built workspace
```bash
source /opt/ros/noetic/setup.bash
source devel/setup.bash
```

Now your workspace is ready for the detection and data gathering system to be executed.

# SINGLE CAMERA
# AprilTags Detector and Pose estimator 
To be able to make the system work, you will need the Intel realsense2 package installed in your computer. You can both install it from source or via GitHub package.
After verifying that the camera works, open the following terminal and execute the commands in the following order:

1) Activate the Realsense camera with depth channel:
```bash
roslaunch realsense2_camera rs_camera.launch align_depth:=true
```
2) Visualize if the camera is active and if the depth topics are being displayed correctly:
```bash
rqt_image_view
```
3) Launch the ApriTags Cube pose estimator for the detection of multiple tags at the same time:
```bash
roslaunch cube_pose_estimator multi_cube_pose.launch 
```
You should see an output similar to this:
SUMMARY
...

NODES
  /
    apriltag_ros (apriltag_ros/apriltag_ros_continuous_node)
    cube_pose_node (cube_pose_estimator/multi_cube_pose_node.py)
...

[INFO] [1777909852.612552039]: Loaded tag config: 10, size: 0.125, frame_name: tag10 <br />
[INFO] [1777909852.612577095]: Loaded tag config: 11, size: 0.125, frame_name: tag11 <br />
[INFO] [1777909852.612582517]: Loaded tag config: 12, size: 0.125, frame_name: tag12 <br />
[INFO] [1777909852.612596753]: Loaded tag config: 13, size: 0.125, frame_name: tag13 <br />
[INFO] [1777909852.612601593]: Loaded tag config: 14, size: 0.125, frame_name: tag14 <br />
[WARN] [1777909852.612779608]: No tag bundles specified <br />
[WARN] [1777909852.612962302]: remove_duplicates parameter not provided. Defaulting to true <br />
[INFO] [1777909852.844341]: cube_pose_node started. <br />
[INFO] [1777909852.847135]: Listening on /tag_detections and publishing /cube_pose/tagXX for all visible tags <br />

Don't worry if only one tag is shown here, it's only the first tag that it is being read.

4) Check if all the topics are being published:
```bash
rostopic list
```
If the estimator is working correctly, you should see the following topics:

/cube_pose/fused
/cube_pose/azimuth_deg

These two topics contain, respectively, the mean pose of the AprilTags cube, computed from all the visible faces, and the corresponding heading disalignment with respect to the tag aligned with the front (or rear) of the car.

# Data Capturing 
To use the Data Capturing system, launch the following command:

*SNAPSHOT BY PRESSING BUTTON*
```bash
roslaunch cube_pose_estimator snapshot_capture.launch
```

*AUTO-CAPTURING SNAPSHOT*

```bash
roslaunch cube_pose_estimator snapshot_capture.launch _auto_capture_rate:=0.5 
```
or modify the ROS parameter for the capturing rate in the snapshot_capture.launch file:
<param name="auto_capture_rate" value="1.0"/>  <!-- 1 Hz -->

and then launch:
roslaunch cube_pose_estimator snapshot_capture.launch

You should see the following log:

SUMMARY
======== <br />

PARAMETERS <br />
 * /rosdistro: noetic <br />
 * /rosversion: 1.17.4 <br />
 * /snapshot_capture_node/output_root: /home/simulator/D... <br />

NODES <br />
  /
    snapshot_capture_node (cube_pose_estimator/snapshot_capture_node.py) <br />

ROS_MASTER_URI=http://localhost:11311 <br />

process[snapshot_capture_node-1]: started with pid [222148] <br />
[INFO] [1777909940.104008]: snapshot_capture_node started. <br />

[INFO] [1777909940.104976]: Press 'c' to capture snapshot, 'q' to quit. <br />
or
[INFO] [1777909940.104976]: Auto-capture enabled at %.2f Hz. Press 'q' to quit. <br />

Every time you press the "c" button on the keyboard, the script subscribes to the following topics (IF they are being published) and saves the data at that exact moment:

  "/camera/color/image_raw": Image, <br />
  "/camera/color/camera_info": CameraInfo, <br />
  "/camera/depth/image_rect_raw": Image, <br />
  "/camera/depth/camera_info": CameraInfo, <br />
  "/tag_detections": AprilTagDetectionArray, <br />
  "/cube_pose/tag10": PoseStamped, <br />
  "/cube_pose/tag11": PoseStamped, <br />
  "/cube_pose/tag12": PoseStamped, <br />
  "/cube_pose/tag13": PoseStamped, <br />
  "/cube_pose/tag14": PoseStamped, <br />
  "/cube_pose/current": CubePoseArray, <br />
  "/cube_pose/fused": PoseStamped, <br />
  "/cube_pose/azimuth_deg": Float64, <br />
  "/fix": NavSatFix, <br />

It also saves the RGB and depth image in .png format.

# DOUBLE CAMERA
# AprilTags Detector and Pose estimator 
To be able to make the system work, you will need the Intel realsense2 package installed in your computer. You can both install it from source or via GitHub package.
After verifying that the camera works, open the following terminal and execute the commands in the following order:

1) Activate the Realsense camera with depth channel:
```bash
roslaunch cube_pose_estimator cameras.launch 
```
2) Visualize if the camera is active and if the depth topics are being displayed correctly:
```bash
rqt
```
When the rqt window opens, you have to open the two image visualizers, one for the topic "camera_up/color/image_raw/" and the other for "camera_down/color/image_raw/"

3) Launch the ApriTags Cube pose estimator for the detection of multiple tags at the same time:
```bash
roslaunch cube_pose_estimator multi_cube_pose_cameras.launch 
```
You should see an output similar to the one for the single camera.

4) Check if all the topics are being published:
```bash
rostopic list
```
If the estimator is working correctly, you should see the following topics:

/cube_pose/fused
/cube_pose/azimuth_deg

These two topics contain, respectively, the mean pose of the AprilTags cube, computed from all the visible faces, and the corresponding heading disalignment with respect to the tag aligned with the front (or rear) of the car.

# Data Capturing 
To use the Data Capturing system, launch the following command:

*SNAPSHOT BY PRESSING BUTTON*
```bash
roslaunch cube_pose_estimator snapshot_capture_cameras.launch
```

*AUTO-CAPTURING SNAPSHOT*

```bash
roslaunch cube_pose_estimator snapshot_capture_cameras.launch _auto_capture_rate:=0.5 
```
or modify the ROS parameter for the capturing rate in the snapshot_capture_cameras.launch file:
<param name="auto_capture_rate" value="1.0"/>  <!-- 1 Hz -->

and then launch:
roslaunch cube_pose_estimator snapshot_capture.launch


SUMMARY
======== <br />

PARAMETERS <br />
 * /rosdistro: noetic <br />
 * /rosversion: 1.17.4 <br />
 * /snapshot_capture_node/output_root: /home/simulator/D... <br />

NODES <br />
  /
    snapshot_capture_node (cube_pose_estimator/snapshot_capture_node.py) <br />

ROS_MASTER_URI=http://localhost:11311 <br />

process[snapshot_capture_node-1]: started with pid [222148] <br />
[INFO] [1777909940.104008]: snapshot_capture_node started. <br />
[INFO] [1777909940.104976]: Press 'c' to capture snapshot, 'q' to quit. <br />
or
[INFO] [1777909940.104976]: Auto-capture enabled at %.2f Hz. Press 'q' to quit. <br />

Every three times you press the ">" button on the pointer (or press the "c" button on the keyboard), the script subscribes to the following topics (IF they are being published) and saves the data at that exact moment:

  "/camera_up/color/image_raw": Image, <br />
  "/camera_up/color/camera_info": CameraInfo, <br />
  "/camera_up/depth/image_rect_raw": Image, <br />
  "/camera_up/depth/camera_info": CameraInfo, <br />
  "/tag_detections": AprilTagDetectionArray, <br />
  "/cube_pose/tag10": PoseStamped, <br />
  "/cube_pose/tag11": PoseStamped, <br />
  "/cube_pose/tag12": PoseStamped, <br />
  "/cube_pose/tag13": PoseStamped, <br />
  "/cube_pose/tag14": PoseStamped, <br />
  "/cube_pose/current": CubePoseArray, <br />
  "/cube_pose/fused": PoseStamped, <br />
  "/cube_pose/azimuth_deg": Float64, <br />
  "/fix": NavSatFix, <br />

It also saves the RGB and depth image in .png format.
