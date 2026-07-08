# Portfolio Card — LiDAR-Camera Calibration

**Title:** LiDAR-Camera Calibration Workflow for ROS 2

**Hook:** ROS 2 perception infrastructure — documented LiDAR-camera extrinsic calibration for Livox Mid-360 + RealSense D435 using rosbag2, SAM, and CalibAnything.

**Problem:** Robot perception pipelines require an accurate LiDAR-to-camera extrinsic before semantic mapping or fused sensing can run — but the toolchain spans rosbag export, mask annotation, C++ build issues, and optimizer edge cases.

**What I built:**
- End-to-end LiDAR-camera extrinsic calibration workflow (rosbag2 → ros2_unbag → SAM → CalibAnything)
- Debugging notes and fixes: ros2_unbag sync, SAM batch rerun, jsoncpp linking, point_range assertion
- Validation protocol and semantic mapping static TF integration

**Tools:** ROS 2 Jazzy, rosbag2 MCAP, ros2_unbag, Segment Anything (vit_l), CalibAnything, OpenCV, Docker, RViz

**Result/demo:** ~224 synchronized image/PCD pairs processed; calibration completed with qualitative validation. No accuracy metrics or demo media in repo.

**Links:**
- Repository: `LiDAR---Camera-Calibration---Annotations-and-Fixes-` (recommended rename: `lidar-camera-calibration-ros2`)
- Related: semantic mapping / CalibAnything stack
