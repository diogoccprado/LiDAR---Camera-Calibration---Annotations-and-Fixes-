# Portfolio Card — LiDAR-Camera Calibration

**Title:** LiDAR-Camera Calibration Workflow for ROS 2

**Hook:** Reusable ROS 2 workflow for calibrating a RealSense D435i RGB camera and Velodyne LiDAR using synchronized bags, SAM2, and CalibAnything.

**Problem:** Robot perception pipelines require an accurate LiDAR-to-camera extrinsic before semantic mapping or fused sensing can run — but the toolchain spans rosbag export, mask annotation, C++ build issues, and optimizer edge cases.

**What I built:**
- End-to-end workflow: rosbag2 → synchronized PNG/PCD export → SAM2 → CalibAnything
- Reusable export, metadata, montage, and FAST-LIO2 transform-composition scripts
- Build/configuration fixes and a camera-optical-to-IMU frame conversion guide
- Projection, physical-mounting, RViz, and FAST-LIO2 validation checklists

**Tools:** ROS 2 Jazzy, rosbag2, SAM2, CalibAnything, OpenCV, PCL, RViz, FAST-LIO2

**Result/demo:** 401 synchronized image/PCD pairs and 21,155 SAM2 masks, with initial/refined projection comparisons included in the repository. Final IMU-frame deployment remains subject to on-robot validation.

**Links:**
- Repository: `lidar-camera-calibration-ros2`
- Case study: [`docs/portfolio_calibration_case_study.md`](docs/portfolio_calibration_case_study.md)
