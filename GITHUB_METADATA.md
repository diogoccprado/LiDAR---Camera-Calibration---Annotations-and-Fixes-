# GitHub Metadata — Recommended Settings

Apply these manually in the GitHub web UI or via `gh` CLI. **Do not run automatically.**

## Recommended repository name

```
lidar-camera-calibration-ros2
```

## Recommended About description

```
ROS 2 workflow for LiDAR-camera extrinsic calibration using Livox Mid-360, RealSense D435, rosbag2, SAM masks, and CalibAnything.
```

## Recommended topics

```
ros2
lidar
camera-calibration
extrinsic-calibration
livox
realsense
calibanything
segment-anything
computer-vision
robotics
```

## Suggested pinned repo sentence

> ROS 2 LiDAR-camera extrinsic calibration workflow: Livox Mid-360 + RealSense D435 via rosbag2, SAM masks, and CalibAnything—with documented fixes.

## Manual rename instructions

### Option A: GitHub web UI

1. Go to **Settings → General → Repository name**
2. Change to `lidar-camera-calibration-ros2`
3. Click **Rename**

### Option B: GitHub CLI

```bash
gh repo rename lidar-camera-calibration-ros2 --repo diogoccprado/LiDAR---Camera-Calibration---Annotations-and-Fixes-
```

### Update local remote after rename

```bash
cd ~/LiDAR---Camera-Calibration---Annotations-and-Fixes-
git remote set-url origin git@github.com:diogoccprado/lidar-camera-calibration-ros2.git
```

## Set About description and topics (CLI)

```bash
gh repo edit diogoccprado/lidar-camera-calibration-ros2 \
  --description "ROS 2 workflow for LiDAR-camera extrinsic calibration using Livox Mid-360, RealSense D435, rosbag2, SAM masks, and CalibAnything." \
  --add-topic ros2 --add-topic lidar --add-topic camera-calibration \
  --add-topic extrinsic-calibration --add-topic livox --add-topic realsense \
  --add-topic calibanything --add-topic segment-anything \
  --add-topic computer-vision --add-topic robotics
```

> Replace `diogoccprado` with your GitHub username if different.
