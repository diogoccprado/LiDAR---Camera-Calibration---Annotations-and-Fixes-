# LiDAR–Camera Extrinsic Calibration with ROS 2

A reusable workflow for calibrating a 3D LiDAR to an RGB camera from a ROS 2
bag, using synchronized PNG/PCD pairs, SAM or SAM2 masks, and
[CalibAnything](https://github.com/OpenCalib/CalibAnything). It also explains
how to convert the camera-optical result into the IMU-frame convention used by
FAST-LIO2.

The guide grew from calibrating the Intel RealSense D435i and Velodyne mounted
on the Espeleo robot with ROS 2 Jazzy. Commands use configurable paths and
topics so the workflow can be reused with other sensors.

![Initial and refined projection](docs/assets/calibration/calibration_before_after.png)

## What this workflow produces

CalibAnything estimates a rigid transform that maps LiDAR points into the
camera color optical frame:

```text
p_camera_color_optical = T_camera_color_optical_lidar · p_lidar
```

The run also produces projection images for qualitative inspection. If a
LiDAR-inertial odometry package expects the LiDAR pose in an IMU frame, the
result must be composed with the camera-to-IMU static transform; it must not be
copied directly.

No universal numerical accuracy can be claimed from projection screenshots
alone. Validate the transform against the physical sensor mounting and in the
downstream system.

## Workflow overview

```text
ROS 2 bag
  → inspect topics and metadata
  → export synchronized images and point clouds
  → verify pairs and intrinsics
  → generate SAM/SAM2 masks
  → post-process masks
  → configure and run CalibAnything
  → inspect projections and refine
  → convert camera-optical result to the deployment frame
  → validate in RViz and FAST-LIO2
```

## Requirements

### Recorded inputs

- `sensor_msgs/msg/Image`
- Matching `sensor_msgs/msg/CameraInfo`
- `sensor_msgs/msg/PointCloud2`
- Camera matrix `K` and distortion coefficients `D`
- A point-cloud field named `intensity`
- A physically plausible initial LiDAR-to-camera extrinsic

The image and point cloud should observe a static, geometrically varied scene.
Avoid a dataset containing only one flat wall. Motion between unsynchronized
captures creates calibration error, so record while the robot and scene are
stationary where possible.

If `/tf` and `/tf_static` are absent from the bag, that does not prevent
calibration, but the initial extrinsic and any later frame conversion must be
obtained from URDF, live static TFs, CAD, measurement, or another trusted
configuration.

### Software

- ROS 2 with `rosbag2_py`, `cv_bridge`, and `sensor_msgs_py`
- Python 3, NumPy, OpenCV, and Pillow
- SAM or SAM2 with a compatible checkpoint
- CMake, OpenCV 4, PCL, Eigen, Boost, and jsoncpp
- CalibAnything
- RViz for deployment validation

On ROS 2 Jazzy, source the installation before running the bag scripts:

```bash
source /opt/ros/jazzy/setup.bash
```

## Configure paths and topics

Keep bags and generated datasets outside this repository:

```bash
export BAG_DIR="/path/to/rosbag2_directory"
export DATASET_DIR="/path/to/calibration_dataset"
export CALIBANYTHING_DIR="/path/to/CalibAnything"
export SAM2_DIR="/path/to/sam2"
export SAM2_CHECKPOINT="/path/to/sam2_checkpoint.pt"

export IMAGE_TOPIC="/camera/camera/color/image_raw"
export CAMERA_INFO_TOPIC="/camera/camera/color/camera_info"
export POINTCLOUD_TOPIC="/velodyne_points"
```

The documented Espeleo run contained 2,423 images, 2,423 camera-info messages,
and 803 clouds. Synchronization produced 401 PNG/PCD pairs. Those values are
examples, not required counts.

The expected working layout is:

```text
dataset/
  images/
    000000.png
    ...
  pc/
    000000.pcd
    ...
  masks/
    000000/
      000.png
      ...
  processed_masks/
    ...
  calib.json
```

## Step 1 — Inspect the bag

Pass the rosbag2 directory containing `metadata.yaml`:

```bash
ros2 bag info "$BAG_DIR"
```

Check exact topic names, message types, message counts, duration, and storage
format. Do not infer topic names from a launch file or another robot.

Dump the first camera and cloud metadata records:

```bash
python3 scripts/dump_calib_metadata.py "$BAG_DIR" \
  --camera-info-topic "$CAMERA_INFO_TOPIC" \
  --pointcloud-topic "$POINTCLOUD_TOPIC" \
  --output /tmp/calib_sensor_metadata.json
```

Confirm:

- `camera_info.K` is populated and corresponds to the exported image size.
- The distortion model and `D` coefficients are known.
- The image frame is the intended color optical frame.
- Point-cloud fields include `x`, `y`, `z`, and `intensity`.
- The point-cloud frame ID is the expected LiDAR frame.

## Step 2 — Export synchronized image/PCD pairs

The local exporter is the robust default. It avoids a failure observed in
`ros2_unbag`, where OpenCV rejected a `pathlib.PosixPath` passed to
`cv2.imwrite`.

```bash
python3 scripts/export_calib_pairs.py "$BAG_DIR" "$DATASET_DIR" \
  --image-topic "$IMAGE_TOPIC" \
  --camera-info-topic "$CAMERA_INFO_TOPIC" \
  --pointcloud-topic "$POINTCLOUD_TOPIC" \
  --sync-tolerance 0.05
```

The script:

- reads the bag with `rosbag2_py`;
- approximately synchronizes image and cloud header stamps;
- writes zero-padded PNG/PCD pairs;
- preserves `x`, `y`, `z`, and `intensity` in binary PCD files;
- writes `calib_metadata.json` with intrinsics and export statistics.

Tune `--sync-tolerance` for the sensors' timestamp behavior. Use `--every 5`
or `--every 10` to reduce a large dataset during iteration. Run
`python3 scripts/export_calib_pairs.py --help` for all options.

`ros2_unbag` remains an alternative. If its code passes a `Path` to OpenCV,
patch the write call:

```python
cv2.imwrite(str(output_path), image)
```

## Step 3 — Verify the export

Check that stems match one-to-one:

```bash
comm -3 \
  <(for f in "$DATASET_DIR"/images/*.png; do basename "${f%.png}"; done | sort) \
  <(for f in "$DATASET_DIR"/pc/*.pcd; do basename "${f%.pcd}"; done | sort)
```

No output means the names match. Then inspect counts and sample files:

```bash
printf "images: "; find "$DATASET_DIR/images" -maxdepth 1 -name '*.png' | wc -l
printf "clouds: "; find "$DATASET_DIR/pc" -maxdepth 1 -name '*.pcd' | wc -l
file "$DATASET_DIR/images/000000.png" "$DATASET_DIR/pc/000000.pcd"
```

Open several images and point clouds, not only the first pair. Verify that:

- images decode at the dimensions reported by `CameraInfo`;
- PCD headers list `FIELDS x y z intensity`;
- point counts are nonzero and coordinates are finite;
- each pair depicts the same static scene;
- filenames have no extension duplicated in `file_name`.

## Step 4 — Generate SAM or SAM2 masks

CalibAnything uses segmentation to compare scene structure across modalities.
This repository includes a parameterized SAM2 helper modeled on the successful
run:

```bash
python3 -m venv .venv-sam2
source .venv-sam2/bin/activate
python -m pip install --upgrade pip

cd "$SAM2_DIR"
python -m pip install -e .
python -m pip install opencv-python
cd -

python3 scripts/generate_sam2_masks.py \
  "$DATASET_DIR/images" "$DATASET_DIR/masks" \
  --checkpoint "$SAM2_CHECKPOINT" \
  --model-config configs/sam2.1/sam2.1_hiera_s.yaml
```

The helper creates one folder per image stem and skips completed folders, which
makes interrupted or out-of-memory runs resumable. The Espeleo dataset produced
21,155 raw masks for 401 images with a SAM2.1 Hiera Small checkpoint.

Classic SAM is also valid. Its automatic mask generator must produce the same
one-folder-per-image structure. Thresholds are dataset-dependent; visually
inspect masks for over-segmentation, missing objects, and nearly full-frame
masks before continuing.

## Step 5 — Process masks

Run CalibAnything's mask post-processor:

```bash
rm -rf "$DATASET_DIR/processed_masks"
python3 "$CALIBANYTHING_DIR/processed_mask.py" \
  -i "$DATASET_DIR/masks" \
  -o "$DATASET_DIR/processed_masks"
```

Confirm every selected stem in `calib.json` has an image, PCD, and processed
mask. Keep raw and processed masks separate.

## Step 6 — Clone and build CalibAnything

```bash
git clone https://github.com/OpenCalib/CalibAnything.git "$CALIBANYTHING_DIR"
cd "$CALIBANYTHING_DIR"
mkdir -p build
cmake -S . -B build
cmake --build build -j"$(nproc)"
```

Install distribution packages for PCL, OpenCV, Eigen, Boost, and jsoncpp if
CMake reports missing dependencies. Package names differ by distribution.

## Step 7 — Patch common build issues

### OpenCV 4: `CV_HSV2BGR` not declared

Find the use:

```bash
rg 'CV_HSV2BGR' "$CALIBANYTHING_DIR"
```

Replace the legacy macro:

```cpp
CV_HSV2BGR
```

with:

```cpp
cv::COLOR_HSV2BGR
```

Then rebuild. The modern enum is declared by OpenCV 4.

### jsoncpp: static/dynamic library mismatch

If CMake hardcodes a missing or incompatible `libjsoncpp.a`, link the shared
library installed by the system. On common x86-64 Debian/Ubuntu systems:

```cmake
/usr/lib/x86_64-linux-gnu/libjsoncpp.so
```

Alternatively use the imported CMake target exposed by the installed jsoncpp
package. Confirm the actual location first:

```bash
ldconfig -p | rg jsoncpp
```

Reconfigure after editing:

```bash
rm -rf "$CALIBANYTHING_DIR/build"
cmake -S "$CALIBANYTHING_DIR" -B "$CALIBANYTHING_DIR/build"
cmake --build "$CALIBANYTHING_DIR/build" -j"$(nproc)"
```

## Step 8 — Create `calib.json`

Copy the sanitized working example into the dataset:

```bash
cp examples/calibanything/calib_velodyne_realsense_example.json \
  "$DATASET_DIR/calib.json"
```

Edit these fields:

- `cam_K.data`: 3×3 matrix from `CameraInfo.K`.
- `cam_dist.data`: coefficients from `CameraInfo.D`.
- `T_lidar_to_cam.data`: initial
  `T_camera_color_optical_lidar`.
- `img_folder`, `pc_folder`, `mask_folder`: folders relative to the JSON file.
- `img_format` and `pc_format`: include the leading dot.
- `file_name`: stems only, such as `"000010"`, not `000010.png`.
- `params`: keep `point_range`, `search_range`, `down_sample`, and `thread`
  nested here for the tested CalibAnything revision.

JSON does not support comments. The example is deliberately comment-free;
field explanations live in this README.

Start with a sparse list, for example every tenth pair, to iterate quickly.
Increase temporal coverage after the pipeline works. Search ranges should be
large enough to contain the expected correction but not so large that the
optimizer can settle on physically impossible alternatives.

## Step 9 — Choose and validate the initial transform

Use the convention `T_target_source`: the matrix maps a point expressed in
`source` into `target`. CalibAnything's `T_lidar_to_cam` therefore means:

```text
T_camera_color_optical_lidar
```

It is not a camera pose in the LiDAR frame.

Good initial sources include a measured sensor mounting, CAD/URDF, a live
static TF tree, or a previously calibrated transform. Check all of the
following before optimizing:

1. Translation is in metres.
2. Rotation is a proper orthonormal matrix with determinant near `+1`.
3. The transform direction is LiDAR → camera, not its inverse.
4. The target is `camera_color_optical_frame`, not a camera link or IMU frame.
5. The initial projection is recognizable.

For a standard optical frame:

- `+x` points right in the image;
- `+y` points down;
- `+z` points forward.

A common robotics sensor frame uses `+x` forward, `+y` left, `+z` up. That axis
difference must be represented in the rotation. In the documented run, a raw
FAST-LIO2 D435i IMU extrinsic projected incorrectly because its target was the
IMU frame. Applying the camera/IMU-to-color-optical axis conversion produced a
usable initial projection.

## Step 10 — Run CalibAnything

Resolve dataset paths relative to `calib.json` by running from the dataset
unless your CalibAnything revision specifies another base:

```bash
cd "$DATASET_DIR"
"$CALIBANYTHING_DIR/bin/run_lidar2camera" "$DATASET_DIR/calib.json"
```

Some revisions place the binary under `build/bin`. Locate it with:

```bash
find "$CALIBANYTHING_DIR" -type f -name run_lidar2camera -perm -111
```

Move each run's outputs into a separate directory before rerunning:

```bash
RUN_DIR="$DATASET_DIR/run_$(date +%Y%m%d_%H%M%S)"
mkdir -p "$RUN_DIR"
mv init_proj.png refined_proj.png init_proj_seg.png refined_proj_seg.png \
  extrinsic.txt calib_log.txt "$RUN_DIR"/
```

Adjust that list if your revision uses different output names.

## Step 11 — Inspect the result

Review:

- `init_proj.png`
- `refined_proj.png`
- `init_proj_seg.png`
- `refined_proj_seg.png`
- `extrinsic.txt`
- the calibration log

Look for LiDAR returns following wall, floor, door, furniture, and object
boundaries across the image—not only at its center. Segmented projections
should improve at meaningful edges rather than merely moving more points into
the frame.

The documented first successful run estimated:

```text
[[ 0.0798394, -0.9962340, -0.0338321, -0.131370],
 [-0.1632750,  0.0204120, -0.9863690,  0.282945],
 [ 0.9833450,  0.0842750, -0.1610310, -0.302923],
 [ 0.0000000,  0.0000000,  0.0000000,  1.000000]]
```

These numbers are evidence from one physical mounting, not defaults for
another robot.

## Step 12 — Refine from the first result

Copy the first run's 4×4 result into `T_lidar_to_cam.data`, reduce the search
range, and use denser temporal sampling. One follow-up used every fifth pair
and a `5°` / `0.12 m` search range. It estimated:

```text
[[ 0.1795430, -0.9821120, -0.0567609, -0.253064],
 [-0.2013620,  0.0197877, -0.9793160,  0.309608],
 [ 0.9629220,  0.1872590, -0.1942080, -0.467019],
 [ 0.0000000,  0.0000000,  0.0000000,  1.000000]]
```

Do not assume the second numerical answer is better merely because it came
from a refinement. Its translation must still agree with the sensor lever arm.
Compare projections, optimizer stability, physical dimensions, and downstream
behavior.

## Coordinate-frame conventions and FAST-LIO2

### CalibAnything

The calibration result is:

```text
T_camera_color_optical_lidar
```

or, abbreviated:

```text
T_color_optical_lidar
```

It satisfies:

```text
p_color_optical = T_color_optical_lidar · p_lidar
```

### FAST-LIO2

FAST-LIO2 expects the LiDAR pose in the IMU frame:

```text
p_imu = R_imu_lidar · p_lidar + t_imu_lidar
```

Therefore its extrinsic is:

```text
T_imu_lidar
```

If the RealSense IMU messages use `camera_imu_optical_frame`, obtain the static
transform from `camera_color_optical_frame` to that exact IMU frame. In
`T_target_source` notation:

```text
T_imu_lidar = T_imu_color_optical · T_color_optical_lidar
```

With ROS TF2, request the transform whose target is the IMU frame and source is
the color optical frame. Confirm direction; inverting the static transform
changes both rotation and translation.

Compose matrices with the helper:

```bash
python3 scripts/compose_fastlio_extrinsic.py \
  --camera-lidar "$RUN_DIR/extrinsic.txt" \
  --imu-camera /path/to/T_imu_color_optical.json \
  --output "$RUN_DIR/fastlio_extrinsic.json"
```

For the resulting matrix `T = T_imu_lidar`, FAST-LIO2 fields are:

```text
extrinsic_T = [T[0,3], T[1,3], T[2,3]]
extrinsic_R = row-major 3x3 rotation
```

`extrinsic_T` is the LiDAR origin expressed in the IMU frame. It is not the
camera position relative to the LiDAR.

If static TFs were not recorded, retrieve them from a live robot, URDF, or
RealSense TF publication. Record the exact frame names and matrix alongside
the calibration result for traceability.

## Step 13 — Convert for FAST-LIO2

1. Identify the frame ID on the actual IMU topic.
2. Obtain `T_imu_color_optical` from the robot's static transform tree.
3. Compose it with CalibAnything's `T_color_optical_lidar`.
4. Extract row-major `extrinsic_R` and `extrinsic_T`.
5. Check the translation against the measured IMU-to-LiDAR lever arm.
6. Save the original matrices and composed result before editing FAST-LIO2.

Never use the raw D435i IMU extrinsic directly as CalibAnything's camera
optical initial transform, and never use the camera optical result directly as
FAST-LIO2's IMU extrinsic unless those frames are demonstrably identical.

## Step 14 — Validation checklist

### Projection

- Refined points follow edges across the full image.
- Floor and wall returns land on the correct surfaces.
- Nearby and distant geometry are both plausible.
- Multiple frame pairs show consistent alignment.
- A deliberately perturbed initial transform produces visibly worse output.

### Physical mounting

- Rotation matches the installed sensor orientations and handedness.
- Translation magnitude matches the measured lever arm.
- Each translation sign is plausible for the robot layout.
- Matrix determinant is near `+1`; rows and columns are orthonormal.

### RViz

- Publish or load the static transform with the exact frame IDs.
- Overlay the colorized/projected cloud while the robot is stationary.
- Inspect floor, vertical walls, poles, and object boundaries.
- Rotate the RViz viewpoint to detect vertical offsets hidden in the camera
  view.

### FAST-LIO2

- Use the exact IMU topic frame expected by the running configuration.
- Start stationary and inspect gravity alignment and map orientation.
- Drive a short loop and check that planar ground remains planar.
- Look for duplicated walls, tilted maps, vertical drift, or oscillation.
- Recheck translation after fixing a rotation error; a correct rotation can
  expose an incorrect lever arm.

## Troubleshooting

### `cv2.imwrite` cannot convert `PosixPath` to `str`

Cause: `ros2_unbag` passes a `pathlib.Path` to an OpenCV binding that expects a
string.

Fix: use this repository's exporter, or patch the call to
`cv2.imwrite(str(path), image)`.

### `CV_HSV2BGR` not declared

Cause: CalibAnything source uses an OpenCV legacy macro.

Fix: replace `CV_HSV2BGR` with `cv::COLOR_HSV2BGR` and rebuild.

### `/usr/bin/ld: cannot find -ljsoncpp`

Cause: jsoncpp development files are missing or CMake links a static archive
that is not installed.

Fix: install the jsoncpp development package and link the imported target or
the verified shared library path, such as
`/usr/lib/x86_64-linux-gnu/libjsoncpp.so`.

### Image path is printed as `dataset//000000`

Cause: wrong schema keys, empty `img_folder`, or missing `img_format`.

Fix: use `img_folder`, `pc_folder`, `mask_folder`, `img_format`, and
`pc_format`; include `".png"` and `".pcd"`; use extension-free stems in
`file_name`; run from the directory against which relative paths are resolved.

### `Point cloud num: 0`

Cause: no PCDs matched the configured folder/format/stems, PCDs are unreadable,
or required fields were lost during export.

Fix: verify `pc/000000.pcd` exists, `pc_format` is `".pcd"`, `file_name`
contains `"000000"`, and the PCD header contains nonzero points plus
`x y z intensity`. Test a PCD independently with PCL or another viewer.

### `Assertion params_.point_range_top < params_.point_range_bottom failed`

Cause: `point_range` is absent, is outside the nested `params` object, or has
invalid ordering for the tested CalibAnything revision.

Fix: follow the example schema and ensure numeric `top < bottom`:

```json
"params": {
  "point_range": {
    "top": 0.0,
    "bottom": 1.0
  }
}
```

### Projection looks good but FAST-LIO2 rotation is wrong

Cause: a LiDAR→camera optical matrix was supplied where LiDAR→IMU was expected,
or a transform was inverted.

Fix: obtain the static camera-optical→IMU transform and compute
`T_imu_lidar = T_imu_color_optical · T_color_optical_lidar`.

### Points appear below or above ground after rotation is fixed

Cause: the frame rotation is now correct but translation still represents the
wrong frame or lever arm.

Fix: recompose the full homogeneous transforms, including translation. Check
the measured IMU-to-LiDAR offset and signs. Do not patch only the rotation
block.

### Translation looks plausible in projection but is physically too large

Image projection can be weakly sensitive to translation in some scenes, and
the optimizer can trade translation against rotation. Reject results that
contradict the actual sensor spacing. Add scene depth variation, tighten the
search range, rerun from a measured initial guess, and validate on independent
frames.

### Initial and refined images are identical

The initial guess may already be near a local optimum, or optimization may
have failed to form useful clusters. Inspect the log for messages such as
`Euclidean cluster number: 0`, compare `extrinsic.txt` numerically, and run a
negative test with a small deliberate perturbation.

### Mask processing says the output directory exists

Remove only the generated processed directory and rerun:

```bash
rm -rf "$DATASET_DIR/processed_masks"
```

Do not delete raw masks unless regeneration is intentional.

## Repository contents

```text
docs/portfolio_calibration_case_study.md  polished project case study
docs/assets/calibration/                 optimized projection examples
examples/calibanything/                  sanitized working config
scripts/export_calib_pairs.py            ROS 2 bag pair exporter
scripts/dump_calib_metadata.py           intrinsics/cloud metadata dumper
scripts/generate_sam2_masks.py           resumable SAM2 mask generation
scripts/make_calibration_montage.py       documentation image builder
scripts/compose_fastlio_extrinsic.py      frame-composition helper
```

Large bags, full image/PCD exports, raw masks, model checkpoints, build trees,
and logs are intentionally excluded.

## Reproduce the documentation images

```bash
python3 -m pip install Pillow
python3 scripts/make_calibration_montage.py \
  /path/to/calibanything_run \
  docs/assets/calibration
```

The checked-in images come from the refined Espeleo run and contain no raw bag
or full dataset.

## Case study and references

- [Portfolio calibration case study](docs/portfolio_calibration_case_study.md)
- [CalibAnything](https://github.com/OpenCalib/CalibAnything)
- [Segment Anything](https://github.com/facebookresearch/segment-anything)
- [SAM 2](https://github.com/facebookresearch/sam2)
- [FAST-LIO2](https://github.com/hku-mars/FAST_LIO)
- [ros2_unbag](https://github.com/ika-rwth-aachen/ros2_unbag)

_Last updated: 2026-07-14_
