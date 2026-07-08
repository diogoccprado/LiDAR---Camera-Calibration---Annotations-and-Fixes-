# LiDAR-Camera Calibration Workflow for ROS 2

Perception infrastructure workflow for extrinsic calibration of a **Livox Mid-360 LiDAR** to an **Intel RealSense D435** RGB camera, using rosbag2, SAM masks, and CalibAnything.

This repository is a **documentation-only workflow** for robot perception pipelines. It captures steps, issues, and fixes from a real calibration run on physical sensors. It does not ship scripts, bags, or calibration outputs.

## Why this matters

Multi-sensor robots need a reliable **LiDAR-to-camera extrinsic transform** before semantic mapping, object detection, or fused perception can work. This workflow documents how to go from raw rosbag2 data to a static TF usable in a semantic mapping stack—with the debugging notes needed to reproduce it.

## Goal

Produce a stable **extrinsic transform (LiDAR → Camera)** for use as a **static TF** in a semantic mapping / robot perception pipeline.

**Expected output:** `calib_result.txt` with the LiDAR-to-camera extrinsic, plus projection validation images (`init_proj*.png`, `refined_proj*.png`).

> No quantitative accuracy metrics are reported in this repo. Validation was qualitative (MD5 comparison, RViz visual check).

## Hardware

| Device | Role | ROS 2 topics |
|--------|------|--------------|
| **Livox Mid-360** | 3D LiDAR | `/livox/lidar` (`sensor_msgs/msg/PointCloud2`) |
| **Intel RealSense D435** | RGB camera | `/camera/color/image_raw`, `/camera/color/camera_info` |

## Tools

| Tool | Purpose |
|------|---------|
| **ROS 2 Jazzy** | Recording, playback, validation |
| **rosbag2 MCAP** | Bag format (`*.mcap` + `metadata.yaml`) |
| [ros2_unbag](https://github.com/ika-rwth-aachen/ros2_unbag) | Export synchronized PNG + PCD pairs from bags |
| [Segment Anything (SAM)](https://github.com/facebookresearch/segment-anything) | Automatic mask generation for calibration targets |
| [CalibAnything](https://github.com/OpenCalib/CalibAnything) | LiDAR-to-camera extrinsic calibration (C++) |
| **RViz** | Visual validation of static TF alignment |

External repos used at runtime (not included here):

- `~/SMN/segment-anything` — SAM installation
- `~/semantic-mapping-nav/CalibAnything` — calibration binary and `processed_mask.py`

## Workflow overview

```
Record rosbag2 → sanity check → ros2_unbag export → prepare dataset
    → SAM masks → post-process masks → build CalibAnything → run calibration → validate
```

### Prerequisites

```bash
export BAG_DIR="$HOME/bagspioneer/<YOUR_BAG_FOLDER>"
export BAG_FILE="<YOUR_BAG_FILE>.mcap"
export OUT_DIR="$HOME/SMN/unbag_out_<DATE>"
export DATA_OK="$HOME/SMN/calibanything_data_<DATE>"
mkdir -p "$OUT_DIR" "$DATA_OK"
```

---

### Step 1 — Record the bag

```bash
ros2 bag record /camera/color/image_raw /camera/color/camera_info /livox/lidar
```

Confirm topics are publishing before recording (`ros2 topic echo -n 1 ...` or RViz).

---

### Step 2 — Bag sanity check

```bash
ros2 bag info "$BAG_DIR/$BAG_FILE"
```

Example counts from one run: ~225 images, ~228 camera_info, ~590 LiDAR clouds.

---

### Step 3 — Export synchronized PNG + PCD pairs

```bash
docker pull ghcr.io/ika-rwth-aachen/ros2_unbag:latest

docker run --rm -it \
  -u "$(id -u)":"$(id -g)" \
  -v "$BAG_DIR:/bags:ro" \
  -v "$OUT_DIR:/out" \
  ghcr.io/ika-rwth-aachen/ros2_unbag:latest \
  ros2 unbag "/bags/$BAG_FILE" \
    --output-dir /out \
    --export /camera/color/image_raw:image/png:images \
    --export /livox/lidar:pointcloud/pcd:pc \
    --resample /camera/color/image_raw:last,5.0 \
    --naming "%name_%index"
```

---

### Step 4 — Prepare CalibAnything dataset

```bash
mkdir -p "$DATA_OK/images" "$DATA_OK/pc"
cp -a "$OUT_DIR/images/." "$DATA_OK/images/"
cp -a "$OUT_DIR/pc/." "$DATA_OK/pc/"
```

Expected layout:

```
$DATA_OK/
  images/
  pc/
  masks/              # raw SAM output
  processed_masks/    # post-processed for CalibAnything
```

Optional numeric rename via symlinks (see [full rename script](#appendix-numeric-rename-script) below).

---

### Step 5 — Run SAM for masks

```bash
python3 -m venv ~/.venvs/sam
source ~/.venvs/sam/bin/activate
pip install torch torchvision torchaudio opencv-python tqdm matplotlib
cd ~/SMN/segment-anything && pip install -e .

export SAM_CKPT="$HOME/Downloads/sam_vit_l_0b3195.pth"
python3 scripts/amg.py \
  --checkpoint "$SAM_CKPT" --model-type vit_l \
  --input "$DATA_OK/images" --output "$DATA_OK/masks" \
  --stability-score-thresh 0.9 --box-nms-thresh 0.5 --stability-score-offset 0.9
```

One run produced 224 images and 222 masks; calibration proceeded with the intersection.

---

### Step 6 — Post-process masks

```bash
cd ~/semantic-mapping-nav/CalibAnything
rm -rf "$DATA_OK/processed_masks"
python3 processed_mask.py -i "$DATA_OK/masks" -o "$DATA_OK/processed_masks"
```

---

### Step 7 — Build CalibAnything

```bash
cd ~/semantic-mapping-nav/CalibAnything
mkdir -p build && cd build
cmake .. && cmake --build . -j"$(nproc)"
```

---

### Step 8 — Run calibration

```bash
cd ~/semantic-mapping-nav/CalibAnything
./bin/run_lidar2camera "$DATA_OK/calib.json"
```

Outputs: `calib_result.txt`, `init_proj*.png`, `refined_proj*.png`.

---

### Step 9 — Validate

```bash
md5sum init_proj.png refined_proj.png init_proj_seg.png refined_proj_seg.png
```

In one run, init and refined images were byte-identical. Recommended follow-up checks:

1. Compare `calib_result.txt` against the initial guess
2. Negative test with a deliberately bad extrinsic
3. RViz static TF visual alignment

---

## Known issues and fixes

| Step | Issue | Fix |
|------|-------|-----|
| Recording | Topics not publishing | Verify with RViz / `ros2 topic echo` before recording |
| Bag check | Topic name mismatch (`/livox/points` vs `/livox/lidar`) | Always use `ros2 bag info` |
| ros2_unbag | "Bag file not found" | Mount to `/bags`, reference `/bags/$BAG_FILE` |
| ros2_unbag | "Unsupported bag extension" | Pass the `.mcap` file, not the directory |
| ros2_unbag | Only ~3 frame pairs exported | Use `--resample /camera/color/image_raw:last,5.0` (~224 pairs) |
| Data prep | `$DATA_OK` unset | Always `export DATA_OK=...` before use |
| SAM | PEP 668 / pip blocked | Use venv at `~/.venvs/sam` |
| SAM | Mid-run crash (OOM) | Batch rerun on missing frames (10 at a time) |
| Masks | `output_dir already exist!` | `rm -rf processed_masks` before rerun |
| Build | `cannot find -ljsoncpp` | Link `jsoncpp` dynamically in `CMakeLists.txt` |
| Calibration | `point_range_top < point_range_bottom` assert | Swap bounds in `calib.json` |
| Validation | Init/refined PNGs identical (MD5) | Initial guess may suffice, or optimizer stuck (`Euclidean cluster number: 0`) |

## Connection to semantic mapping

This calibration produces the **LiDAR-to-camera extrinsic** needed to project 3D point clouds into the camera frame. In a semantic mapping stack, that static TF enables:

- Aligning LiDAR returns with RGB semantic labels
- Fusing dense geometry with image-based segmentation
- Publishing consistent transforms for navigation and perception nodes

CalibAnything lives alongside the semantic mapping project at `~/semantic-mapping-nav/CalibAnything`.

## My contribution

Authored this end-to-end calibration workflow as part of a semantic mapping perception stack.

| Area | Contribution |
|------|--------------|
| **Workflow design** | Full pipeline from rosbag2 recording through CalibAnything calibration and validation |
| **Data export** | ros2_unbag Docker workflow with resample tuning for synchronized PNG/PCD pairs |
| **SAM integration** | Mask generation setup, venv workaround (PEP 668), and batch rerun for OOM recovery |
| **Build fixes** | jsoncpp dynamic linking fix for CalibAnything `CMakeLists.txt` |
| **Calibration debugging** | `point_range` assertion fix, mask post-processing, and validation protocol (MD5, RViz, negative test) |
| **Semantic mapping integration** | Documented static TF output for LiDAR-camera fusion in the perception pipeline |

## Status

| Area | Status |
|------|--------|
| Workflow documentation | Complete (this README) |
| Scripts in this repo | None — inline commands only |
| Calibration accuracy | Not quantified; qualitative validation only |
| Reproducibility | Requires external SAM and CalibAnything repos |

## Quick checklist

1. Record bag with required topics
2. `ros2 bag info` sanity check
3. `ros2_unbag` export (`.mcap`, correct topics, `last,5.0` resample)
4. Copy to `DATA_OK`; normalize naming (optional)
5. Run SAM in venv; batch if crashes; use mask intersection
6. `processed_mask.py` → `processed_masks/`
7. Build CalibAnything; fix jsoncpp link if needed
8. Run calibration; fix `point_range` assert if needed
9. Validate (MD5, RViz static TF, negative test)

## Appendix: numeric rename script

```bash
cd "$DATA_OK"
mkdir -p images_renamed pc_renamed

python3 - <<'PY'
import re
from pathlib import Path

data = Path(".")
img_in, pc_in = data/"images", data/"pc"
img_out, pc_out = data/"images_renamed", data/"pc_renamed"
img_out.mkdir(exist_ok=True)
pc_out.mkdir(exist_ok=True)

img_pat = re.compile(r".*_(\d+)\.png$")
pc_pat  = re.compile(r".*_(\d+)\.pcd$")

for f in sorted(img_in.glob("*.png")):
    m = img_pat.match(f.name)
    if m:
        (img_out / ("%06d.png" % int(m.group(1)))).symlink_to(f)

for f in sorted(pc_in.glob("*.pcd")):
    m = pc_pat.match(f.name)
    if m:
        (pc_out / ("%06d.pcd" % int(m.group(1)))).symlink_to(f)

print("renamed images:", len(list(img_out.glob("*.png"))))
print("renamed pc    :", len(list(pc_out.glob("*.pcd"))))
PY
```

Point `calib.json` to `images_renamed` / `pc_renamed`.

## References

- [CalibAnything](https://github.com/OpenCalib/CalibAnything)
- [Segment Anything](https://github.com/facebookresearch/segment-anything)
- [ros2_unbag](https://github.com/ika-rwth-aachen/ros2_unbag)

_Last updated: 2025-12-16_
