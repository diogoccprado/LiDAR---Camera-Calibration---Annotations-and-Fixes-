
# Livox Mid-360 ↔ RealSense D435 Calibration (rosbag2 → ros2_unbag → SAM → CalibAnything)

This note documents the end-to-end workflow we used to calibrate a **Livox Mid-360 LiDAR** to a **RealSense D435 RGB camera** using **CalibAnything**, including the issues we hit and how we worked around them.

> Goal: produce a stable **extrinsic transform (LiDAR → Camera)** that can be used as a static TF in the semantic mapping pipeline.

---

## 0) Prereqs / assumptions

- ROS 2 (Jazzy in our case)
- Bag format: **rosbag2 MCAP** (`*.mcap` + `metadata.yaml`)
- Tools:
  - `ros2 bag`
  - `ros2_unbag` via Docker image `ghcr.io/ika-rwth-aachen/ros2_unbag:latest`
  - Segment Anything (SAM) repo for automatic masks
  - CalibAnything (C++)

Recommended environment variables:

```bash
export BAG_DIR="$HOME/bagspioneer/<YOUR_BAG_FOLDER>"         # contains metadata.yaml + *.mcap
export BAG_FILE="<YOUR_BAG_FILE>.mcap"                      # e.g., rosbag2_..._0-001.mcap

export OUT_DIR="$HOME/SMN/unbag_out_<DATE>"
export DATA_OK="$HOME/SMN/calibanything_data_<DATE>"

mkdir -p "$OUT_DIR" "$DATA_OK"
```

---

## 1) Record the bag (rosbag2)

Record at least:
- `/camera/color/image_raw` (`sensor_msgs/msg/Image`)
- `/camera/color/camera_info` (`sensor_msgs/msg/CameraInfo`)
- `/livox/lidar` (`sensor_msgs/msg/PointCloud2`)

Example (adjust topics):

```bash
ros2 bag record   /camera/color/image_raw   /camera/color/camera_info   /livox/lidar
```

### Problems through testing
- Make sure the topics are **actually publishing** before recording (RViz or `ros2 topic echo -n 1 ...`).
- Ensure camera info is present; CalibAnything needs intrinsics.

---

## 2) Bag sanity check

Run:

```bash
ros2 bag info "$BAG_DIR/$BAG_FILE"
```

We confirmed the bag had:
- `/camera/color/image_raw` — `sensor_msgs/msg/Image` (≈225)
- `/camera/color/camera_info` — `sensor_msgs/msg/CameraInfo` (≈228)
- `/livox/lidar` — `sensor_msgs/msg/PointCloud2` (≈590)

Also checked playback + RViz looked sane.

### Problems through testing
- Topic name mismatches are common (`/livox/points` vs `/livox/lidar`, etc.). Always trust `ros2 bag info`.

---

## 3) Export synchronized PNG + PCD pairs with `ros2_unbag` (Docker)

Pull the image:

```bash
docker pull ghcr.io/ika-rwth-aachen/ros2_unbag:latest
```

Run export (important: pass the **.mcap file**, not just `/bags`):

```bash
mkdir -p "$OUT_DIR"

docker run --rm -it   -u "$(id -u)":"$(id -g)"   -v "$BAG_DIR:/bags:ro"   -v "$OUT_DIR:/out"   ghcr.io/ika-rwth-aachen/ros2_unbag:latest   ros2 unbag "/bags/$BAG_FILE"     --output-dir /out     --export /camera/color/image_raw:image/png:images     --export /livox/lidar:pointcloud/pcd:pc     --resample /camera/color/image_raw:last,5.0     --naming "%name_%index"
```

After this you should have:

```bash
ls "$OUT_DIR/images" | head
ls "$OUT_DIR/pc"     | head
```

### Problems through testing

**(A) “Bag file not found”**
- Cause: passing a host path inside the container (e.g., `~/bagspioneer/...`) instead of `/bags/...`.
- Fix: mount `$BAG_DIR` to `/bags` and use `/bags/$BAG_FILE`.

**(B) “Unsupported bag extension:”**
- Cause: running `ros2 unbag /bags` (a directory). Some setups require the actual `*.mcap`.
- Fix: use `ros2 unbag "/bags/$BAG_FILE"`.

**(C) Only a few frames exported (e.g., 3 pairs)**
- Symptom: it stops very early and warns about dropping frames / topic not available.
- Root cause: strict sync / discard eps / resample choices.
- Fix: switch resample strategy. `last,5.0` worked well to get ~224 pairs.  
  (You can tune the `5.0` if you need stricter/looser matching.)

**(D) Wrong LiDAR topic**
- We initially tried `/livox/points`; the bag actually had `/livox/lidar`.
- Fix: use the exact topic from `ros2 bag info`.

---

## 4) Prepare the CalibAnything dataset folder

CalibAnything expects something like:

```
$DATA_OK/
  images/            # *.png
  pc/                # *.pcd
  masks/             # masks/<frame_id>/*.png  (raw SAM output)
  processed_masks/   # processed_masks/<frame_id>/*.png
```

Copy/move exports:

```bash
mkdir -p "$DATA_OK/images" "$DATA_OK/pc"
cp -a "$OUT_DIR/images/." "$DATA_OK/images/"
cp -a "$OUT_DIR/pc/."     "$DATA_OK/pc/"
```

If you want a clean numeric naming scheme (optional but recommended), you can rename:
- `camera_color_image_raw_000.png` → `000000.png`
- `livox_lidar_000.pcd`            → `000000.pcd`

Example rename script (safe copy via symlink, no data duplication):

```bash
cd "$DATA_OK"
mkdir -p images_renamed pc_renamed

python3 - <<'PY'
import re
from pathlib import Path

data = Path(".")
img_in = data/"images"
pc_in  = data/"pc"
img_out = data/"images_renamed"
pc_out  = data/"pc_renamed"

img_out.mkdir(exist_ok=True)
pc_out.mkdir(exist_ok=True)

img_pat = re.compile(r".*_(\d+)\.png$")
pc_pat  = re.compile(r".*_(\d+)\.pcd$")

for f in sorted(img_in.glob("*.png")):
    m = img_pat.match(f.name)
    if not m:
        continue
    idx = int(m.group(1))
    (img_out / ("%06d.png" % idx)).symlink_to(f)

for f in sorted(pc_in.glob("*.pcd")):
    m = pc_pat.match(f.name)
    if not m:
        continue
    idx = int(m.group(1))
    (pc_out / ("%06d.pcd" % idx)).symlink_to(f)

print("renamed images:", len(list(img_out.glob("*.png"))))
print("renamed pc    :", len(list(pc_out.glob("*.pcd"))))
PY
```

Then point `calib.json` to `images_renamed` / `pc_renamed`.

### Problems through testing
- We accidentally ran checks with `$DATA_OK` unset and got `ls: cannot access '/images'`.
- Fix: always `export DATA_OK=...` before using `$DATA_OK/...`.

---

## 5) Run SAM to generate masks

### 5.1) PEP 668 / “externally-managed-environment” (Ubuntu)
On some Ubuntu installs, `pip install ...` system-wide is blocked.  
**Best practice:** use a per-user venv (does not affect other users; it lives in your home folder).

Create + activate venv:

```bash
python3 -m venv ~/.venvs/sam
source ~/.venvs/sam/bin/activate
python3 -m pip install --upgrade pip
```

Install dependencies (torch varies by CUDA/CPU; use what matches your machine):

```bash
pip install torch torchvision torchaudio
pip install opencv-python tqdm matplotlib
```

Install Segment Anything (repo-based):

```bash
cd ~/SMN/segment-anything
pip install -e .
```

### 5.2) Run automatic mask generation

```bash
export DATA_OK="$HOME/SMN/calibanything_data_<DATE>"
export SAM_CKPT="$HOME/Downloads/sam_vit_l_0b3195.pth"

python3 scripts/amg.py   --checkpoint "$SAM_CKPT"   --model-type vit_l   --input  "$DATA_OK/images"   --output "$DATA_OK/masks"   --stability-score-thresh 0.9   --box-nms-thresh 0.5   --stability-score-offset 0.9
```

### Problems through testing

**(A) `ModuleNotFoundError: No module named 'torch'`**
- Fix: install torch inside the venv.

**(B) `python: command not found`**
- Fix: use `python3` (or install `python-is-python3`, but `python3` is safer).

**(C) Script crash mid-run (TypeError / memory issues)**
We hit an exception mid-processing. Workaround: run SAM in smaller batches and rerun only missing frames.

Count status:
```bash
echo "images: $(ls -1 "$DATA_OK/images" | wc -l)"
echo "masks : $(ls -1 "$DATA_OK/masks"  | wc -l)"
```

Create list of missing frames:
```bash
python3 - <<'PY'
import os
from pathlib import Path

data = Path(os.environ["DATA_OK"])
imgs = sorted([p.stem for p in (data/"images").glob("*.png")])
masks = set([p.name for p in (data/"masks").iterdir() if p.is_dir()])
missing = [f for f in imgs if f not in masks]

out = Path("/tmp/missing_frames.txt")
out.write_text("\n".join(missing) + "\n")
print("missing:", len(missing))
print("wrote:", out)
PY
```

Batch rerun SAM on only missing frames (symlink batch → run → clear batch):
```bash
source ~/.venvs/sam/bin/activate
export DATA_OK="$HOME/SMN/calibanything_data_<DATE>"
export SAM_CKPT="$HOME/Downloads/sam_vit_l_0b3195.pth"
export TMP_IN="/tmp/sam_batch_in"

rm -rf "$TMP_IN"
mkdir -p "$TMP_IN"

count=0
while read -r frame; do
  ln -sf "$DATA_OK/images/${frame}.png" "$TMP_IN/${frame}.png"
  count=$((count+1))

  if [ $count -ge 10 ]; then
    python3 ~/SMN/segment-anything/scripts/amg.py       --checkpoint "$SAM_CKPT"       --model-type vit_l       --input  "$TMP_IN"       --output "$DATA_OK/masks"       --stability-score-thresh 0.9       --box-nms-thresh 0.5       --stability-score-offset 0.9
    rm -rf "$TMP_IN"
    mkdir -p "$TMP_IN"
    count=0
  fi
done < /tmp/missing_frames.txt

# run remaining (<10)
if [ "$(ls -1 "$TMP_IN" 2>/dev/null | wc -l)" -gt 0 ]; then
  python3 ~/SMN/segment-anything/scripts/amg.py     --checkpoint "$SAM_CKPT"     --model-type vit_l     --input  "$TMP_IN"     --output "$DATA_OK/masks"     --stability-score-thresh 0.9     --box-nms-thresh 0.5     --stability-score-offset 0.9
fi

rm -rf "$TMP_IN"
```

We ended with **224 images** but **222 masks**, and proceeded with the intersection.

---

## 6) Post-process masks (CalibAnything)

From CalibAnything repo:

```bash
cd ~/semantic-mapping-nav/CalibAnything
rm -rf "$DATA_OK/processed_masks"
python3 processed_mask.py -i "$DATA_OK/masks" -o "$DATA_OK/processed_masks"
```

### Problems through testing
- `Error: output_dir already exist!`  
  Fix: delete the folder (`rm -rf processed_masks`) or choose a new output path.

---

## 7) Build CalibAnything (C++)

Typical build:

```bash
cd ~/semantic-mapping-nav/CalibAnything
mkdir -p build && cd build
cmake ..
cmake --build . -j"$(nproc)"
```

### Problems through testing: jsoncpp link error

We hit:
```
/usr/bin/ld: cannot find -ljsoncpp
```

But the system had:
- `/usr/lib/x86_64-linux-gnu/libjsoncpp.so`

The link line contained `-Wl,-Bstatic -ljsoncpp -Wl,-Bdynamic`, which forces a **static** jsoncpp that may not exist.

**Fix approach (recommended): link jsoncpp dynamically**
Edit `CMakeLists.txt`:
- Remove `libjsoncpp.a` usage
- Link `jsoncpp` normally (dynamic)

Example minimal change:

```cmake
# remove: target_link_libraries(${PROJECT_NAME} libjsoncpp.a ...)
target_link_libraries(${PROJECT_NAME} jsoncpp ${OpenCV_LIBS} ${Boost_SYSTEM_LIBRARY})
```

Then clean rebuild:

```bash
cd ~/semantic-mapping-nav/CalibAnything
rm -rf build
mkdir build && cd build
cmake ..
cmake --build . -j"$(nproc)"
```

---

## 8) Run calibration

Run (from repo root so outputs save there):

```bash
cd ~/semantic-mapping-nav/CalibAnything
./bin/run_lidar2camera "$DATA_OK/calib.json"
```

Expected outputs:
- `calib_result.txt`
- `init_proj.png`, `init_proj_seg.png`
- `refined_proj.png`, `refined_proj_seg.png`

### Problems through testing: point range assertion
We hit:
```
Assertion `params_.point_range_top < params_.point_range_bottom' failed.
```

Fix: swap the point range bounds in `calib.json` to satisfy what the code expects:
- ensure `point_range_top < point_range_bottom` (even if naming feels inverted).

After fixing that, the calibration completed.

---

## 9) Post-run validation / “refinement identical” case

We checked:
```bash
cd ~/semantic-mapping-nav/CalibAnything
md5sum init_proj.png refined_proj.png init_proj_seg.png refined_proj_seg.png
```

Result: **init and refined images were byte-identical** (same MD5), suggesting:
- either the initial extrinsic guess was already good enough, **or**
- the optimizer didn’t get enough signal to improve (we repeatedly saw `Euclidean cluster number: 0` during processing).

Recommended next checks:
1) Verify `cat calib_result.txt` differs from your initial guess (if it never changes, optimizer may be stuck).
2) Run a “negative test”: intentionally set a bad initial extrinsic and confirm refined differs.
3) Validate in RViz by publishing the resulting transform as a static TF and visually checking alignment.

---

## Appendix: quick checklist (one screen)

1) Record bag with required topics.
2) `ros2 bag info` sanity check.
3) `ros2_unbag` export to `OUT_DIR` (use `.mcap`, correct topic names, `last,5.0` resample).
4) Copy to `DATA_OK` and normalize naming (optional).
5) Run SAM in venv; batch if it crashes; accept intersection (e.g., 222/224).
6) `processed_mask.py` → `processed_masks/`.
7) Build CalibAnything; fix jsoncpp static-link issue if needed.
8) Run calibration; fix point_range assert by swapping bounds.
9) Validate results (MD5, RViz static TF, negative test).


---

_Last updated: 2025-12-16_


## References and Credit:
https://github.com/OpenCalib/CalibAnything
https://github.com/facebookresearch/segment-anything?tab=readme-ov-file
https://github.com/ika-rwth-aachen/ros2_unbag
