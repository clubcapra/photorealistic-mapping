# Camera calibration — workflow + IP-cam stretch discovery

This document covers:

1. Running [`scripts/calibrate_camera.py`](scripts/calibrate_camera.py) on a
   USB or IP camera and getting a usable intrinsic.
2. A non-obvious gotcha discovered on the Rove's cardinal cams: the IP
   stream is vertically stretched, which produces an anisotropic `fy/fx`
   that **is geometrically correct** for the stream — meaning downstream
   tooling needs to match the calibration to the resolution it reads at.
3. What still needs to happen before [`scripts/color_mesh.py`](scripts/color_mesh.py)
   can produce correct color projections from a bag.

## TL;DR

- The IP cam at `192.168.2.33` (the cardinal-cam type) outputs `640×480`
  but its sensor is 16:9 — vertical pixels are stretched by 1.333×.
- This is **not** a bug in OpenCV or in our calibrator. The calibration
  faithfully reports `fy/fx ≈ 1.336` for the as-streamed input, and
  `≈ 1.000` when frames are pre-resized to `640×360`.
- For **bag-replay color mapping** (which feeds `color_mesh.py` 640×480
  frames straight out of `rosbag2_test_camera_lidars/*.db3`), use the
  640×480 calibration as-is. The math is right; pixels just aren't square.
- For **fresh acquisitions** where you control the pipeline end-to-end,
  pre-resize to 640×360 and use the isotropic calibration. Saves
  downstream consumers from having to know about the stretch.

## Section 1 — How to calibrate

### Quick start

```sh
# USB cam
ros2 run rove_slam_ros calibrate_camera.py

# IP cam (this is the one you'll use for the Rove cardinals)
ros2 run rove_slam_ros calibrate_camera.py --source rtsp://192.168.2.33/

# Fresh acquisition, unstretched preview (recommended workflow)
ros2 run rove_slam_ros calibrate_camera.py \
    --source rtsp://192.168.2.33/ \
    --input-resize 640x360
```

Workflow inside the window:

- Move the chessboard around — vary distance, angle, pitch/yaw tilt
- `SPACE` saves a clean capture each time the green grid overlay
  appears
- Collect ~15–25 varied captures
- `C` runs the fit and saves `calibration.npz`
- `U` toggles a live undistorted preview
- `P` dumps the current raw + rectified frames to disk (debug tool —
  use to confirm whether stretch is in the source or the viewer)
- `S` toggles a fixed-pixel reference square at image center (debug
  tool — hold a real square in front of the cam to check aspect)
- `X` deletes the most recent capture; `Q` / `ESC` exits

### Important flags

| Flag | What it does |
|---|---|
| `--source SRC` | USB index (`0`, `2`) or URL string (`rtsp://…`, `http://…`). |
| `--model {fisheye,pinhole,rational}` | Default `fisheye`. The cardinal cams are wide-angle; the default 5-param plumb_bob (`pinhole`) **cannot fit them** and the optimizer absorbs the residual by skewing `fy/fx`. Use `fisheye` unless you know better. |
| `--pattern COLSxROWS` | Inner-corner count of the chessboard. Default `9x6` (a 10×7 printed-square board). |
| `--square MM` | Cell edge length in mm. Default `24` (matches our current printout). |
| `--square-x MM` / `--square-y MM` | Override one axis when the printer scaled the board non-uniformly (rectangular cells). |
| `--input-resize WxH` | Pre-resize every incoming frame before display, detection, capture, calibration, and undistort. Use `640x360` for the IP cam to get an isotropic K. |
| `--out FILE` | Where to write the calibration `.npz`. Default `calibration.npz`. |

## Section 2 — The IP-cam stretch

When the IP cam at `192.168.2.33` was calibrated with all defaults
(640×480 input, fisheye model, 24 mm square cells), the result was:

```
model:        fisheye
image_size:   (640, 480)
reproj_error: 0.4988 px        ← good (< 0.5)
fx = 397.24                    ← horizontal focal length
fy = 530.53                    ← vertical focal length (BIG)
fy / fx = 1.3356               ← suspiciously close to 4/3
cx = 324.79, cy = 273.13       ← principal point, ~image center
D = [-0.09449, 0.07199, -0.14210, 0.09781]   ← Kannala-Brandt k1..k4
```

We chased four wrong hypotheses before landing on the right one:

1. ❌ **Insufficient variety in captures** — re-collected with strong
   pitch/yaw, no change.
2. ❌ **Wrong distortion model** — switched plumb_bob → rational → fisheye,
   `fy/fx` was within 0.001 across all three.
3. ❌ **Non-square printed cells** — measured the printout; cells are
   genuinely 24×24 mm. Telling the calibrator `--square-x 24 --square-y 32`
   produced `fy/fx ≈ 1.0` but with **3.84 px reproj** (vs 0.50 px), proving
   the cells aren't actually rectangular.
4. ✅ **Camera output stream is vertically stretched.** Pre-resizing each
   frame to `640×360` before calibration produced:

   ```
   image_size:   (640, 360)
   reproj_error: 0.5694 px        ← still good
   fy / fx = 1.0013                ← isotropic!
   ```

   So the camera's actual sensor footprint is 640×360 (16:9), and the RTSP
   stream stretches the vertical axis 1.333× to fill a 4:3 output frame.

### Both calibrations are "correct" — pick by use case

| Calibration | image_size | fy/fx | What it's correct for |
|---|---|---|---|
| Default | 640×480 | 1.336 | Reading frames as-delivered (RTSP live, rosbag2 recordings). Use for `color_mesh.py` against the camera-lidar bag. |
| `--input-resize 640x360` | 640×360 | 1.000 | A pipeline that pre-resizes to 640×360 everywhere. Cleaner physically, but every consumer must also resize. |

The 640×480 K with `fy/fx = 1.336` **is not a bad calibration** — it
correctly models a camera with anisotropic effective pixels. Standard
projection math works:

```
u = fx * X/Z + cx
v = fy * Y/Z + cy
```

You just have to remember that for this camera, `fy ≠ fx` is real, not
an artifact.

## Section 3 — What's needed for color mapping

[`scripts/color_mesh.py`](scripts/color_mesh.py) projects mesh vertices
into time-aligned camera images via:

1. URDF → static `T_cam_optical ← base_link` transforms (from
   [`src/rove_description/urdf/rove_standard.urdf`](../rove_description/urdf/rove_standard.urdf))
2. TUM trajectory → time-interpolated `T_map_base`
3. Per camera intrinsics K + D from
   [`config/cam_intrinsics.yaml`](config/cam_intrinsics.yaml)
4. Project + bilinear-sample, weighted by `1/depth²`, average across cams

For this to produce **correct** colors, every K in `cam_intrinsics.yaml`
must match the resolution the bag frames are at (640×480 for the camera-
lidar bag we have), AND each of the four cardinal cams must be calibrated
independently. Today's `cam_intrinsics.yaml` ships **placeholder** values
(fx=fy=320, no distortion) for all four — colors *will* be wrong with
them.

### Calibration checklist before running color mapping

For each of `cam_north`, `cam_south`, `cam_east`, `cam_west`:

- [ ] Acquire the cam's RTSP URL on the Rove network (each cardinal cam
      probably has its own IP, like `192.168.2.33/34/35/36`).
- [ ] Run:
      ```sh
      ros2 run rove_slam_ros calibrate_camera.py \
          --source rtsp://192.168.2.<ip>/ \
          --capture-dir calib_<dir> \
          --out calib_<dir>/cam.npz
      ```
      Don't use `--input-resize` here — we want the K that matches what
      ends up in the rosbag (640×480 with `fy/fx ≈ 1.336` for these cams).
- [ ] Collect ~20 varied captures, press `C` to fit. Confirm reproj < 1 px
      and `fy/fx ≈ 1.336` for sanity (different from 1.0 is expected for
      this camera family).
- [ ] Copy the K and D values into `config/cam_intrinsics.yaml` under
      the matching `cam_<dir>_optical_frame:` block:

      ```yaml
      cam_north_optical_frame:
        width: 640
        height: 480
        fx: 397.24
        fy: 530.53
        cx: 324.79
        cy: 273.13
        distortion_model: equidistant   # = Kannala-Brandt = fisheye
        D: [-0.09449, 0.07199, -0.14210, 0.09781]
      ```

      Note: this YAML is currently consumed by `color_mesh.py` which only
      understands `plumb_bob` and `none`. **TODO** below — see "Action
      items".

### Action items still open

1. **`color_mesh.py` doesn't yet support the fisheye/equidistant model.**
   Its `project_and_sample()` path runs `cv2.projectPoints` (plumb_bob).
   When real fisheye intrinsics get plugged in, it needs to dispatch to
   `cv2.fisheye.projectPoints` based on `distortion_model:`. Roughly a
   20-line change.
2. **The URDF cam→base extrinsics may be inaccurate.** Per
   [`src/rove_description/urdf/README_rove_standard.md`](../rove_description/urdf/README_rove_standard.md),
   the URDF was recovered from a partial bag and might not match the
   physical rig. Calibrating cam→base via PnP from a few synchronized
   lidar-cam frames is a separate task.
3. **The same calibrate_camera flow should also drive the 4 cardinal
   cams**, not just the test cam at `192.168.2.33`. Save each `.npz`
   alongside its captures so the per-cam history is reproducible.
4. **Decide on a single source-of-truth resolution** for the pipeline.
   The bag is 640×480 stretched; if we ever standardize on 640×360
   un-stretched at acquisition, calibrations will need redoing and
   downstream tools (color_mesh, rerun_live) get an `--input-resize`
   knob too.

## Section 4 — Files this commit added

- [`scripts/calibrate_camera.py`](scripts/calibrate_camera.py) — the
  threaded-detection calibration GUI. New flags this session:
  `--source`, `--model`, `--pattern`, `--square`, `--square-x`,
  `--square-y`, `--input-resize`, `--ref-square`, `--out`,
  `--capture-dir`.
- [`config/cam_intrinsics.yaml`](config/cam_intrinsics.yaml) —
  placeholder intrinsics for the four cardinal cams, plus a comment
  block explaining the calibration workflow.
- This document — written 2026-06-03 after a multi-hour debugging
  session that produced the IP-cam-stretch finding above.
