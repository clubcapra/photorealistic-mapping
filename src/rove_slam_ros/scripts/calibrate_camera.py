#!/usr/bin/env python3
"""
All-in-one camera intrinsic calibration tool.

Features
--------
- Smooth live preview (chessboard detection runs in a background thread so the
  slow corner finder never stalls the video feed).
- Live grid overlay on detected boards.
- Manual capture of CLEAN frames to disk (so calibration uses un-annotated images).
- One-key calibration from whatever images are currently in the capture folder.
- Live undistortion preview using the computed intrinsics.
- Calibration result saved to disk and auto-loaded on startup.

Workflow
--------
1. Run the script. A live window opens.
2. Move the chessboard around the frame (vary angle / distance / position).
3. Press SPACE when the grid is detected (green overlay) to save a capture.
4. Collect ~15-25 varied captures.
5. Press C to compute intrinsics from all saved captures.
6. Press U to toggle the live undistorted preview.
7. To drop bad shots: delete files from the capture folder (or press X to delete
   the most recent capture), then press C again to recompute.

Keys
----
  SPACE : save current frame (only when a board is detected)
  C     : (re)compute calibration from all images in the capture folder
  U     : toggle undistorted live preview (after calibration)
  X     : delete the most recently saved capture
  Q/ESC : quit

Examples
--------
  # Default — local USB camera index 0
  calibrate_camera.py

  # Different USB camera
  calibrate_camera.py --source 2

  # IP camera over RTSP
  calibrate_camera.py --source rtsp://user:pass@192.168.1.42:554/stream1

  # IP camera HTTP MJPEG stream
  calibrate_camera.py --source http://192.168.1.42:8080/video

  # Re-calibrate from a folder you've already collected
  calibrate_camera.py --capture-dir my_caps --out my_caps/cam.npz

  # Different chessboard
  calibrate_camera.py --pattern 9x6 --square 24
"""

import argparse
import os
import glob
import time
import threading

import cv2
import numpy as np

# --------------------------------------------------------------------------- #
# Configuration (defaults — every value below is overridable via CLI flags)
# --------------------------------------------------------------------------- #
CAMERA_SOURCE  = 0                 # local USB cam index, or an IP-cam URL string
PATTERN_SIZE   = (9, 6)            # INNER corners (cols, rows). 10x7 squares -> (9, 6)
SQUARE_SIZE    = 24.0              # real-world square edge length (mm, or any unit)
CAPTURE_DIR    = "calib_captures"  # where clean captures are stored
CALIB_FILE     = "calibration.npz" # where intrinsics are saved
DETECT_SCALE   = 0.5               # downscale factor used only for live detection (speed)
# --------------------------------------------------------------------------- #


def parse_source(s):
    """Accept '0' / '2' as local USB index (int) and everything else as a
    URL/path string passed straight through to cv2.VideoCapture (RTSP, HTTP
    MJPEG, file path)."""
    try:
        return int(s)
    except (TypeError, ValueError):
        return s


def open_capture(source):
    """Open cv2.VideoCapture, applying IP-cam-friendly options (1-frame
    buffer keeps latency low; FFMPEG backend is the safest default for
    rtsp:// / http:// streams)."""
    is_url = isinstance(source, str)
    if is_url:
        cap = cv2.VideoCapture(source, cv2.CAP_FFMPEG)
    else:
        cap = cv2.VideoCapture(source)
    if not cap.isOpened():
        return None
    # Shrink the driver-side buffer to one frame so live preview shows the
    # current scene, not a 1-2 s replay. Not all backends honor this; for
    # IP cams that don't, we drop stale frames in the main loop instead.
    try:
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    except Exception:
        pass
    return cap


# --------------------------------------------------------------------------- #
# Background chessboard detector
# --------------------------------------------------------------------------- #
class ChessboardDetector(threading.Thread):
    """
    Runs cv2.findChessboardCorners off the main thread.

    The main loop calls submit(frame) every iteration. This worker grabs the
    most recent frame whenever it finishes its previous detection, so the video
    feed is never blocked by the (slow) corner finder. The latest result is read
    with get_result().
    """

    def __init__(self, pattern_size, detect_scale=0.5):
        super().__init__(daemon=True)
        self.pattern_size = pattern_size
        self.detect_scale = detect_scale

        self._lock = threading.Lock()
        self._input_frame = None
        self._found = False
        self._corners = None          # stored at FULL-resolution coordinates
        self._running = True
        self._new_frame = threading.Event()

        # FAST_CHECK lets the detector bail out quickly when no board is present,
        # which keeps the live feedback responsive.
        self._flags = (cv2.CALIB_CB_ADAPTIVE_THRESH
                       + cv2.CALIB_CB_NORMALIZE_IMAGE
                       + cv2.CALIB_CB_FAST_CHECK)

    def submit(self, frame):
        with self._lock:
            self._input_frame = frame
        self._new_frame.set()

    def get_result(self):
        with self._lock:
            corners = None if self._corners is None else self._corners.copy()
            return self._found, corners

    def stop(self):
        self._running = False
        self._new_frame.set()

    def run(self):
        while self._running:
            # Wait until a new frame is available (with timeout so we can exit).
            self._new_frame.wait(timeout=0.1)
            self._new_frame.clear()

            with self._lock:
                frame = None if self._input_frame is None else self._input_frame.copy()
            if frame is None:
                continue

            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

            # Detect on a downscaled image for speed; scale corners back up after.
            if self.detect_scale != 1.0:
                small = cv2.resize(gray, None,
                                   fx=self.detect_scale, fy=self.detect_scale,
                                   interpolation=cv2.INTER_AREA)
            else:
                small = gray

            found, corners = cv2.findChessboardCorners(
                small, self.pattern_size, flags=self._flags)

            if found and self.detect_scale != 1.0:
                corners = corners / self.detect_scale  # back to full-res coords

            with self._lock:
                self._found = found
                self._corners = corners if found else None


# --------------------------------------------------------------------------- #
# Calibration
# --------------------------------------------------------------------------- #
def build_object_points(pattern_size, square_size):
    """3D coordinates of the chessboard corners in board space (z = 0).

    ``square_size`` may be a scalar (square cells) or a (sx, sy) tuple for
    rectangular cells — useful when the print is scaled non-uniformly. With
    a scalar the cell aspect is forced to 1:1; mismatched-aspect cells in
    that case bleed into fy/fx in the calibration."""
    sx, sy = (square_size if hasattr(square_size, "__len__")
              else (square_size, square_size))
    objp = np.zeros((pattern_size[0] * pattern_size[1], 3), np.float32)
    grid = np.mgrid[0:pattern_size[0], 0:pattern_size[1]].T.reshape(-1, 2)
    objp[:, 0] = grid[:, 0] * sx
    objp[:, 1] = grid[:, 1] * sy
    return objp


def calibrate_from_folder(folder, pattern_size, square_size, model="fisheye"):
    """
    Read every image in `folder`, detect corners at full resolution, and run
    the appropriate OpenCV calibrator.

    model:
      - "pinhole"  : cv2.calibrateCamera, 5-param plumb_bob (default OpenCV
                      model). Good for narrow-FOV / low-distortion lenses.
      - "rational" : cv2.calibrateCamera with CALIB_RATIONAL_MODEL, 8
                      distortion params. Good for moderate wide-angle.
      - "fisheye"  : cv2.fisheye.calibrate, Kannala-Brandt 4-param model.
                      Designed for fisheye / strong-wide-angle lenses where
                      plumb_bob can't fit the distortion and the optimizer
                      absorbs residuals by skewing fy/fx away from 1.0.

    Returns (camera_matrix, dist_coeffs, rms, image_size, model)
    or None on failure.
    """
    paths = sorted(glob.glob(os.path.join(folder, "*.png")))
    if not paths:
        print(f"[calib] No images found in '{folder}'.")
        return None

    criteria_corner = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
    cb_flags = cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_NORMALIZE_IMAGE

    objp = build_object_points(pattern_size, square_size)
    objpoints, imgpoints = [], []
    image_size = None
    used, skipped = 0, 0

    print(f"[calib] Processing {len(paths)} image(s) with model={model}...")
    for p in paths:
        img = cv2.imread(p)
        if img is None:
            print(f"  - {os.path.basename(p)}: could not read, skipping")
            skipped += 1
            continue
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        image_size = gray.shape[::-1]  # (w, h)

        found, corners = cv2.findChessboardCorners(gray, pattern_size, flags=cb_flags)
        if not found:
            print(f"  - {os.path.basename(p)}: board NOT found, skipping")
            skipped += 1
            continue

        corners = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria_corner)
        objpoints.append(objp)
        imgpoints.append(corners)
        used += 1

    if used < 3:
        print(f"[calib] Only {used} usable image(s); need >= 3 (ideally 10+). Aborting.")
        return None

    if model == "fisheye":
        # cv2.fisheye is picky:
        # 1. objpoints want shape (1, N, 3) f64; imgpoints want (1, N, 2) f64.
        # 2. With CALIB_CHECK_COND, it raises on ill-conditioned images
        #    (typical: board too close to a corner or near-parallel to the
        #    optical axis). The well-known recipe is to catch the error,
        #    extract the offending image index from the message, drop it,
        #    and retry until success or we run out of images.
        fo = [o.reshape(1, -1, 3).astype(np.float64) for o in objpoints]
        fi = [c.reshape(1, -1, 2).astype(np.float64) for c in imgpoints]
        f_flags = (cv2.fisheye.CALIB_RECOMPUTE_EXTRINSIC
                   + cv2.fisheye.CALIB_FIX_SKEW
                   + cv2.fisheye.CALIB_CHECK_COND)
        f_criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 200, 1e-9)
        excluded = []
        import re as _re
        while True:
            try:
                K = np.zeros((3, 3))
                D = np.zeros((4, 1))
                rms, K, D, rvecs, tvecs = cv2.fisheye.calibrate(
                    fo, fi, image_size, K, D, flags=f_flags, criteria=f_criteria)
                break
            except cv2.error as e:
                m = _re.search(r"input array (\d+)", str(e))
                if not m or len(fo) < 4:
                    raise
                bad = int(m.group(1))
                print(f"  excluding ill-conditioned image #{bad}")
                excluded.append(bad)
                del fo[bad]; del fi[bad]
        if excluded:
            print(f"[calib] dropped {len(excluded)} ill-conditioned image(s); "
                   f"used {len(fo)} for the final fit.")
        mtx = K
        dist = D
        # Reprojection error
        total_err, total_pts = 0.0, 0
        for i in range(len(fo)):
            proj, _ = cv2.fisheye.projectPoints(fo[i], rvecs[i], tvecs[i], K, D)
            err = cv2.norm(fi[i], proj, cv2.NORM_L2)
            total_err += err ** 2
            total_pts += proj.shape[1]
        mean_err = float(np.sqrt(total_err / total_pts))
    else:
        flags = 0
        if model == "rational":
            flags |= cv2.CALIB_RATIONAL_MODEL
        rms, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(
            objpoints, imgpoints, image_size, None, None, flags=flags)
        # Mean reprojection error (a quality measure; < ~0.5 px is good).
        total_err, total_pts = 0.0, 0
        for i in range(len(objpoints)):
            proj, _ = cv2.projectPoints(objpoints[i], rvecs[i], tvecs[i], mtx, dist)
            err = cv2.norm(imgpoints[i], proj, cv2.NORM_L2)
            total_err += err ** 2
            total_pts += len(proj)
        mean_err = float(np.sqrt(total_err / total_pts))

    print(f"[calib] Done. Used {used}, skipped {skipped}.")
    print(f"[calib] RMS (calibrate): {float(rms):.4f}")
    print(f"[calib] Mean reprojection error: {mean_err:.4f} px")
    print(f"[calib] fy/fx: {mtx[1,1]/mtx[0,0]:.4f}   (1.0 = isotropic)")
    print("[calib] Camera matrix:\n", mtx)
    print("[calib] Distortion coeffs:\n", np.asarray(dist).ravel())

    np.savez(CALIB_FILE,
             camera_matrix=mtx, dist_coeffs=dist,
             image_size=np.array(image_size), reproj_error=mean_err,
             model=np.array(model))
    print(f"[calib] Saved to '{CALIB_FILE}'.")

    return mtx, dist, mean_err, image_size, model


def load_calibration():
    """Load a previously saved calibration, if present."""
    if not os.path.exists(CALIB_FILE):
        return None
    data = np.load(CALIB_FILE, allow_pickle=False)
    mtx = data["camera_matrix"]
    dist = data["dist_coeffs"]
    err = float(data["reproj_error"]) if "reproj_error" in data else -1.0
    size = tuple(int(v) for v in data["image_size"])
    model = (str(data["model"])
             if "model" in data.files else "pinhole")
    print(f"[calib] Loaded existing calibration from '{CALIB_FILE}' "
          f"(model={model}, reproj err {err:.4f} px).")
    return mtx, dist, err, size, model


def build_undistort_maps(camera_matrix, dist_coeffs, image_size, model="pinhole"):
    """Precompute remap tables so live undistortion is fast (remap per frame).

    For fisheye, we also estimate a new K with balance=0 (= fit fully into the
    output frame, no black borders); switch to balance=1 to keep the full FOV
    with letterboxing instead.
    """
    if model == "fisheye":
        new_K = cv2.fisheye.estimateNewCameraMatrixForUndistortRectify(
            camera_matrix, dist_coeffs, image_size, np.eye(3), balance=0.0)
        mapx, mapy = cv2.fisheye.initUndistortRectifyMap(
            camera_matrix, dist_coeffs, np.eye(3), new_K,
            image_size, cv2.CV_16SC2)
    else:
        new_mtx, _ = cv2.getOptimalNewCameraMatrix(
            camera_matrix, dist_coeffs, image_size, alpha=0, newImgSize=image_size)
        mapx, mapy = cv2.initUndistortRectifyMap(
            camera_matrix, dist_coeffs, None, new_mtx,
            image_size, cv2.CV_16SC2)
    return mapx, mapy


# --------------------------------------------------------------------------- #
# HUD
# --------------------------------------------------------------------------- #
def draw_hud(img, lines, color=(255, 255, 255)):
    y = 26
    for text in lines:
        cv2.putText(img, text, (12, y), cv2.FONT_HERSHEY_SIMPLEX,
                    0.6, (0, 0, 0), 3, cv2.LINE_AA)   # outline for legibility
        cv2.putText(img, text, (12, y), cv2.FONT_HERSHEY_SIMPLEX,
                    0.6, color, 1, cv2.LINE_AA)
        y += 26


# --------------------------------------------------------------------------- #
# Main
# --------------------------------------------------------------------------- #
def main():
    # Declared up-front so argparse defaults can reference the module-level
    # values without triggering Python's "use prior to global declaration".
    global CALIB_FILE, CAPTURE_DIR, SQUARE_SIZE
    ap = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    ap.add_argument(
        "--source", default=str(CAMERA_SOURCE),
        help="Camera source. An integer (0, 1, …) opens that USB cam by "
             "index. Anything else is passed straight to cv2.VideoCapture — "
             "so RTSP URLs (rtsp://user:pass@host:554/stream), HTTP MJPEG "
             "(http://host:port/video), or local file paths all work. "
             f"Default: {CAMERA_SOURCE}.",
    )
    ap.add_argument(
        "--pattern", default=f"{PATTERN_SIZE[0]}x{PATTERN_SIZE[1]}",
        help="Chessboard inner-corner count as COLSxROWS "
             f"(default {PATTERN_SIZE[0]}x{PATTERN_SIZE[1]}, i.e. a "
             "10x7-square printed board).",
    )
    ap.add_argument(
        "--square", type=float, default=SQUARE_SIZE,
        help=f"Real-world square edge length in mm — used for both axes "
             f"when --square-x / --square-y are not set (default {SQUARE_SIZE}).",
    )
    ap.add_argument(
        "--square-x", type=float, default=None,
        help="Horizontal cell width in mm. Overrides --square for the x axis. "
             "Use when your printed cells are rectangular (common when "
             "'fit to page' rescaled the chessboard non-uniformly) — without "
             "this the optimizer absorbs the aspect mismatch into fy/fx.",
    )
    ap.add_argument(
        "--square-y", type=float, default=None,
        help="Vertical cell height in mm. Overrides --square for the y axis.",
    )
    ap.add_argument(
        "--input-resize", default=None,
        help="Pre-resize each incoming frame to this WxH before display, "
             "detection, capture, calibration, and undistort. Use when the "
             "camera is outputting a non-square-pixel stream (e.g. a 16:9 "
             "sensor stretched to 4:3) — the IP cam at 192.168.2.33 does "
             "this; calibrate with --input-resize 640x360 to get an "
             "isotropic K. Without this flag the stream is processed at "
             "its native resolution (geometrically correct for downstream "
             "consumers that also see the unresized stream).",
    )
    ap.add_argument(
        "--capture-dir", default=CAPTURE_DIR,
        help=f"Where to save board captures (default {CAPTURE_DIR}).",
    )
    ap.add_argument(
        "--out", default=CALIB_FILE,
        help=f"Where to write the calibration .npz (default {CALIB_FILE}).",
    )
    ap.add_argument(
        "--model", choices=("fisheye", "pinhole", "rational"), default="fisheye",
        help="Distortion model. 'fisheye' (default, Kannala-Brandt 4-param) "
             "is the right choice for wide-angle / fisheye lenses; the "
             "default OpenCV 5-param plumb_bob model ('pinhole') can't fit "
             "their distortion and the optimizer absorbs the residual by "
             "skewing fy/fx away from 1.0. 'rational' is the 8-param "
             "plumb_bob variant — useful for moderate wide-angle.",
    )
    ap.add_argument(
        "--ref-square", type=int, default=200,
        help="Side length (px) of the reference square drawn at the image "
             "center. Hold a physically-square object in front of the camera "
             "and compare aspects — if the on-screen object looks the wrong "
             "shape vs this overlay, the camera is stretching the source "
             "(which explains an fy/fx far from 1.0 in the calibration). "
             "Toggle with the 'S' key. Default 200; set 0 to disable.",
    )
    args = ap.parse_args()

    pattern_size = tuple(int(v) for v in args.pattern.lower().split("x"))
    if len(pattern_size) != 2:
        ap.error("--pattern must be COLSxROWS, e.g. 9x6")

    # Globals are read by calibrate_from_folder() / load_calibration() for
    # the .npz path; rebind so the CLI override flows through.
    CALIB_FILE = args.out
    CAPTURE_DIR = args.capture_dir
    sx = args.square_x if args.square_x is not None else args.square
    sy = args.square_y if args.square_y is not None else args.square
    SQUARE_SIZE = (sx, sy)
    if sx != sy:
        print(f"[init] using rectangular cells: {sx:.2f} x {sy:.2f} mm")

    os.makedirs(CAPTURE_DIR, exist_ok=True)

    source = parse_source(args.source)
    print(f"[init] opening source: {source!r}")
    cap = open_capture(source)
    if cap is None:
        print(f"ERROR: could not open source {source!r}.")
        return

    resize_to = None
    if args.input_resize:
        try:
            rw, rh = (int(v) for v in args.input_resize.lower().split("x"))
            resize_to = (rw, rh)
            print(f"[init] pre-resizing every frame to {rw}x{rh}")
        except Exception:
            ap.error("--input-resize must be WxH (e.g. 640x360)")

    detector = ChessboardDetector(pattern_size, DETECT_SCALE)
    detector.start()

    # Restore any prior calibration so undistort preview works immediately.
    calib = load_calibration()
    undistort_mode = False
    show_ref_square = args.ref_square > 0
    ref_square_px = max(0, args.ref_square)
    maps = None
    if calib is not None:
        _mtx, _dist, _err, _size, _model = calib
        maps = build_undistort_maps(_mtx, _dist, _size, _model)

    capture_count = len(glob.glob(os.path.join(CAPTURE_DIR, "*.png")))
    last_saved_path = None
    status_msg = ""
    status_until = 0.0

    def set_status(msg, seconds=2.0):
        nonlocal status_msg, status_until
        status_msg = msg
        status_until = time.time() + seconds

    window = "Camera Calibration"
    cv2.namedWindow(window, cv2.WINDOW_NORMAL)
    print(__doc__)

    while True:
        ok, frame = cap.read()
        if ok and resize_to is not None and (frame.shape[1], frame.shape[0]) != resize_to:
            frame = cv2.resize(frame, resize_to, interpolation=cv2.INTER_AREA)
        if not ok:
            print("ERROR: failed to read frame.")
            break

        # Hand the latest frame to the detector (non-blocking).
        detector.submit(frame)
        found, corners = detector.get_result()

        # Build the display image.
        if undistort_mode and maps is not None:
            display = cv2.remap(frame, maps[0], maps[1], cv2.INTER_LINEAR)
        else:
            display = frame.copy()
            # Overlay is only meaningful on the raw (distorted) frame.
            if found and corners is not None:
                cv2.drawChessboardCorners(
                    display, pattern_size,
                    corners.astype(np.float32), True)

        # Reference square overlay — fixed-pixel side length, centered. If
        # you hold a real square in front of the camera and it doesn't
        # match this overlay's aspect, the camera output is stretched at
        # the source.
        if show_ref_square and ref_square_px > 0:
            dh, dw = display.shape[:2]
            half = ref_square_px // 2
            x0, y0 = dw // 2 - half, dh // 2 - half
            x1, y1 = x0 + ref_square_px, y0 + ref_square_px
            cv2.rectangle(display, (x0, y0), (x1, y1), (0, 0, 0), 3, cv2.LINE_AA)
            cv2.rectangle(display, (x0, y0), (x1, y1), (0, 255, 255), 1, cv2.LINE_AA)
            cv2.putText(display, f"{ref_square_px}x{ref_square_px} px",
                        (x0, y0 - 6),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 0, 0), 3, cv2.LINE_AA)
            cv2.putText(display, f"{ref_square_px}x{ref_square_px} px",
                        (x0, y0 - 6),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 255), 1, cv2.LINE_AA)

        # HUD
        det_txt = "BOARD DETECTED" if found else "searching..."
        det_col = (0, 230, 0) if found else (0, 200, 255)
        cal_txt = (
            f"{calib[4]} reproj {calib[2]:.3f} px fy/fx {calib[0][1,1]/calib[0][0,0]:.3f}"
            if calib else "not calibrated"
        )
        mode_txt = "UNDISTORTED" if undistort_mode else "RAW"
        lines = [
            f"Captures: {capture_count}   [{det_txt}]",
            f"Mode: {mode_txt}   Calib: {cal_txt}",
            "SPACE save | C calibrate | U undistort | S ref-sq | P dump | X del-last | Q quit",
        ]
        draw_hud(display, lines, color=det_col)
        if status_msg and time.time() < status_until:
            draw_hud_y = display.shape[0] - 16
            cv2.putText(display, status_msg, (12, draw_hud_y),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 0), 4, cv2.LINE_AA)
            cv2.putText(display, status_msg, (12, draw_hud_y),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (50, 220, 255), 1, cv2.LINE_AA)

        cv2.imshow(window, display)
        key = cv2.waitKey(1) & 0xFF

        # ---- key handling ----
        if key in (ord('q'), 27):  # q or ESC
            break

        elif key == ord(' '):
            if found:
                fname = time.strftime("cap_%Y%m%d_%H%M%S")
                # add ms to avoid collisions on fast captures
                fname += f"_{int((time.time() % 1) * 1000):03d}.png"
                path = os.path.join(CAPTURE_DIR, fname)
                cv2.imwrite(path, frame)          # save the CLEAN frame
                last_saved_path = path
                capture_count += 1
                set_status(f"Saved {fname}")
                print(f"[capture] {path}")
            else:
                set_status("No board detected - not saved")

        elif key == ord('c'):
            set_status("Calibrating... (see console)")
            cv2.imshow(window, display)
            cv2.waitKey(1)
            result = calibrate_from_folder(
                CAPTURE_DIR, pattern_size, SQUARE_SIZE, model=args.model)
            if result is not None:
                calib = result
                maps = build_undistort_maps(calib[0], calib[1], calib[3], calib[4])
                set_status(
                    f"Calibrated ({calib[4]})! reproj {calib[2]:.3f} px "
                    f"fy/fx {calib[0][1,1]/calib[0][0,0]:.3f}",
                    3.0,
                )
            else:
                set_status("Calibration failed (see console)", 3.0)

        elif key == ord('u'):
            if maps is not None:
                undistort_mode = not undistort_mode
                set_status("Undistort ON" if undistort_mode else "Undistort OFF")
            else:
                set_status("Calibrate first (press C)")

        elif key == ord('p'):
            # Diagnostic save: dump the CLEAN raw frame + the CLEAN
            # rectified frame (if calibrated), both before any HUD /
            # overlay / window-resize touches them. Lets you check
            # whether visual stretching is in cv2.remap or in the viewer.
            ts = time.strftime("dump_%Y%m%d_%H%M%S")
            ts += f"_{int((time.time() % 1) * 1000):03d}"
            raw_path = f"{ts}_raw_{frame.shape[1]}x{frame.shape[0]}.png"
            cv2.imwrite(raw_path, frame)
            print(f"[dump]  {raw_path}")
            paths = [raw_path]
            if maps is not None:
                undist = cv2.remap(frame, maps[0], maps[1], cv2.INTER_LINEAR)
                undist_path = (f"{ts}_undistorted_"
                                f"{undist.shape[1]}x{undist.shape[0]}.png")
                cv2.imwrite(undist_path, undist)
                print(f"[dump]  {undist_path}")
                paths.append(undist_path)
            set_status("Saved: " + " + ".join(os.path.basename(p) for p in paths), 3.5)

        elif key == ord('s'):
            show_ref_square = not show_ref_square
            set_status("Ref-square ON" if show_ref_square else "Ref-square OFF")

        elif key == ord('x'):
            if last_saved_path and os.path.exists(last_saved_path):
                os.remove(last_saved_path)
                capture_count = max(0, capture_count - 1)
                set_status(f"Deleted {os.path.basename(last_saved_path)}")
                print(f"[delete] {last_saved_path}")
                last_saved_path = None
            else:
                set_status("Nothing to delete")

    # cleanup
    detector.stop()
    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
