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
"""

import os
import glob
import time
import threading

import cv2
import numpy as np

# --------------------------------------------------------------------------- #
# Configuration
# --------------------------------------------------------------------------- #
CAMERA_INDEX   = 0                 # which camera to open
PATTERN_SIZE   = (9, 6)            # INNER corners (cols, rows). 10x7 squares -> (9, 6)
SQUARE_SIZE    = 24.0              # real-world square edge length (mm, or any unit)
CAPTURE_DIR    = "calib_captures"  # where clean captures are stored
CALIB_FILE     = "calibration.npz" # where intrinsics are saved
DETECT_SCALE   = 0.5               # downscale factor used only for live detection (speed)
# --------------------------------------------------------------------------- #


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
    """3D coordinates of the chessboard corners in board space (z = 0)."""
    objp = np.zeros((pattern_size[0] * pattern_size[1], 3), np.float32)
    objp[:, :2] = np.mgrid[0:pattern_size[0], 0:pattern_size[1]].T.reshape(-1, 2)
    objp *= square_size
    return objp


def calibrate_from_folder(folder, pattern_size, square_size):
    """
    Read every image in `folder`, detect corners at full resolution, and run
    cv2.calibrateCamera. Returns (camera_matrix, dist_coeffs, rms, image_size)
    or None on failure.
    """
    paths = sorted(glob.glob(os.path.join(folder, "*.png")))
    if not paths:
        print(f"[calib] No images found in '{folder}'.")
        return None

    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
    flags = cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_NORMALIZE_IMAGE

    objp = build_object_points(pattern_size, square_size)
    objpoints, imgpoints = [], []
    image_size = None
    used, skipped = 0, 0

    print(f"[calib] Processing {len(paths)} image(s)...")
    for p in paths:
        img = cv2.imread(p)
        if img is None:
            print(f"  - {os.path.basename(p)}: could not read, skipping")
            skipped += 1
            continue
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        image_size = gray.shape[::-1]  # (w, h)

        found, corners = cv2.findChessboardCorners(gray, pattern_size, flags=flags)
        if not found:
            print(f"  - {os.path.basename(p)}: board NOT found, skipping")
            skipped += 1
            continue

        corners = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
        objpoints.append(objp)
        imgpoints.append(corners)
        used += 1

    if used < 3:
        print(f"[calib] Only {used} usable image(s); need >= 3 (ideally 10+). Aborting.")
        return None

    rms, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(
        objpoints, imgpoints, image_size, None, None)

    # Mean reprojection error (a quality measure; < ~0.5 px is good).
    total_err, total_pts = 0.0, 0
    for i in range(len(objpoints)):
        proj, _ = cv2.projectPoints(objpoints[i], rvecs[i], tvecs[i], mtx, dist)
        err = cv2.norm(imgpoints[i], proj, cv2.NORM_L2)
        total_err += err ** 2
        total_pts += len(proj)
    mean_err = np.sqrt(total_err / total_pts)

    print(f"[calib] Done. Used {used}, skipped {skipped}.")
    print(f"[calib] RMS (calibrateCamera): {rms:.4f}")
    print(f"[calib] Mean reprojection error: {mean_err:.4f} px")
    print("[calib] Camera matrix:\n", mtx)
    print("[calib] Distortion coeffs:\n", dist.ravel())

    np.savez(CALIB_FILE,
             camera_matrix=mtx, dist_coeffs=dist,
             image_size=np.array(image_size), reproj_error=mean_err)
    print(f"[calib] Saved to '{CALIB_FILE}'.")

    return mtx, dist, mean_err, image_size


def load_calibration():
    """Load a previously saved calibration, if present."""
    if not os.path.exists(CALIB_FILE):
        return None
    data = np.load(CALIB_FILE)
    mtx = data["camera_matrix"]
    dist = data["dist_coeffs"]
    err = float(data["reproj_error"]) if "reproj_error" in data else -1.0
    size = tuple(int(v) for v in data["image_size"])
    print(f"[calib] Loaded existing calibration from '{CALIB_FILE}' "
          f"(reproj err {err:.4f} px).")
    return mtx, dist, err, size


def build_undistort_maps(camera_matrix, dist_coeffs, image_size):
    """Precompute remap tables so live undistortion is fast (remap per frame)."""
    new_mtx, _ = cv2.getOptimalNewCameraMatrix(
        camera_matrix, dist_coeffs, image_size, alpha=0, newImgSize=image_size)
    mapx, mapy = cv2.initUndistortRectifyMap(
        camera_matrix, dist_coeffs, None, new_mtx, image_size, cv2.CV_16SC2)
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
    os.makedirs(CAPTURE_DIR, exist_ok=True)

    cap = cv2.VideoCapture(CAMERA_INDEX)
    if not cap.isOpened():
        print(f"ERROR: could not open camera index {CAMERA_INDEX}.")
        return

    detector = ChessboardDetector(PATTERN_SIZE, DETECT_SCALE)
    detector.start()

    # Restore any prior calibration so undistort preview works immediately.
    calib = load_calibration()
    undistort_mode = False
    maps = None
    if calib is not None:
        _mtx, _dist, _err, _size = calib
        maps = build_undistort_maps(_mtx, _dist, _size)

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
                    display, PATTERN_SIZE,
                    corners.astype(np.float32), True)

        # HUD
        det_txt = "BOARD DETECTED" if found else "searching..."
        det_col = (0, 230, 0) if found else (0, 200, 255)
        cal_txt = ("calibrated, reproj %.3f px" % calib[2]) if calib else "not calibrated"
        mode_txt = "UNDISTORTED" if undistort_mode else "RAW"
        lines = [
            f"Captures: {capture_count}   [{det_txt}]",
            f"Mode: {mode_txt}   Calib: {cal_txt}",
            "SPACE save | C calibrate | U undistort | X del-last | Q quit",
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
            result = calibrate_from_folder(CAPTURE_DIR, PATTERN_SIZE, SQUARE_SIZE)
            if result is not None:
                calib = result
                maps = build_undistort_maps(calib[0], calib[1], calib[3])
                set_status(f"Calibrated! reproj {calib[2]:.3f} px", 3.0)
            else:
                set_status("Calibration failed (see console)", 3.0)

        elif key == ord('u'):
            if maps is not None:
                undistort_mode = not undistort_mode
                set_status("Undistort ON" if undistort_mode else "Undistort OFF")
            else:
                set_status("Calibrate first (press C)")

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
