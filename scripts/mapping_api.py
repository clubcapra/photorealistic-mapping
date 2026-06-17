#!/usr/bin/env python3
"""
mapping_api.py

Run as a standalone process:
    source /opt/ros/humble/setup.bash
    source ~/capra/photorealistic-mapping/install/setup.bash
    python3 mapping_api.py

The HTTP server runs in the MAIN thread.
ROS spins in a BACKGROUND thread.
Service calls use future.add_done_callback + threading.Event — no busy polling.
"""

import json
import math
import os
import shlex
import subprocess
import datetime
import threading
import time
import yaml
import signal
import sys
import uuid

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.qos import (
    QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy, QoSHistoryPolicy
)
from nav_msgs.msg import OccupancyGrid
from std_srvs.srv import Empty
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType
from tf2_ros import Buffer, TransformListener
from scipy.ndimage import rotate as ndimage_rotate
import numpy as np
from http.server import HTTPServer, BaseHTTPRequestHandler

# ── QoS profiles ─────────────────────────────────────────────────────────────
QOS_TRANSIENT = QoSProfile(
    reliability=QoSReliabilityPolicy.RELIABLE,
    durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=1,
)
QOS_SENSOR = QoSProfile(
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    durability=QoSDurabilityPolicy.VOLATILE,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=1,
)

# ── Settings ──────────────────────────────────────────────────────────────────
HTTP_HOST   = "0.0.0.0"
HTTP_PORT   = 8888
MAP_TOPIC   = "/grid_prob_map"
ROBOT_FRAME = "Core"
MAP_FRAME   = "new_map"
LOCAL_SIZE  = 200

EXPORT_DIR         = "/mnt/ssd/sftp/maps"
RTABMAP_DB         = "/mnt/ssd/sftp/rtabmapdb/rtabmap.db"
POIS_PATH          = "/mnt/ssd/sftp/maps/pois.json"
RTABMAP_EXPORT_BIN   = "/opt/ros/humble/bin/rtabmap-export"
COLOR_PROJECTOR_PATH = os.path.join(
    os.path.dirname(os.path.abspath(__file__)), "color_projector.py"
)
CONFIG_PATH        = os.path.join(
    os.path.dirname(os.path.abspath(__file__)), "config", "rtabmap.yaml"
)
LAUNCH_LOG_PATH    = "/mnt/ssd/sftp/logs/run_launch.log"
LAUNCH_PID_PATH    = "/mnt/ssd/sftp/logs/run_launch.pid"
_workspace = os.environ.get(
    "ROVE_WS",
    "/home/nathan/capra/photorealistic-mapping"  # laptop default
)
LAUNCH_CMD         = (
    f"bash -c 'source /opt/ros/humble/setup.bash && "
    f"source {_workspace}/install/setup.bash && "
    f"ros2 launch rove_color_mapping run.launch.py'"
)
SVC_PAUSE     = "/rtabmap/pause"
SVC_RESUME    = "/rtabmap/resume"
SVC_RESET     = "/rtabmap/reset"
SVC_TRIGGER   = "/rtabmap/trigger_new_map"
SVC_SET_PARAM = "/rtabmap/set_parameters"

# ── Shared state ──────────────────────────────────────────────────────────────
_state_lock     = threading.Lock()
_snapshot       = None
_mapping_state  = "unknown"
_node_count     = 0
_loop_closures  = 0

_launch_process = None
_launch_lock    = threading.Lock()
_launch_log_fh  = None   # open file handle for the log


def _restore_launch_process():
    """Re-attach to a running launch process from a previous API session."""
    global _launch_process
    pid = _load_pid()
    if pid is None:
        return
    # Wrap the existing PID in a Popen-like object using psutil approach:
    # We cannot get a real Popen handle back, so use a sentinel object
    # that supports .poll() and .terminate()/.kill() via os.kill()
    class _ExternalProcess:
        def __init__(self, pid):
            self.pid = pid
            self.returncode = None
        def poll(self):
            try:
                os.kill(self.pid, 0)
                return None   # still running
            except ProcessLookupError:
                self.returncode = -1
                return -1
            except PermissionError:
                return None   # running but not ours
        def terminate(self):
            try: os.kill(self.pid, 15)
            except Exception: pass
        def kill(self):
            try: os.kill(self.pid, 9)
            except Exception: pass
        def wait(self, timeout=None):
            import time as _t
            deadline = _t.time() + (timeout or 30)
            while _t.time() < deadline:
                if self.poll() is not None:
                    return
                _t.sleep(0.2)
            raise subprocess.TimeoutExpired(str(self.pid), timeout)
    with _launch_lock:
        _launch_process = _ExternalProcess(pid)
    print(f"[mapping_api] re-attached to existing launch process pid={pid}", flush=True)

def _load_pois() -> dict:
    """Load POIs from disk on startup."""
    if not os.path.isfile(POIS_PATH):
        return {}
    try:
        with open(POIS_PATH) as f:
            data = json.load(f)
        pois = {p["id"]: p for p in data.get("pois", []) if "id" in p}
        print(f"[mapping_api] Loaded {len(pois)} POIs from {POIS_PATH}", flush=True)
        return pois
    except Exception as e:
        print(f"[mapping_api] Could not load POIs: {e}", flush=True)
        return {}


def _save_pois(pois: dict) -> None:
    """Persist POIs to disk immediately."""
    try:
        os.makedirs(os.path.dirname(POIS_PATH), exist_ok=True)
        with open(POIS_PATH, "w") as f:
            json.dump({
                "pois":       list(pois.values()),
                "saved_at":   datetime.datetime.utcnow().isoformat(),
            }, f, indent=2)
    except Exception as e:
        print(f"[mapping_api] Could not save POIs: {e}", flush=True)


# ── POIs, robot pose, path history ───────────────────────────────────────────
_pois      = _load_pois()  # id → {id, name, type, x, y, z, yaw, timestamp} — persisted to disk
_poi_lock  = threading.Lock()
_robot_pose = None       # latest {x, y, z, roll, pitch, yaw, timestamp}
_path      = []          # list of {x, y, z, timestamp} — appended each map update


def _sqlite_stats() -> dict:
    """Read node count and odometry link count directly from the DB."""
    result = {"node_count": 0, "odom_links": 0, "loop_closures": 0, "db_size_mb": 0.0}
    if not os.path.isfile(RTABMAP_DB):
        return result
    try:
        result["db_size_mb"] = round(os.path.getsize(RTABMAP_DB) / 1e6, 2)
    except OSError:
        pass
    try:
        import sqlite3 as _sqlite3
        con = _sqlite3.connect(f"file:{RTABMAP_DB}?mode=ro", uri=True, timeout=2)
        cur = con.cursor()
        result["node_count"]    = cur.execute("SELECT COUNT(*) FROM Node").fetchone()[0]
        result["odom_links"]    = cur.execute(
            "SELECT COUNT(*) FROM Link WHERE type=0"
        ).fetchone()[0]
        result["loop_closures"] = cur.execute(
            "SELECT COUNT(*) FROM Link WHERE type=1"
        ).fetchone()[0]
        con.close()
    except Exception:
        pass
    return result


def get_status(node=None) -> dict:
    global _mapping_state

    # Check rtabmap service availability
    rtabmap_alive = False
    if node is not None:
        try:
            rtabmap_alive = node.cli_resume.service_is_ready()
        except Exception:
            pass

    # Update internal state based on service availability
    with _state_lock:
        state = _mapping_state
    if not rtabmap_alive:
        if state in ("running", "paused"):
            with _state_lock:
                _mapping_state = "unknown"
            state = "unknown"
        state = "rtabmap not running"
    elif state == "unknown":
        state = "running (no frames yet)"

    # Read ground-truth counts from DB — more reliable than /rtabmap/info
    stats = _sqlite_stats()

    return {
        "state":          state,
        "rtabmap_alive":  rtabmap_alive,
        "node_count":     stats["node_count"],
        "odom_links":     stats["odom_links"],
        "loop_closures":  stats["loop_closures"],
        "db_size_mb":     stats["db_size_mb"],
        "timestamp":      time.time(),
        "export_ready":   stats["node_count"] >= 2 and stats["odom_links"] >= 1,
    }


def get_launch_log() -> str:
    """Read the entire launch log file, or return empty string if not found."""
    if not os.path.isfile(LAUNCH_LOG_PATH):
        return ""
    try:
        with open(LAUNCH_LOG_PATH, "r", errors="replace") as f:
            return f.read()
    except Exception as e:
        return f"(could not read log: {e})"


def _save_pid(pid: int) -> None:
    try:
        os.makedirs(os.path.dirname(LAUNCH_PID_PATH), exist_ok=True)
        with open(LAUNCH_PID_PATH, "w") as f:
            f.write(str(pid))
    except Exception:
        pass


def _load_pid() -> int | None:
    """Read saved PID. Returns None if file missing or process no longer exists."""
    try:
        with open(LAUNCH_PID_PATH) as f:
            pid = int(f.read().strip())
        # Verify process is actually still alive
        os.kill(pid, 0)
        return pid
    except Exception:
        return None


def _clear_pid() -> None:
    try:
        os.remove(LAUNCH_PID_PATH)
    except Exception:
        pass


# ─────────────────────────────────────────────────────────────────────────────
# ROS node
# ─────────────────────────────────────────────────────────────────────────────

class MappingNode(Node):
    def __init__(self):
        super().__init__("mapping_api")
        cb = ReentrantCallbackGroup()

        self.tf_buffer   = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.create_subscription(
            OccupancyGrid, MAP_TOPIC, self._on_map,
            QOS_TRANSIENT, callback_group=cb
        )

        try:
            from rtabmap_msgs.msg import Info
            self.create_subscription(
                Info, "/rtabmap/info", self._on_info,
                10, callback_group=cb
            )
            self.get_logger().info("Subscribed to /rtabmap/info")
        except Exception:
            self.get_logger().warn("rtabmap_msgs unavailable — node_count disabled")

        self.cli_pause     = self.create_client(Empty, SVC_PAUSE,    callback_group=cb)
        self.cli_resume    = self.create_client(Empty, SVC_RESUME,   callback_group=cb)
        self.cli_reset     = self.create_client(Empty, SVC_RESET,    callback_group=cb)
        self.cli_trigger   = self.create_client(Empty, SVC_TRIGGER,  callback_group=cb)
        self.cli_set_param = self.create_client(
            SetParameters, SVC_SET_PARAM, callback_group=cb
        )
        # Subscribe to rtabmap's path topic for trajectory recording
        try:
            from nav_msgs.msg import Path
            self.create_subscription(
                Path, "/rtabmap/mapPath", self._on_path,
                QOS_TRANSIENT, callback_group=cb
            )
        except Exception:
            pass

        self.get_logger().info(f"mapping_api node ready — HTTP on :{HTTP_PORT}")

    # ── callbacks ─────────────────────────────────────────────────────────────

    def _on_info(self, msg) -> None:
        global _mapping_state, _node_count, _loop_closures
        with _state_lock:
            if _mapping_state == "unknown":
                _mapping_state = "running"
            if hasattr(msg, "nodes_count"):
                _node_count = msg.nodes_count
            if getattr(msg, "loop_closure_id", 0) > 0:
                _loop_closures += 1

    def _on_path(self, msg) -> None:
        """Store the full robot path from rtabmap/mapPath."""
        global _path
        pts = []
        for pose_stamped in msg.poses:
            p = pose_stamped.pose.position
            pts.append({"x": p.x, "y": p.y, "z": p.z,
                        "timestamp": pose_stamped.header.stamp.sec +
                                     pose_stamped.header.stamp.nanosec * 1e-9})
        with _state_lock:
            _path = pts

    def _on_map(self, msg: OccupancyGrid) -> None:
        global _snapshot
        rx, ry, yaw, found = 0.0, 0.0, 0.0, False
        try:
            tf = self.tf_buffer.lookup_transform(
                MAP_FRAME, ROBOT_FRAME,
                rclpy.time.Time(seconds=0, nanoseconds=0,
                                clock_type=rclpy.clock.ClockType.SYSTEM_TIME),
            )
            rx  = tf.transform.translation.x
            ry  = tf.transform.translation.y
            q   = tf.transform.rotation
            yaw = math.atan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y*q.y + q.z*q.z))
            found = True
        except Exception:
            pass

        res    = msg.info.resolution
        ox, oy = msg.info.origin.position.x, msg.info.origin.position.y
        mw, mh = msg.info.width, msg.info.height
        cx     = int((rx - ox) / res)
        cy     = int((ry - oy) / res)
        half   = LOCAL_SIZE // 2

        grid = np.array(msg.data, dtype=np.int8).reshape((mh, mw))
        crop = np.full((LOCAL_SIZE, LOCAL_SIZE), -1, dtype=np.int8)

        sr0 = cy-half; sr1 = cy+half; sc0 = cx-half; sc1 = cx+half
        dr0 = max(0,-sr0); dr1 = LOCAL_SIZE - max(0, sr1-mh)
        dc0 = max(0,-sc0); dc1 = LOCAL_SIZE - max(0, sc1-mw)
        sr0 = max(0,sr0); sr1 = min(mh,sr1)
        sc0 = max(0,sc0); sc1 = min(mw,sc1)

        if sr1 > sr0 and sc1 > sc0:
            crop[dr0:dr1, dc0:dc1] = grid[sr0:sr1, sc0:sc1]

        crop = np.flipud(crop)
        crop = ndimage_rotate(crop, math.degrees(yaw)+90.0,
                              reshape=False, order=0, cval=-1)
        # Store full 3D pose
        rz, roll, pitch = 0.0, 0.0, 0.0
        if found:
            try:
                tf3 = self.tf_buffer.lookup_transform(
                    MAP_FRAME, ROBOT_FRAME,
                    rclpy.time.Time(seconds=0, nanoseconds=0,
                                    clock_type=rclpy.clock.ClockType.SYSTEM_TIME),
                )
                rz    = tf3.transform.translation.z
                q     = tf3.transform.rotation
                # roll, pitch, yaw from quaternion
                sinr  = 2*(q.w*q.x + q.y*q.z)
                cosr  = 1 - 2*(q.x*q.x + q.y*q.y)
                roll  = math.atan2(sinr, cosr)
                sinp  = 2*(q.w*q.y - q.z*q.x)
                pitch = math.asin(max(-1.0, min(1.0, sinp)))
            except Exception:
                pass

        global _robot_pose
        with _state_lock:
            if found:
                _robot_pose = {
                    "x": rx, "y": ry, "z": rz,
                    "roll": roll, "pitch": pitch, "yaw": yaw,
                    "timestamp": time.time(),
                }
            _snapshot = {
                "timestamp":   time.time(),
                "local_size":  LOCAL_SIZE,
                "resolution":  res,
                "robot_x":     rx, "robot_y": ry, "robot_yaw": yaw,
                "robot_found": found,
                "data":        crop.tolist(),
            }

    # ── service helpers ───────────────────────────────────────────────────────

    def call_empty(self, client, svc_name: str, timeout: float = 5.0):
        if not client.wait_for_service(timeout_sec=timeout):
            return False, f"{svc_name} not available — is rtabmap running?"

        event  = threading.Event()
        result = [None, None]

        def _done(future):
            if future.exception():
                result[0] = False
                result[1] = str(future.exception())
            else:
                result[0] = True
                result[1] = "ok"
            event.set()

        client.call_async(Empty.Request()).add_done_callback(_done)

        if not event.wait(timeout=timeout):
            return False, f"{svc_name} timed out after {timeout}s"
        return result[0], result[1]

    def call_set_params(self, params_dict: dict, timeout: float = 5.0) -> dict:
        if not self.cli_set_param.wait_for_service(timeout_sec=2.0):
            return {"ok": False, "message": f"{SVC_SET_PARAM} not available"}

        ros_params = []
        for k, v in params_dict.items():
            pv = ParameterValue()
            if isinstance(v, bool):
                pv.type = ParameterType.PARAMETER_BOOL;    pv.bool_value    = v
            elif isinstance(v, int):
                pv.type = ParameterType.PARAMETER_INTEGER; pv.integer_value = v
            elif isinstance(v, float):
                pv.type = ParameterType.PARAMETER_DOUBLE;  pv.double_value  = v
            elif isinstance(v, str):
                pv.type = ParameterType.PARAMETER_STRING;  pv.string_value  = v
            else:
                continue
            p = Parameter(); p.name = k; p.value = pv
            ros_params.append(p)

        if not ros_params:
            return {"ok": False, "message": "No settable parameters found"}

        event  = threading.Event()
        result = [None]
        error  = [None]

        def _done(future):
            if future.exception():
                error[0] = str(future.exception())
            else:
                result[0] = future.result()
            event.set()

        req = SetParameters.Request()
        req.parameters = ros_params
        self.cli_set_param.call_async(req).add_done_callback(_done)

        if not event.wait(timeout=timeout):
            return {"ok": False, "message": "set_parameters timed out"}
        if error[0]:
            return {"ok": False, "message": error[0]}

        failed = [ros_params[i].name
                  for i, r in enumerate(result[0].results) if not r.successful]
        if failed:
            return {"ok": False, "message": f"Failed: {failed}",
                    "loaded": len(ros_params) - len(failed)}
        return {"ok": True, "message": f"Applied {len(ros_params)} parameters"}


# ─────────────────────────────────────────────────────────────────────────────
# Actions
# ─────────────────────────────────────────────────────────────────────────────

def action_start(node: MappingNode) -> dict:
    global _mapping_state
    with _state_lock:
        state = _mapping_state
    if state == "running":
        return {"ok": True, "message": "already running"}
    ok, msg = node.call_empty(node.cli_resume, SVC_RESUME)
    if ok:
        with _state_lock: _mapping_state = "running"
    return {"ok": ok, "message": msg}


def action_pause(node: MappingNode) -> dict:
    global _mapping_state
    with _state_lock:
        if _mapping_state == "paused":
            return {"ok": True, "message": "already paused"}
    ok, msg = node.call_empty(node.cli_pause, SVC_PAUSE)
    if ok:
        with _state_lock: _mapping_state = "paused"
    return {"ok": ok, "message": msg}


def action_reset(node: MappingNode) -> dict:
    global _mapping_state, _node_count, _loop_closures
    with _state_lock:
        is_paused = _mapping_state == "paused"
    if is_paused:
        node.call_empty(node.cli_resume, SVC_RESUME)
        time.sleep(0.3)
    ok, msg = node.call_empty(node.cli_reset, SVC_RESET)
    if ok:
        with _state_lock:
            _mapping_state = "running"; _node_count = 0; _loop_closures = 0
    return {"ok": ok, "message": msg}


def action_new_map(node: MappingNode) -> dict:
    ok, msg = node.call_empty(node.cli_trigger, SVC_TRIGGER)
    return {"ok": ok, "message": msg}


def action_export(node: MappingNode, filename) -> dict:
    global _mapping_state
    if not os.path.isfile(RTABMAP_DB):
        return {"ok": False, "message": f"DB not found: {RTABMAP_DB}",
                "path": "", "size_mb": 0, "duration_s": 0}

    os.makedirs(EXPORT_DIR, exist_ok=True)
    if not filename:
        filename = f"scan_{datetime.datetime.utcnow().strftime('%Y%m%d_%H%M%S')}.ply"
    filename = os.path.basename(filename)
    if not filename.lower().endswith(".ply"):
        filename += ".ply"
    stem = filename[:-4]

    with _state_lock:
        state = _mapping_state
    was_running = state in ("running", "unknown")
    if was_running:
        ok, _ = node.call_empty(node.cli_pause, SVC_PAUSE)
        if ok:
            with _state_lock: _mapping_state = "paused"
        # Give rtabmap time to flush in-memory nodes to the DB
        time.sleep(2.0)

    cmd = [RTABMAP_EXPORT_BIN,
           "--scan",                       # LiDAR scan data (not depth/stereo)
           "--scan_voxel",     "0.01",     # assemble_voxel=0.01
           "--scan_decimation","1",        # regenerate_decimation=1
           "--scan_max_range", "4.0",      # regenerate_max_depth=4
           "--mls",                        # Moving Least Squares smoothing
           "--mls_radius",     "0.1",      # mls_radius=0.1
           "--normals_k",      "20",       # normals_k=20
           "--binary",                     # binary PLY (binary=true, smaller file)
           "--ply",
           "--output_dir", EXPORT_DIR,
           "--output",     stem,
           RTABMAP_DB]
    t0 = time.time()
    try:
        proc = subprocess.run(cmd, capture_output=True, text=True, timeout=300)
    except FileNotFoundError:
        result = {"ok": False, "message": f"rtabmap-export not found: {RTABMAP_EXPORT_BIN}",
                  "path": "", "size_mb": 0, "duration_s": round(time.time()-t0, 1)}
    except subprocess.TimeoutExpired:
        result = {"ok": False, "message": "timed out after 300s",
                  "path": "", "size_mb": 0, "duration_s": 300}
    else:
        dur = round(time.time() - t0, 1)
        if proc.returncode != 0:
            result = {"ok": False,
                      "message": proc.stderr.strip() or proc.stdout.strip() or "failed",
                      "path": "", "size_mb": 0, "duration_s": dur}
        else:
            candidates = [os.path.join(EXPORT_DIR, filename),
                          os.path.join(EXPORT_DIR, stem + "_cloud.ply"),
                          os.path.join(EXPORT_DIR, stem + "0.ply")]
            found = next((c for c in candidates if os.path.isfile(c)), None)
            if not found:
                plys = sorted(
                    [os.path.join(EXPORT_DIR, f) for f in os.listdir(EXPORT_DIR)
                     if f.endswith(".ply")], key=os.path.getmtime)
                found = plys[-1] if plys else None
            result = ({"ok": False, "message": "no PLY after export",
                       "path": "", "size_mb": 0, "duration_s": dur}
                      if not found else
                      {"ok": True, "message": "Export complete",
                       "path": found,
                       "size_mb": round(os.path.getsize(found) / 1e6, 2),
                       "duration_s": dur})

    if was_running:
        ok2, msg2 = node.call_empty(node.cli_resume, SVC_RESUME)
        if ok2:
            with _state_lock: _mapping_state = "running"
        else:
            result["message"] += f" (WARNING: resume failed: {msg2})"

    # ── Export POIs as JSON ───────────────────────────────────────────────────
    with _poi_lock:
        pois_snapshot = list(_pois.values())
    poi_path = os.path.join(EXPORT_DIR, stem + "_pois.json")
    try:
        with open(poi_path, "w") as f:
            json.dump({"pois": pois_snapshot,
                       "exported_at": datetime.datetime.utcnow().isoformat()}, f, indent=2)
        result["pois_path"] = poi_path
        result["pois_count"] = len(pois_snapshot)
    except Exception as e:
        result["pois_warning"] = f"Could not write POIs: {e}"

    # ── Export path as JSON ───────────────────────────────────────────────────
    with _state_lock:
        path_snapshot = list(_path)
    path_path = os.path.join(EXPORT_DIR, stem + "_path.json")
    try:
        with open(path_path, "w") as f:
            json.dump({"path": path_snapshot,
                       "point_count": len(path_snapshot),
                       "exported_at": datetime.datetime.utcnow().isoformat()}, f, indent=2)
        result["path_path"] = path_path
        result["path_points"] = len(path_snapshot)
    except Exception as e:
        result["path_warning"] = f"Could not write path: {e}"

    # ── Export 2D occupancy grid as PGM + YAML (standard ROS map format) ─────
    with _state_lock:
        snap = _snapshot
    if snap and snap.get("data"):
        pgm_path  = os.path.join(EXPORT_DIR, stem + "_map.pgm")
        yaml_path = os.path.join(EXPORT_DIR, stem + "_map.yaml")
        try:
            grid = np.array(snap["data"], dtype=np.int8)
            h, w = grid.shape
            # Convert occupancy (-1=unknown, 0=free, 100=occupied) → PGM grey
            # ROS convention: 205=unknown, 254=free, 0=occupied
            pgm = np.full((h, w), 205, dtype=np.uint8)
            pgm[grid == 0]   = 254
            pgm[grid == 100] = 0
            # Write PGM (P5 binary)
            with open(pgm_path, "wb") as f:
                pgm_header = ("P5\n" + str(w) + " " + str(h) + "\n255\n")
                f.write(pgm_header.encode())
                f.write(pgm.tobytes())
            # Write companion YAML
            res = snap["resolution"]
            map_meta = {
                "image":      os.path.basename(pgm_path),
                "resolution": res,
                "origin":     [0.0, 0.0, 0.0],
                "negate":     0,
                "occupied_thresh": 0.65,
                "free_thresh":     0.196,
                "exported_at": datetime.datetime.utcnow().isoformat(),
            }
            with open(yaml_path, "w") as f:
                yaml.dump(map_meta, f)
            result["map_pgm_path"]  = pgm_path
            result["map_yaml_path"] = yaml_path
        except Exception as e:
            result["map_warning"] = f"Could not write 2D map: {e}"
    else:
        result["map_warning"] = "No 2D map snapshot available yet"

    # ── Color projection from all 4 cameras ──────────────────────────────────
    if result.get("ok") and result.get("path") and os.path.isfile(COLOR_PROJECTOR_PATH):
        ply_path     = result["path"]
        colored_path = ply_path.replace("_cloud.ply", "_colored.ply").replace(".ply", "_colored.ply")
        cameras_json = os.path.join(EXPORT_DIR, "..", "images", "cameras.json")
        cameras_json = os.path.normpath(cameras_json)
        images_dir   = os.path.normpath(os.path.join(EXPORT_DIR, "..", "images"))

        if not os.path.isfile(cameras_json):
            result["color_warning"] = (
                "cameras.json not found — image_recorder.py may not have run. "
                "Skipping color projection."
            )
        elif not os.path.isdir(images_dir):
            result["color_warning"] = "Images directory not found — skipping color projection."
        else:
            try:
                color_cmd = [
                    sys.executable, COLOR_PROJECTOR_PATH,
                    ply_path,
                    "--output",  colored_path,
                    "--images",  images_dir,
                    "--cameras", cameras_json,
                ]
                t_color = time.time()
                proc2 = subprocess.run(
                    color_cmd, capture_output=True, text=True, timeout=600
                )
                color_dur = round(time.time() - t_color, 1)
                if proc2.returncode == 0 and os.path.isfile(colored_path):
                    result["colored_path"]     = colored_path
                    result["colored_size_mb"]  = round(
                        os.path.getsize(colored_path) / 1e6, 2
                    )
                    result["color_duration_s"] = color_dur
                    result["message"]         += " + color projection complete"
                else:
                    result["color_warning"] = (
                        f"Color projection failed after {color_dur}s: "
                        + (proc2.stderr.strip() or proc2.stdout.strip() or "unknown error")
                    )
            except subprocess.TimeoutExpired:
                result["color_warning"] = "Color projection timed out after 600s"
            except Exception as e:
                result["color_warning"] = f"Color projection error: {e}"
    elif not os.path.isfile(COLOR_PROJECTOR_PATH):
        result["color_warning"] = f"color_projector.py not found at {COLOR_PROJECTOR_PATH}"

    return result


def action_config(node: MappingNode, path) -> dict:
    cfg = os.path.expandvars(os.path.expanduser(path or CONFIG_PATH))
    if not os.path.isfile(cfg):
        return {"ok": False, "message": f"File not found: {cfg}"}
    try:
        with open(cfg) as f:
            raw = yaml.safe_load(f)
    except Exception as e:
        return {"ok": False, "message": f"YAML error: {e}"}

    params = (raw["rtabmap"]["ros__parameters"]
              if isinstance(raw.get("rtabmap"), dict)
                 and "ros__parameters" in raw["rtabmap"]
              else raw)
    result = node.call_set_params(params)
    if result["ok"]:
        result["path"] = cfg
    return result


def _tf_to_pose(tf, frame: str, note: str = "") -> dict:
    """Convert a TF transform to a pose dict."""
    q     = tf.transform.rotation
    t     = tf.transform.translation
    yaw   = math.atan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y*q.y + q.z*q.z))
    sinr  = 2*(q.w*q.x + q.y*q.z)
    cosr  = 1 - 2*(q.x*q.x + q.y*q.y)
    roll  = math.atan2(sinr, cosr)
    sinp  = 2*(q.w*q.y - q.z*q.x)
    pitch = math.asin(max(-1.0, min(1.0, sinp)))
    result = {
        "ok": True, "found": True,
        "x": t.x, "y": t.y, "z": t.z,
        "roll": roll, "pitch": pitch, "yaw": yaw,
        "roll_deg":  math.degrees(roll),
        "pitch_deg": math.degrees(pitch),
        "yaw_deg":   math.degrees(yaw),
        "timestamp": time.time(),
        "frame": frame,
    }
    if note:
        result["note"] = note
    return result


def action_robot_position(node: MappingNode) -> dict:
    """Return the robot's current 6-DOF pose.

    Tries frames in order:
      1. new_map → Core  (full SLAM pose, most accurate)
      2. icp_odom → Core (odometry only, no loop closure corrections)
      3. Cached pose from last map callback
    """
    zero_time = rclpy.time.Time(seconds=0, nanoseconds=0,
                                clock_type=rclpy.clock.ClockType.SYSTEM_TIME)

    # 1. Full SLAM pose
    try:
        tf = node.tf_buffer.lookup_transform(MAP_FRAME, ROBOT_FRAME, zero_time)
        return _tf_to_pose(tf, MAP_FRAME)
    except Exception:
        pass

    # 2. Odometry-only pose (icp_odom → Core)
    try:
        tf = node.tf_buffer.lookup_transform("icp_odom", ROBOT_FRAME, zero_time)
        return _tf_to_pose(tf, "icp_odom",
                           "new_map not available yet — odometry frame only, "
                           "no loop closure corrections applied")
    except Exception:
        pass

    # 3. Cached pose from last map callback
    with _state_lock:
        pose = _robot_pose
    if pose:
        return {"ok": True, "found": True, **pose,
                "roll_deg":  math.degrees(pose["roll"]),
                "pitch_deg": math.degrees(pose["pitch"]),
                "yaw_deg":   math.degrees(pose["yaw"]),
                "frame": MAP_FRAME, "cached": True,
                "note": "Cached from last map update — may be slightly stale"}

    return {"ok": False, "found": False,
            "message": "No pose available — is rtabmap running and has the robot moved?"}


def action_add_poi(node: MappingNode, name: str, poi_type: str,
                   distance: float = 0.0) -> dict:
    """
    Add a POI at the robot's current position, optionally offset
    by `distance` metres in the direction the robot is facing.
    """
    pos = action_robot_position(node)
    if not pos.get("found"):
        return {"ok": False, "message": "Robot position unknown — cannot place POI"}

    x   = pos["x"] + distance * math.cos(pos["yaw"])
    y   = pos["y"] + distance * math.sin(pos["yaw"])
    z   = pos["z"]
    yaw = pos["yaw"]

    poi_id = str(uuid.uuid4())
    poi = {
        "id":        poi_id,
        "name":      name or f"POI-{poi_id[:8]}",
        "type":      poi_type or "generic",
        "x": x, "y": y, "z": z,
        "yaw":       yaw,
        "distance_from_robot": distance,
        "timestamp": time.time(),
        "frame":     MAP_FRAME,
    }
    with _poi_lock:
        _pois[poi_id] = poi
        _save_pois(_pois)
    return {"ok": True, "message": "POI added", "poi": poi}


def action_get_pois() -> dict:
    with _poi_lock:
        pois = list(_pois.values())
    return {"ok": True, "count": len(pois), "pois": pois}


def action_delete_poi(poi_id: str) -> dict:
    with _poi_lock:
        if poi_id not in _pois:
            return {"ok": False, "message": f"POI {poi_id!r} not found"}
        del _pois[poi_id]
        _save_pois(_pois)
    return {"ok": True, "message": f"POI {poi_id!r} deleted"}


def action_clear_pois() -> dict:
    """Delete all POIs from memory and from disk."""
    global _pois
    with _poi_lock:
        count = len(_pois)
        _pois = {}
        _save_pois(_pois)
    return {"ok": True, "message": f"Cleared {count} POI(s)"}


def action_launch() -> dict:
    global _launch_process, _launch_log_fh
    with _launch_lock:
        if _launch_process is not None and _launch_process.poll() is None:
            return {"ok": False,
                    "message": f"Already running (pid={_launch_process.pid})"}
        try:
            os.makedirs(os.path.dirname(LAUNCH_LOG_PATH), exist_ok=True)
            # Truncate log on each new launch so status only shows current session
            _launch_log_fh = open(LAUNCH_LOG_PATH, "w", buffering=1)
            _launch_process = subprocess.Popen(
                shlex.split(LAUNCH_CMD),
                stdout=_launch_log_fh,
                stderr=_launch_log_fh,
                start_new_session=True,  # detach — survives mapping_api restart
            )
            _save_pid(_launch_process.pid)
            return {"ok": True,
                    "message": f"Launched (pid={_launch_process.pid})",
                    "log": LAUNCH_LOG_PATH}
        except Exception as e:
            return {"ok": False, "message": str(e)}


def action_stop_launch() -> dict:
    global _launch_process, _launch_log_fh
    stopped_via = []

    # 1. Stop tracked launch process if we have one
    with _launch_lock:
        if _launch_process is not None and _launch_process.poll() is None:
            _launch_process.terminate()
            try:
                _launch_process.wait(timeout=10)
            except subprocess.TimeoutExpired:
                _launch_process.kill()
            stopped_via.append("tracked process")
        if _launch_log_fh:
            try:
                _launch_log_fh.close()
            except Exception:
                pass
            _launch_log_fh = None
    _clear_pid()

    # 2. Always pkill any surviving nodes
    pattern = (
        "rtabmap|icp_odometry|lidar_deskewing|lidar_merger|"
        "livox_ros_driver2|joint_state_publisher|robot_state_publisher|"
        "imu_to_tf|gscam|vectornav_udp|fisheye_rectify"
    )
    try:
        subprocess.run(["pkill", "-9", "-f", pattern], capture_output=True)
        stopped_via.append("pkill")
    except Exception:
        pass

    # 4. Reset internal state
    global _mapping_state, _node_count, _loop_closures
    with _state_lock:
        _mapping_state = "unknown"
        _node_count    = 0
        _loop_closures = 0

    if not stopped_via:
        return {"ok": False, "message": "Nothing was running"}
    return {"ok": True, "message": f"Stopped via: {', '.join(stopped_via)}"}


def action_launch_status(include_log: bool = True) -> dict:
    with _launch_lock:
        running  = _launch_process is not None and _launch_process.poll() is None
        pid      = _launch_process.pid if _launch_process else None
        exitcode = None if running else (
            _launch_process.returncode if _launch_process else None
        )
    result = {
        "running":   running,
        "pid":       pid,
        "exit_code": exitcode,
        "log_path":  LAUNCH_LOG_PATH,
    }
    if include_log:
        result["log"] = get_launch_log()
    return result


# ─────────────────────────────────────────────────────────────────────────────
# HTTP
# ─────────────────────────────────────────────────────────────────────────────

OPENAPI_SPEC = {
    "openapi": "3.0.3",
    "info": {
        "title": "Rove Mapping API", "version": "1.0.0",
        "description": (
            "Controls rtabmap SLAM and the ROS launch stack.\n\n"
            "Service endpoints return 503 with a clear message if rtabmap is not running."
        ),
    },
    "servers": [{"url": "/", "description": "This server"}],
    "tags": [
        {"name": "Status",  "description": "Map and launch status"},
        {"name": "Launch",  "description": "Start and stop the ROS launch stack"},
        {"name": "Mapping", "description": "Control rtabmap SLAM"},
        {"name": "Robot",   "description": "Robot pose and position"},
        {"name": "POIs",    "description": "Points of interest — persisted to disk"},
    ],
    "paths": {
        "/minimap": {"get": {
            "tags": ["Status"],
            "summary": "Robot-centred 2-D occupancy map crop",
            "description": "Cells: -1=unknown  0=free  100=occupied",
            "responses": {"200": {"description": "Snapshot JSON"}},
        }},
        "/mapping/status": {"get": {
            "tags": ["Status"],
            "summary": "State, node count, odom links, loop closures, DB size",
            "responses": {"200": {"description": "Status JSON"}},
        }},
        "/mapping/launch": {"post": {
            "tags": ["Launch"],
            "summary": "Start run.launch.py (detached, log written to LAUNCH_LOG_PATH)",
            "responses": {"200": {"description": "ok"}, "503": {"description": "already running or error"}},
        }},
        "/mapping/launch_status": {"get": {
            "tags": ["Launch"],
            "summary": "Launch process status + optional log output",
            "description": "Add ?log=false to skip log content (useful for fast polling).",
            "parameters": [{"name": "log", "in": "query", "required": False,
                            "schema": {"type": "string", "enum": ["true", "false"],
                                       "default": "true"},
                            "description": "Include log output in response (default true)"}],
            "responses": {"200": {"description": "running, pid, exit_code, log_path, log (if requested)"}},
        }},
        "/mapping/stop": {"post": {
            "tags": ["Launch"],
            "summary": "Stop run.launch.py and kill all ROS nodes",
            "description": "Terminates the tracked launch process, then pkill -9 all known ROS node patterns.",
            "responses": {"200": {"description": "ok"}, "503": {"description": "nothing was running"}},
        }},
        "/mapping/go": {"post": {
            "tags": ["Mapping"],
            "summary": "Resume mapping (no-op if already running)",
            "responses": {"200": {"description": "ok"}, "503": {"description": "error"}},
        }},
        "/mapping/pause": {"post": {
            "tags": ["Mapping"],
            "summary": "Pause scan integration",
            "responses": {"200": {"description": "ok"}, "503": {"description": "error"}},
        }},
        "/mapping/restart": {"post": {
            "tags": ["Mapping"],
            "summary": "Clear map + DB, start fresh (irreversible)",
            "responses": {"200": {"description": "ok"}, "503": {"description": "error"}},
        }},
        "/mapping/new_map": {"post": {
            "tags": ["Mapping"],
            "summary": "Start a new sub-map while keeping the existing pose graph",
            "responses": {"200": {"description": "ok"}, "503": {"description": "error"}},
        }},
        "/mapping/export": {"post": {
            "tags": ["Mapping"],
            "summary": "Export scan PLY + POIs JSON + path JSON + 2D map PGM/YAML (blocks 10-120s)",
            "requestBody": {"required": False, "content": {"application/json": {"schema": {
                "type": "object",
                "properties": {"filename": {"type": "string", "example": "scan.ply"}},
            }}}},
            "responses": {"200": {"description": "ok"}, "503": {"description": "error"}},
        }},
        "/mapping/config": {"post": {
            "tags": ["Mapping"],
            "summary": "Hot-reload rtabmap.yaml without restart",
            "requestBody": {"required": False, "content": {"application/json": {"schema": {
                "type": "object",
                "properties": {"path": {"type": "string"}},
            }}}},
            "responses": {"200": {"description": "ok"}, "503": {"description": "error"}},
        }},
        "/robot/position": {"get": {
            "tags": ["Robot"],
            "summary": "Current robot pose (xyz + roll/pitch/yaw) in map frame",
            "description": "Returns position relative to map origin (0,0,0). Angles in radians and degrees.",
            "responses": {"200": {"description": "Pose JSON"}},
        }},
        "/pois": {"get": {
            "tags": ["POIs"],
            "summary": "Get all POIs (loaded from disk on startup)",
            "responses": {"200": {"description": "List of POIs"}},
        }},
        "/pois/add": {"post": {
            "tags": ["POIs"],
            "summary": "Add a POI at the robot\'s current position",
            "description": "Optionally offset in front of the robot by `distance` metres. Saved to disk immediately.",
            "requestBody": {"required": True, "content": {"application/json": {"schema": {
                "type": "object",
                "required": ["name"],
                "properties": {
                    "name":     {"type": "string", "example": "Charging station"},
                    "type":     {"type": "string", "example": "infrastructure",
                                 "description": "Arbitrary category string"},
                    "distance": {"type": "number", "example": 0.5,
                                 "description": "Metres in front of robot (default 0)"},
                },
            }}}},
            "responses": {"200": {"description": "Created POI"}, "503": {"description": "error"}},
        }},
        "/pois/delete": {"post": {
            "tags": ["POIs"],
            "summary": "Delete a POI by ID",
            "requestBody": {"required": True, "content": {"application/json": {"schema": {
                "type": "object",
                "required": ["id"],
                "properties": {"id": {"type": "string"}},
            }}}},
            "responses": {"200": {"description": "ok"}, "503": {"description": "not found"}},
        }},
        "/pois/clear": {"post": {
            "tags": ["POIs"],
            "summary": "Delete all POIs from memory and disk (irreversible)",
            "responses": {"200": {"description": "ok"}},
        }},
    },
}
SWAGGER_HTML = b"""<!DOCTYPE html>
<html lang="en">
<head>
  <meta charset="UTF-8"/>
  <title>Rove Mapping API</title>
  <meta name="viewport" content="width=device-width, initial-scale=1"/>
  <link rel="stylesheet" href="https://unpkg.com/swagger-ui-dist@5/swagger-ui.css"/>
</head>
<body>
  <div id="swagger-ui"></div>
  <script src="https://unpkg.com/swagger-ui-dist@5/swagger-ui-bundle.js"></script>
  <script>
    SwaggerUIBundle({
      url: "/openapi.json",
      dom_id: "#swagger-ui",
      presets: [SwaggerUIBundle.presets.apis, SwaggerUIBundle.SwaggerUIStandalonePreset],
      layout: "BaseLayout",
      deepLinking: true,
      tryItOutEnabled: true,
      displayRequestDuration: true,
    });
  </script>
</body>
</html>"""


def make_handler(node: MappingNode):

    class Handler(BaseHTTPRequestHandler):

        def log_message(self, fmt, *args):
            print(f"[http] {self.address_string()} {fmt % args}", flush=True)

        def do_OPTIONS(self):
            self.send_response(204)
            self._cors()
            self.end_headers()

        def _cors(self):
            self.send_header("Access-Control-Allow-Origin",  "*")
            self.send_header("Access-Control-Allow-Methods", "GET, POST, OPTIONS")
            self.send_header("Access-Control-Allow-Headers", "Content-Type")

        def _send(self, code: int, ctype: str, body: bytes):
            self.send_response(code)
            self.send_header("Content-Type",   ctype)
            self.send_header("Content-Length", str(len(body)))
            self._cors()
            self.end_headers()
            self.wfile.write(body)

        def _json(self, code: int, obj: dict):
            self._send(code, "application/json",
                       json.dumps(obj, indent=2).encode())

        def _body(self) -> dict:
            n = int(self.headers.get("Content-Length") or 0)
            if not n:
                return {}
            try:
                return json.loads(self.rfile.read(n))
            except Exception:
                return {}

        def do_GET(self):
            p = self.path.split("?")[0].rstrip("/") or "/"

            if p in ("/", "/docs"):
                self._send(200, "text/html; charset=utf-8", SWAGGER_HTML)
            elif p == "/openapi.json":
                self._send(200, "application/json",
                           json.dumps(OPENAPI_SPEC, indent=2).encode())
            elif p == "/minimap":
                with _state_lock:
                    snap = _snapshot
                self._json(200, snap or {
                    "error": "No map yet",
                    "hint":  "Is run.launch.py running and publishing /grid_prob_map?",
                })
            elif p == "/mapping/status":
                self._json(200, get_status(node))
            elif p == "/robot/position":
                self._json(200, action_robot_position(node))
            elif p == "/pois":
                self._json(200, action_get_pois())
            elif p == "/mapping/launch_status":
                # ?log=false to skip log content (faster for polling)
                qs = self.path.split("?", 1)[1] if "?" in self.path else ""
                include_log = "log=false" not in qs.lower()
                self._json(200, action_launch_status(include_log=include_log))
            else:
                self._json(404, {"error": f"Not found: {p}"})

        def do_POST(self):
            p    = self.path.split("?")[0].rstrip("/")
            body = self._body()

            routes = {
                "/mapping/go":      lambda: action_start(node),
                "/mapping/pause":   lambda: action_pause(node),
                "/mapping/restart": lambda: action_reset(node),
                "/mapping/new_map": lambda: action_new_map(node),
                "/mapping/export":  lambda: action_export(node, body.get("filename")),
                "/mapping/config":  lambda: action_config(node, body.get("path")),
                "/mapping/launch":  lambda: action_launch(),
                "/mapping/stop":    lambda: action_stop_launch(),
                "/pois/add":        lambda: action_add_poi(
                    node,
                    body.get("name", ""),
                    body.get("type", "generic"),
                    float(body.get("distance", 0.0))
                ),
                "/pois/delete":     lambda: action_delete_poi(body.get("id", "")),
                "/pois/clear":      lambda: action_clear_pois(),
            }

            if p not in routes:
                self._json(404, {"error": f"Not found: {p}"})
                return

            result = routes[p]()
            self._json(200 if result.get("ok") else 503, result)

    return Handler


# ─────────────────────────────────────────────────────────────────────────────
# Entry point
# ─────────────────────────────────────────────────────────────────────────────

def main():
    rclpy.init()
    node     = MappingNode()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)

    ros_thread = threading.Thread(
        target=executor.spin, daemon=True, name="ros-executor"
    )
    ros_thread.start()

    # Re-attach to any launch process that survived a previous API restart
    _restore_launch_process()

    server = HTTPServer((HTTP_HOST, HTTP_PORT), make_handler(node))

    def _on_signal(sig, _frame):
        print(f"\n[mapping_api] signal {sig} — stopping", flush=True)
        threading.Thread(target=server.shutdown, daemon=True).start()

    signal.signal(signal.SIGINT,  _on_signal)
    signal.signal(signal.SIGTERM, _on_signal)

    print(f"[mapping_api] listening on :{HTTP_PORT}", flush=True)
    print(f"[mapping_api] swagger → http://<robot-ip>:{HTTP_PORT}/", flush=True)
    print(f"[mapping_api] test   → curl http://<robot-ip>:{HTTP_PORT}/mapping/status", flush=True)

    try:
        server.serve_forever()
    finally:
        executor.shutdown(wait=False)
        rclpy.shutdown()
        print("[mapping_api] done", flush=True)


if __name__ == "__main__":
    main()