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
import subprocess
import datetime
import threading
import time
import yaml
import signal
import sys

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

# ── QoS profiles matching rtabmap's publishers ────────────────────────────────
# rtabmap publishes maps and info with transient_local so late-joining
# subscribers receive the last message immediately on connect.
# A volatile subscriber will never receive anything from a transient_local
# publisher — they must match.
QOS_TRANSIENT = QoSProfile(
    reliability=QoSReliabilityPolicy.RELIABLE,
    durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=1,
)
# Sensor data (imu, scan) uses best-effort
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
ROBOT_FRAME = "base_link"
MAP_FRAME   = "new_map"
LOCAL_SIZE  = 200

EXPORT_DIR         = "/mnt/ssd/maps"
RTABMAP_DB         = os.path.expanduser("~/.ros/rtabmap.db")
RTABMAP_EXPORT_BIN = "/opt/ros/humble/bin/rtabmap-export"
CONFIG_PATH        = os.path.join(
    os.path.dirname(os.path.abspath(__file__)), "config", "rtabmap.yaml"
)

SVC_PAUSE     = "/rtabmap/pause"
SVC_RESUME    = "/rtabmap/resume"
SVC_RESET     = "/rtabmap/reset"
SVC_TRIGGER   = "/rtabmap/trigger_new_map"
SVC_SET_PARAM = "/rtabmap/set_parameters"

# ── Shared state ──────────────────────────────────────────────────────────────
_state_lock    = threading.Lock()
_snapshot      = None
_mapping_state = "unknown"
_node_count    = 0
_loop_closures = 0


def get_status() -> dict:
    db_mb = 0.0
    if os.path.isfile(RTABMAP_DB):
        try:
            db_mb = round(os.path.getsize(RTABMAP_DB) / 1e6, 2)
        except OSError:
            pass
    with _state_lock:
        return {
            "state":         _mapping_state,
            "node_count":    _node_count,
            "loop_closures": _loop_closures,
            "db_size_mb":    db_mb,
            "timestamp":     time.time(),
        }


# ─────────────────────────────────────────────────────────────────────────────
# ROS node
# ─────────────────────────────────────────────────────────────────────────────

class MappingNode(Node):
    def __init__(self):
        super().__init__("mapping_api")
        cb = ReentrantCallbackGroup()

        self.tf_buffer   = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # rtabmap publishes /grid_prob_map and /rtabmap/info with
        # transient_local durability. A subscriber with the default volatile
        # profile receives nothing — the QoS must match.
        self.create_subscription(
            OccupancyGrid, MAP_TOPIC, self._on_map,
            QOS_TRANSIENT, callback_group=cb
        )

        try:
            from rtabmap_msgs.msg import Info
            # /rtabmap/info uses volatile durability unlike /grid_prob_map
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
        with _state_lock:
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
        """
        Call an Empty ROS service from any thread safely.

        Uses add_done_callback + threading.Event:
        - HTTP handler thread blocks on event.wait()
        - Executor thread runs the callback when the future resolves
        - No busy-polling, no deadlock
        """
        if not client.wait_for_service(timeout_sec=timeout):
            return False, f"{svc_name} not available — is rtabmap running?"

        event  = threading.Event()
        result = [None, None]  # [ok, error_msg]

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
    # Only skip if we are certain it is running (not unknown)
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
    # Resume first if paused — reset may not fully clear on a paused node
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
    # Pause if running or unknown — always safer to pause before export
    was_running = state in ("running", "unknown")
    if was_running:
        ok, _ = node.call_empty(node.cli_pause, SVC_PAUSE)
        if ok:
            with _state_lock: _mapping_state = "paused"

    cmd = [RTABMAP_EXPORT_BIN,
           "--scan",                     # export LiDAR scan cloud (not depth/stereo)
           "--scan_voxel", "0.01",       # voxel downsample
           "--ply",                      # output format
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


# ─────────────────────────────────────────────────────────────────────────────
# HTTP
# ─────────────────────────────────────────────────────────────────────────────

# "servers": [{"url": "/"}]  → Swagger Try-it-out sends requests to the same
# host:port the browser loaded the page from — works from any machine.
OPENAPI_SPEC = {
    "openapi": "3.0.3",
    "info": {
        "title": "Rove Mapping API", "version": "1.0.0",
        "description": (
            "Controls rtabmap SLAM. Can start before or after run.launch.py.\n\n"
            "Service endpoints return 503 with a clear message if rtabmap is not running."
        ),
    },
    "servers": [{"url": "/", "description": "This server"}],
    "paths": {
        "/minimap": {"get": {
            "summary": "Robot-centred 2-D occupancy map crop",
            "description": "Cells: -1=unknown  0=free  100=occupied",
            "responses": {"200": {"description": "Snapshot JSON"}},
        }},
        "/mapping/status": {"get": {
            "summary": "State, node count, loop closures, DB size",
            "responses": {"200": {"description": "Status JSON"}},
        }},
        "/mapping/go": {"post": {
            "summary": "Resume mapping (no-op if already running)",
            "responses": {"200": {"description": "ok"}, "503": {"description": "error"}},
        }},
        "/mapping/pause": {"post": {
            "summary": "Pause scan integration",
            "responses": {"200": {"description": "ok"}, "503": {"description": "error"}},
        }},
        "/mapping/restart": {"post": {
            "summary": "Clear map + DB, start fresh (irreversible)",
            "responses": {"200": {"description": "ok"}, "503": {"description": "error"}},
        }},
        "/mapping/new_map": {"post": {
            "summary": "Start a new sub-map while keeping the existing pose graph (multi-floor / multi-session use only)",
            "responses": {"200": {"description": "ok"}, "503": {"description": "error"}},
        }},
        "/mapping/export": {"post": {
            "summary": "Export coloured PLY → /mnt/ssd/maps  (blocks 10-120s)",
            "requestBody": {"required": False, "content": {"application/json": {"schema": {
                "type": "object",
                "properties": {"filename": {"type": "string", "example": "scan.ply"}},
            }}}},
            "responses": {"200": {"description": "ok"}, "503": {"description": "error"}},
        }},
        "/mapping/config": {"post": {
            "summary": "Hot-reload rtabmap.yaml without restart",
            "requestBody": {"required": False, "content": {"application/json": {"schema": {
                "type": "object",
                "properties": {"path": {"type": "string",
                                        "description": "Absolute path (optional)"}},
            }}}},
            "responses": {"200": {"description": "ok"}, "503": {"description": "error"}},
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

            if p == "/":
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
                self._json(200, get_status())
            else:
                self._json(404, {"error": f"Not found: {p}"})

        def do_POST(self):
            p    = self.path.split("?")[0].rstrip("/")
            body = self._body()

            routes = {
                "/mapping/go":   lambda: action_start(node),
                "/mapping/pause":   lambda: action_pause(node),
                "/mapping/restart":   lambda: action_reset(node),
                "/mapping/new_map": lambda: action_new_map(node),
                "/mapping/export":  lambda: action_export(node, body.get("filename")),
                "/mapping/config":  lambda: action_config(node, body.get("path")),
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

    # ROS in background daemon thread
    ros_thread = threading.Thread(
        target=executor.spin, daemon=True, name="ros-executor"
    )
    ros_thread.start()

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