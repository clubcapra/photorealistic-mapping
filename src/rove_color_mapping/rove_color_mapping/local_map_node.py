#!/usr/bin/env python3
"""
local_map_server_node.py

Subscribes to rtabmap's /grid_prob_map (nav_msgs/OccupancyGrid) and
the robot's TF pose, then:
  - Crops a LOCAL_SIZE x LOCAL_SIZE cell window centred on the robot
  - Serves the latest snapshot as JSON + PNG on HTTP port MAP_PORT
"""

import threading
import json
import io
import base64
import struct
import time
import math

import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from tf2_ros import Buffer, TransformListener, LookupException, ConnectivityException, ExtrapolationException
import numpy as np

from http.server import HTTPServer, BaseHTTPRequestHandler

# ─── tunables (can be overridden via ROS parameters) ────────────────────────
DEFAULT_LOCAL_SIZE   = 200   # output grid cells (N x N)
DEFAULT_HTTP_PORT    = 8765
DEFAULT_MAP_TOPIC    = "/grid_prob_map"
DEFAULT_ROBOT_FRAME  = "base_link"
DEFAULT_MAP_FRAME    = "new_map"
# ─────────────────────────────────────────────────────────────────────────────


def encode_png_gray(array_2d: np.ndarray) -> bytes:
    """Pure-stdlib minimal grayscale PNG encoder (no Pillow/cv2 needed)."""
    import zlib
    h, w = array_2d.shape
    raw = array_2d.astype(np.uint8)

    def make_chunk(tag: bytes, data: bytes) -> bytes:
        length = struct.pack(">I", len(data))
        crc = struct.pack(">I", zlib.crc32(tag + data) & 0xFFFFFFFF)
        return length + tag + data + crc

    ihdr_data = struct.pack(">IIBBBBB", w, h, 8, 0, 0, 0, 0)
    scanlines = b"".join(b"\x00" + raw[y].tobytes() for y in range(h))
    idat_data = zlib.compress(scanlines, 9)

    return (
        b"\x89PNG\r\n\x1a\n"
        + make_chunk(b"IHDR", ihdr_data)
        + make_chunk(b"IDAT", idat_data)
        + make_chunk(b"IEND", b"")
    )


class LocalMapNode(Node):
    def __init__(self):
        super().__init__("local_map_server")

        # Parameters
        self.declare_parameter("local_size",  DEFAULT_LOCAL_SIZE)
        self.declare_parameter("http_port",   DEFAULT_HTTP_PORT)
        self.declare_parameter("map_topic",   DEFAULT_MAP_TOPIC)
        self.declare_parameter("robot_frame", DEFAULT_ROBOT_FRAME)
        self.declare_parameter("map_frame",   DEFAULT_MAP_FRAME)

        self.local_size   = self.get_parameter("local_size").value
        self.http_port    = self.get_parameter("http_port").value
        self.map_topic    = self.get_parameter("map_topic").value
        self.robot_frame  = self.get_parameter("robot_frame").value
        self.map_frame    = self.get_parameter("map_frame").value

        # TF
        self.tf_buffer   = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # State (protected by lock)
        self._lock      = threading.Lock()
        self._snapshot  = None   # dict ready to JSON-serialise
        self._png_bytes = None   # raw PNG bytes for /map.png

        # Subscriber
        self.sub = self.create_subscription(
            OccupancyGrid, self.map_topic, self._map_cb, 1
        )

        # HTTP server in background thread
        node_ref = self
        class Handler(BaseHTTPRequestHandler):
            def log_message(self, fmt, *args):   # silence access log
                pass
            def do_GET(self):
                if self.path == "/" or self.path == "/index.html":
                    self._serve_ui()
                elif self.path == "/snapshot":
                    self._serve_json()
                elif self.path == "/map.png":
                    self._serve_png()
                else:
                    self.send_response(404); self.end_headers()

            def _serve_json(self):
                with node_ref._lock:
                    snap = node_ref._snapshot
                if snap is None:
                    body = json.dumps({"error": "No map received yet"}).encode()
                else:
                    body = json.dumps(snap).encode()
                self.send_response(200)
                self.send_header("Content-Type", "application/json")
                self.send_header("Access-Control-Allow-Origin", "*")
                self.send_header("Content-Length", str(len(body)))
                self.end_headers()
                self.wfile.write(body)

            def _serve_png(self):
                with node_ref._lock:
                    png = node_ref._png_bytes
                if png is None:
                    self.send_response(503); self.end_headers(); return
                self.send_response(200)
                self.send_header("Content-Type", "image/png")
                self.send_header("Access-Control-Allow-Origin", "*")
                self.send_header("Content-Length", str(len(png)))
                self.end_headers()
                self.wfile.write(png)

            def _serve_ui(self):
                html = node_ref._get_ui_html()
                body = html.encode()
                self.send_response(200)
                self.send_header("Content-Type", "text/html; charset=utf-8")
                self.send_header("Content-Length", str(len(body)))
                self.end_headers()
                self.wfile.write(body)

        server = HTTPServer(("0.0.0.0", self.http_port), Handler)
        t = threading.Thread(target=server.serve_forever, daemon=True)
        t.start()
        self.get_logger().info(
            f"HTTP server started on port {self.http_port}  "
            f"(topic: {self.map_topic})"
        )

    # ── map callback ──────────────────────────────────────────────────────────
    def _map_cb(self, msg: OccupancyGrid):
        # Try to get robot position in map frame
        rx_m, ry_m, robot_yaw = 0.0, 0.0, 0.0
        robot_found = False
        try:
            t = self.tf_buffer.lookup_transform(
                    self.map_frame, self.robot_frame,
                    rclpy.time.Time(seconds=0, nanoseconds=0,
                                    clock_type=rclpy.clock.ClockType.SYSTEM_TIME),
                )
            rx_m = t.transform.translation.x
            ry_m = t.transform.translation.y
            q = t.transform.rotation
            robot_yaw = math.atan2(
                2.0 * (q.w * q.z + q.x * q.y),
                1.0 - 2.0 * (q.y * q.y + q.z * q.z)
            )
            robot_found = True
            self.get_logger().info(f"TF ok: x={rx_m:.2f} y={ry_m:.2f}", throttle_duration_sec=1)
        except Exception as e:
            self.get_logger().warn(f"TF failed: {type(e).__name__}: {e}", throttle_duration_sec=1)

        # Map metadata
        res   = msg.info.resolution          # m/cell
        ox    = msg.info.origin.position.x   # map origin x
        oy    = msg.info.origin.position.y
        mw    = msg.info.width
        mh    = msg.info.height

        # Robot cell position in full map
        cx = int((rx_m - ox) / res)
        cy = int((ry_m - oy) / res)

        half = self.local_size // 2

        # Full map as numpy array (row-major, y=0 is bottom in ROS)
        full = np.array(msg.data, dtype=np.int8).reshape((mh, mw))

        # Build output canvas (unknown = 205 grey)
        canvas = np.full((self.local_size, self.local_size), 128, dtype=np.uint8)
        canvas_raw = np.full((self.local_size, self.local_size), -1, dtype=np.int8)
        # Source rectangle in full map
        src_r0 = cy - half;  src_r1 = cy + half
        src_c0 = cx - half;  src_c1 = cx + half

        # Clamp to map bounds
        dst_r0 = max(0, -src_r0);   dst_r1 = self.local_size - max(0, src_r1 - mh)
        dst_c0 = max(0, -src_c0);   dst_c1 = self.local_size - max(0, src_c1 - mw)
        src_r0 = max(0, src_r0);    src_r1 = min(mh, src_r1)
        src_c0 = max(0, src_c0);    src_c1 = min(mw, src_c1)

        if src_r1 > src_r0 and src_c1 > src_c0:
            patch = full[src_r0:src_r1, src_c0:src_c1]
            # Convert OccupancyGrid values → grayscale
            # -1 (unknown) → 128, 0 (free) → 255, 100 (occ) → 0
            vis = np.where(patch == -1, 128,
                  np.where(patch == 0,  255,
                  np.clip(255 - patch * 255 // 100, 0, 255))).astype(np.uint8)
            canvas[dst_r0:dst_r1, dst_c0:dst_c1] = vis
            canvas_raw[dst_r0:dst_r1, dst_c0:dst_c1] = patch

        # Flip vertically so y-up (ROS) → y-down (image)
        canvas = np.flipud(canvas)

        # Mark robot centre with a cross (only if inside crop)
        def mark_cross(img, r, c, size=5):
            h, w = img.shape
            for i in range(-size, size + 1):
                if 0 <= r + i < h: img[r + i, c] = 80
                if 0 <= c + i < w: img[r, c + i] = 80

        mark_cross(canvas, self.local_size - half - 1, half)

        png = encode_png_gray(canvas)

        # JSON metadata
        snap = {
            "timestamp":   time.time(),
            "local_size":  self.local_size,
            "resolution":  res,
            "robot_x":     rx_m,
            "robot_y":     ry_m,
            "robot_yaw":   robot_yaw,
            "robot_found": robot_found,
            "data":        canvas_raw.tolist(),   # flat list of int8, -1..100
        }

        with self._lock:
            self._snapshot  = snap
            self._png_bytes = png

    # ── embedded debug UI ─────────────────────────────────────────────────────
    def _get_ui_html(self) -> str:
        return f"""<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>Local Prob-Map Debug</title>
<style>
  :root {{
    --bg: #0d1117; --panel: #161b22; --border: #30363d;
    --accent: #58a6ff; --ok: #3fb950; --warn: #d29922;
    --text: #e6edf3; --muted: #8b949e;
    --font: 'JetBrains Mono', 'Fira Mono', monospace;
  }}
  * {{ box-sizing: border-box; margin: 0; padding: 0; }}
  body {{ background: var(--bg); color: var(--text); font-family: var(--font); font-size: 13px; }}
  header {{ padding: 12px 20px; border-bottom: 1px solid var(--border);
            display: flex; align-items: center; gap: 12px; }}
  header h1 {{ font-size: 15px; color: var(--accent); letter-spacing: .04em; }}
  .dot {{ width: 10px; height: 10px; border-radius: 50%; background: var(--muted); flex-shrink: 0; }}
  .dot.live {{ background: var(--ok); box-shadow: 0 0 6px var(--ok); animation: pulse 1.5s infinite; }}
  @keyframes pulse {{ 0%,100% {{ opacity:1 }} 50% {{ opacity:.4 }} }}
  .layout {{ display: flex; gap: 0; height: calc(100vh - 45px); }}
  .map-area {{ flex: 1; display: flex; align-items: center; justify-content: center;
               background: #080b10; }}
  canvas {{ image-rendering: pixelated; max-width: min(70vw,70vh); max-height: min(70vw,70vh);
            border: 1px solid var(--border); }}
  .sidebar {{ width: 270px; background: var(--panel); border-left: 1px solid var(--border);
              overflow-y: auto; padding: 16px; display: flex; flex-direction: column; gap: 14px; }}
  .card {{ background: var(--bg); border: 1px solid var(--border); border-radius: 6px; padding: 12px; }}
  .card h2 {{ font-size: 11px; color: var(--muted); text-transform: uppercase;
              letter-spacing: .08em; margin-bottom: 8px; }}
  .kv {{ display: flex; justify-content: space-between; padding: 3px 0; border-bottom: 1px solid var(--border); }}
  .kv:last-child {{ border-bottom: none; }}
  .kv .k {{ color: var(--muted); }}
  .kv .v {{ color: var(--text); font-weight: 600; }}
  .legend {{ display: flex; flex-direction: column; gap: 6px; }}
  .swatch {{ display: flex; align-items: center; gap: 8px; }}
  .swatch span {{ width: 16px; height: 16px; border-radius: 3px; flex-shrink: 0; }}
  label {{ display: flex; justify-content: space-between; align-items: center; }}
  input[type=range] {{ width: 120px; accent-color: var(--accent); }}
  .fps {{ font-size: 11px; color: var(--muted); }}
  button {{ background: var(--accent); color: #000; border: none; border-radius: 4px;
            padding: 6px 14px; cursor: pointer; font-family: var(--font); font-size: 12px;
            font-weight: 700; width: 100%; }}
  button:hover {{ filter: brightness(1.15); }}
  #err {{ color: #f85149; font-size: 12px; display: none; }}
</style>
</head>
<body>
<header>
  <div class="dot" id="dot"></div>
  <h1>rtabmap · local_map_server</h1>
  <span class="fps" id="fps"></span>
  <span style="flex:1"></span>
  <span class="fps" id="ts"></span>
</header>
<div class="layout">
  <div class="map-area">
    <canvas id="cv"></canvas>
  </div>
  <div class="sidebar">
    <div class="card">
      <h2>Robot Pose</h2>
      <div class="kv"><span class="k">x</span><span class="v" id="rx">—</span></div>
      <div class="kv"><span class="k">y</span><span class="v" id="ry">—</span></div>
      <div class="kv"><span class="k">yaw</span><span class="v" id="ryaw">—</span></div>
      <div class="kv"><span class="k">TF</span><span class="v" id="rtf">—</span></div>
    </div>
    <div class="card">
      <h2>Map Info</h2>
      <div class="kv"><span class="k">crop</span><span class="v" id="sz">—</span></div>
      <div class="kv"><span class="k">res</span><span class="v" id="res">—</span></div>
      <div class="kv"><span class="k">full map</span><span class="v" id="full">—</span></div>
      <div class="kv"><span class="k">robot cell</span><span class="v" id="rcell">—</span></div>
    </div>
    <div class="card">
      <h2>Legend</h2>
      <div class="legend">
        <div class="swatch"><span style="background:#fff"></span>Free</div>
        <div class="swatch"><span style="background:#505050"></span>Unknown</div>
        <div class="swatch"><span style="background:#000"></span>Occupied</div>
        <div class="swatch"><span style="background:#505050;outline:2px solid #58a6ff"></span>Robot ✛</div>
      </div>
    </div>
    <div class="card">
      <h2>Refresh</h2>
      <label>Rate <input type="range" id="rate" min="1" max="30" value="5"> <span id="ratelbl">5 Hz</span></label>
    </div>
    <button onclick="saveSnapshot()">💾 Save PNG</button>
    <div id="err"></div>
  </div>
</div>
<script>
const cv = document.getElementById('cv');
const ctx = cv.getContext('2d');
let hz = 5, last = 0, frames = 0, fpsAccum = 0;

document.getElementById('rate').addEventListener('input', e => {{
  hz = +e.target.value;
  document.getElementById('ratelbl').textContent = hz + ' Hz';
}});

async function fetchSnap() {{
  try {{
    const r = await fetch('/snapshot?' + Date.now());
    if (!r.ok) throw new Error(r.status);
    const d = await r.json();
    if (d.error) {{ showErr(d.error); return; }}
    hideErr();

    document.getElementById('dot').className = 'dot live';
    document.getElementById('ts').textContent =
      new Date(d.timestamp * 1000).toLocaleTimeString();

    // Pose
    document.getElementById('rx').textContent   = d.robot_x.toFixed(3) + ' m';
    document.getElementById('ry').textContent   = d.robot_y.toFixed(3) + ' m';
    document.getElementById('ryaw').textContent = (d.robot_yaw * 180 / Math.PI).toFixed(1) + '°';
    document.getElementById('rtf').textContent  = d.robot_found ? '✓ ok' : '✗ missing';
    document.getElementById('rtf').style.color  = d.robot_found ? 'var(--ok)' : 'var(--warn)';

    // Map info
    document.getElementById('sz').textContent   = d.local_size + ' × ' + d.local_size;
    document.getElementById('res').textContent  = (d.resolution * 100).toFixed(1) + ' cm/cell';
    document.getElementById('full').textContent = d.map_w + ' × ' + d.map_h;
    document.getElementById('rcell').textContent= d.crop_cx + ', ' + d.crop_cy;

    // Draw PNG
    const img = new Image();
    img.onload = () => {{
      cv.width = d.local_size; cv.height = d.local_size;
      ctx.drawImage(img, 0, 0);
      // Compass rose
      drawCompass(ctx, d.local_size, d.robot_yaw);
    }};
    img.src = 'data:image/png;base64,' + d.png_b64;

    // FPS
    frames++;
    const now = performance.now();
    if (now - fpsAccum > 1000) {{
      document.getElementById('fps').textContent = frames + ' fps';
      frames = 0; fpsAccum = now;
    }}
  }} catch(e) {{ showErr('Fetch error: ' + e.message); }}
}}

function drawCompass(ctx, size, yaw) {{
  const cx = size - 18, cy = 18, r = 12;
  ctx.save();
  ctx.globalAlpha = 0.85;
  ctx.beginPath(); ctx.arc(cx, cy, r, 0, Math.PI*2);
  ctx.fillStyle = '#161b22'; ctx.fill();
  ctx.strokeStyle = '#30363d'; ctx.lineWidth = 1; ctx.stroke();
  // North arrow (red) — note: map y-axis is flipped
  const ang = -yaw - Math.PI / 2;
  ctx.translate(cx, cy); ctx.rotate(ang);
  ctx.beginPath();
  ctx.moveTo(0, -r + 3); ctx.lineTo(3, 2); ctx.lineTo(-3, 2); ctx.closePath();
  ctx.fillStyle = '#f85149'; ctx.fill();
  ctx.restore();
}}

function showErr(msg) {{
  const el = document.getElementById('err');
  el.style.display = 'block'; el.textContent = msg;
  document.getElementById('dot').className = 'dot';
}}
function hideErr() {{ document.getElementById('err').style.display = 'none'; }}

async function saveSnapshot() {{
  const r = await fetch('/map.png');
  const blob = await r.blob();
  const a = document.createElement('a');
  a.href = URL.createObjectURL(blob);
  a.download = 'local_map_' + Date.now() + '.png';
  a.click();
}}

function schedule() {{
  fetchSnap().finally(() => setTimeout(schedule, Math.round(1000 / hz)));
}}
schedule();
</script>
</body>
</html>
"""


def main(args=None):
    rclpy.init(args=args)
    node = LocalMapNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()