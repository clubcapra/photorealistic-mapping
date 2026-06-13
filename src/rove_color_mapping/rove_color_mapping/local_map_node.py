#!/usr/bin/env python3
import threading
import json
import math
import time

import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from tf2_ros import Buffer, TransformListener, LookupException, ConnectivityException, ExtrapolationException
from scipy.ndimage import rotate as ndimage_rotate
import numpy as np

from http.server import HTTPServer, BaseHTTPRequestHandler

DEFAULT_LOCAL_SIZE  = 200
DEFAULT_HTTP_PORT   = 8765
DEFAULT_MAP_TOPIC   = "/grid_prob_map"
DEFAULT_ROBOT_FRAME = "base_link"
DEFAULT_MAP_FRAME   = "new_map"


class LocalMapNode(Node):
    def __init__(self):
        super().__init__("local_map_server")

        self.declare_parameter("local_size",  DEFAULT_LOCAL_SIZE)
        self.declare_parameter("http_port",   DEFAULT_HTTP_PORT)
        self.declare_parameter("map_topic",   DEFAULT_MAP_TOPIC)
        self.declare_parameter("robot_frame", DEFAULT_ROBOT_FRAME)
        self.declare_parameter("map_frame",   DEFAULT_MAP_FRAME)

        self.local_size  = self.get_parameter("local_size").value
        self.http_port   = self.get_parameter("http_port").value
        self.map_topic   = self.get_parameter("map_topic").value
        self.robot_frame = self.get_parameter("robot_frame").value
        self.map_frame   = self.get_parameter("map_frame").value

        self.tf_buffer   = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self._lock     = threading.Lock()
        self._snapshot = None

        self.sub = self.create_subscription(
            OccupancyGrid, self.map_topic, self._map_cb, 1
        )

        node_ref = self
        class Handler(BaseHTTPRequestHandler):
            def log_message(self, fmt, *args):
                pass
            def do_GET(self):
                if self.path.startswith("/snapshot"):
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
                else:
                    self.send_response(404)
                    self.end_headers()

        server = HTTPServer(("0.0.0.0", self.http_port), Handler)
        t = threading.Thread(target=server.serve_forever, daemon=True)
        t.start()
        self.get_logger().info(f"local_map_server on port {self.http_port} ({self.map_topic})")

    def _map_cb(self, msg: OccupancyGrid):
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

        res = msg.info.resolution
        ox  = msg.info.origin.position.x
        oy  = msg.info.origin.position.y
        mw  = msg.info.width
        mh  = msg.info.height

        cx = int((rx_m - ox) / res)
        cy = int((ry_m - oy) / res)
        half = self.local_size // 2

        full = np.array(msg.data, dtype=np.int8).reshape((mh, mw))
        canvas_raw = np.full((self.local_size, self.local_size), -1, dtype=np.int8)

        src_r0 = cy - half;  src_r1 = cy + half
        src_c0 = cx - half;  src_c1 = cx + half

        dst_r0 = max(0, -src_r0);  dst_r1 = self.local_size - max(0, src_r1 - mh)
        dst_c0 = max(0, -src_c0);  dst_c1 = self.local_size - max(0, src_c1 - mw)
        src_r0 = max(0, src_r0);   src_r1 = min(mh, src_r1)
        src_c0 = max(0, src_c0);   src_c1 = min(mw, src_c1)

        if src_r1 > src_r0 and src_c1 > src_c0:
            canvas_raw[dst_r0:dst_r1, dst_c0:dst_c1] = full[src_r0:src_r1, src_c0:src_c1]

        # Flip y (ROS y-up → image y-down) then rotate to face robot forward
        canvas_raw = np.flipud(canvas_raw)
        angle_deg  = math.degrees(robot_yaw) + 90.0
        canvas_raw = ndimage_rotate(canvas_raw, angle_deg, reshape=False, order=0, cval=-1)

        snap = {
            "timestamp":   time.time(),
            "local_size":  self.local_size,
            "resolution":  res,
            "robot_x":     rx_m,
            "robot_y":     ry_m,
            "robot_yaw":   robot_yaw,
            "robot_found": robot_found,
            "data":        canvas_raw.tolist(),
        }

        with self._lock:
            self._snapshot = snap


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