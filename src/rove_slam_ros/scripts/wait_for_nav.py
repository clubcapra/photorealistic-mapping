#!/usr/bin/env python3
"""Wait for the nav2 lifecycle nodes to be active, retrying any that fall
into the configure/activate race. Used by the headless nav test.

Usage:
    wait_for_nav.py [--timeout 60] [--nodes node1 node2 ...]

Returns exit 0 once every named node reports state "active". Returns 1 on
timeout. Idempotent — calling change_state on an already-active node is a
no-op.
"""
from __future__ import annotations
import argparse
import sys
import time

import rclpy
from rclpy.node import Node
from lifecycle_msgs.msg import Transition
from lifecycle_msgs.srv import ChangeState, GetState

DEFAULT_NODES = [
    "controller_server",
    "planner_server",
    "behavior_server",
    "bt_navigator",
    "waypoint_follower",
    "velocity_smoother",
    "smoother_server",
]


class Waiter(Node):
    def __init__(self):
        super().__init__("rove_slam_wait_for_nav")

    def get_state(self, node_name: str) -> str:
        client = self.create_client(GetState, f"/{node_name}/get_state")
        if not client.wait_for_service(timeout_sec=0.5):
            return "absent"
        future = client.call_async(GetState.Request())
        rclpy.spin_until_future_complete(self, future, timeout_sec=3.0)
        try:
            return future.result().current_state.label
        except Exception:
            return "unknown"

    def change(self, node_name: str, transition_id: int) -> bool:
        client = self.create_client(ChangeState, f"/{node_name}/change_state")
        if not client.wait_for_service(timeout_sec=0.5):
            return False
        req = ChangeState.Request()
        req.transition.id = transition_id
        future = client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=20.0)
        try:
            return bool(future.result().success)
        except Exception:
            return False


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--timeout", type=float, default=60.0)
    ap.add_argument("--nodes", nargs="+", default=DEFAULT_NODES)
    args = ap.parse_args()

    rclpy.init()
    w = Waiter()
    deadline = time.monotonic() + args.timeout

    while time.monotonic() < deadline:
        states = {n: w.get_state(n) for n in args.nodes}
        bad = [n for n, s in states.items() if s != "active"]
        if not bad:
            print("\nALL ACTIVE:", states)
            rclpy.shutdown()
            return 0

        # Try to advance each non-active node along the lifecycle.
        for name in bad:
            s = states[name]
            if s == "unconfigured":
                ok = w.change(name, Transition.TRANSITION_CONFIGURE)
                w.get_logger().info(f"  {name}: configure -> {ok}")
            elif s == "inactive":
                ok = w.change(name, Transition.TRANSITION_ACTIVATE)
                w.get_logger().info(f"  {name}: activate -> {ok}")
            elif s == "absent":
                pass
        print(f"[{time.monotonic():.1f}] still waiting on: "
              + ", ".join(f"{n}={states[n]}" for n in bad))
        time.sleep(1.0)

    print("TIMEOUT. final states:", states, file=sys.stderr)
    rclpy.shutdown()
    return 1


if __name__ == "__main__":
    sys.exit(main())
