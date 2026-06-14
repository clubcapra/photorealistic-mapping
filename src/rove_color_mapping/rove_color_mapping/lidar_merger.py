#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2


class LidarMerger(Node):
    """Merge two Livox MID360 clouds into one.

    Both lidars publish in the same frame with an identical point layout (the
    driver applies each unit's extrinsics), so merging is a zero-copy byte
    concatenation — no per-point deserialization. The previous implementation
    decoded ~40k points/cycle in pure Python, which throttled the 10 Hz lidars
    to ~1 Hz (and pegged a core), and re-stamped the output with wall-clock-now,
    injecting the merger's own latency/jitter into the lidar timeline.

    Pairing is "latest of each": publish whenever both units have a fresh cloud,
    then clear. The units are time-synced (~2.5 ms apart), so the paired clouds
    are temporally close. This is deliberately simpler/more robust than a
    timestamp synchronizer, which would silently stop emitting if a lidar clock
    ever drifted beyond the matching window — a dangerous failure for a SLAM
    input.
    """

    def __init__(self):
        super().__init__('lidar_merger')

        self.declare_parameter('topic_1', '/livox/lidar_192_168_2_41')
        self.declare_parameter('topic_2', '/livox/lidar_192_168_2_40')
        self.declare_parameter('output_topic', '/livox/lidar')
        self.declare_parameter('output_frame', 'livox_frame')

        topic_1      = self.get_parameter('topic_1').value
        topic_2      = self.get_parameter('topic_2').value
        output_topic = self.get_parameter('output_topic').value
        self.output_frame = self.get_parameter('output_frame').value

        self.cloud_1 = None
        self.cloud_2 = None

        # Reliable intake to match the Livox driver (a best-effort subscriber
        # received nothing from it). Output reliable for icp_odometry.
        self.sub_1 = self.create_subscription(
            PointCloud2, topic_1, self.cb_cloud_1, 10)
        self.sub_2 = self.create_subscription(
            PointCloud2, topic_2, self.cb_cloud_2, 10)

        self.pub = self.create_publisher(PointCloud2, output_topic, 10)

        self.get_logger().info(
            f'Merging {topic_1} + {topic_2} → {output_topic}')

    def cb_cloud_1(self, msg):
        self.cloud_1 = msg
        self.try_publish()

    def cb_cloud_2(self, msg):
        self.cloud_2 = msg
        self.try_publish()

    def try_publish(self):
        c1, c2 = self.cloud_1, self.cloud_2
        if c1 is None or c2 is None:
            return
        # Consume the pair up front so we never block on stale data.
        self.cloud_1 = self.cloud_2 = None

        if c1.point_step != c2.point_step or c1.fields != c2.fields:
            self.get_logger().warn(
                'Lidar point layouts differ; skipping this frame.')
            return

        out = PointCloud2()
        # Keep the lidar capture stamp (the later of the two), not now().
        s1 = c1.header.stamp.sec * 1_000_000_000 + c1.header.stamp.nanosec
        s2 = c2.header.stamp.sec * 1_000_000_000 + c2.header.stamp.nanosec
        out.header = c1.header if s1 >= s2 else c2.header
        out.header.frame_id = self.output_frame
        out.height = 1
        out.fields = c1.fields
        out.is_bigendian = c1.is_bigendian
        out.point_step = c1.point_step
        out.is_dense = c1.is_dense and c2.is_dense
        out.data = bytes(c1.data) + bytes(c2.data)
        out.width = c1.width + c2.width
        out.row_step = out.point_step * out.width
        self.pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = LidarMerger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
