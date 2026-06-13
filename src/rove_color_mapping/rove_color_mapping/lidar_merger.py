#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
from std_msgs.msg import Header
import numpy as np


class LidarMerger(Node):
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
        if self.cloud_1 is None or self.cloud_2 is None:
            return

        try:
            # Read all fields as raw structured arrays to preserve exact dtype
            field_names = [f.name for f in self.cloud_1.fields]

            points_1 = np.array(list(pc2.read_points(
                self.cloud_1, field_names=field_names, skip_nans=True)))
            points_2 = np.array(list(pc2.read_points(
                self.cloud_2, field_names=field_names, skip_nans=True)))

            if points_1.size == 0 and points_2.size == 0:
                return
            elif points_1.size == 0:
                merged = points_2
            elif points_2.size == 0:
                merged = points_1
            else:
                merged = np.concatenate((points_1, points_2), axis=0)

            header = Header()
            header.stamp = self.get_clock().now().to_msg()
            header.frame_id = self.output_frame

            out_msg = pc2.create_cloud(header, self.cloud_1.fields, merged)
            self.pub.publish(out_msg)

        except Exception as e:
            self.get_logger().warn(f'Merge failed: {e}')

        finally:
            # Always reset so we don't block on stale data
            self.cloud_1 = None
            self.cloud_2 = None


def main(args=None):
    rclpy.init(args=args)
    node = LidarMerger()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()