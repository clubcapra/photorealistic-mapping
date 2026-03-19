#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
import numpy as np

class LivoxToLioSam(Node):
    def __init__(self):
        super().__init__('livox_to_lio_sam')

        # Subscribe to Livox MID360 PointCloud2
        self.subscription = self.create_subscription(
            PointCloud2,
            '/livox/lidar',  # change if your Livox topic is different
            self.pointcloud_callback,
            10
        )

        # Publish LIO-SAM compatible PointCloud2
        self.publisher = self.create_publisher(
            PointCloud2,
            '/points',  # LIO-SAM expects /points
            10
        )

        self.get_logger().info("Livox to LIO-SAM wrapper node started.")

    def pointcloud_callback(self, msg: PointCloud2):
        num_points = msg.width * msg.height

        # Livox MID360 original layout
        old_dtype = np.dtype([
            ('x', np.float32),
            ('y', np.float32),
            ('z', np.float32),
            ('intensity', np.float32),
            ('tag', np.uint8),
            ('line', np.uint8),
            ('offset_time', np.float64)
        ])

        # Load data as structured array
        points = np.frombuffer(msg.data, dtype=old_dtype, count=num_points)

        # New dtype for LIO-SAM
        new_dtype = np.dtype([
            ('x', np.float32),
            ('y', np.float32),
            ('z', np.float32),
            ('intensity', np.float32),
            ('ring', np.uint16),
            ('time', np.float32)
        ])
        new_points = np.zeros(num_points, dtype=new_dtype)

        # Copy fields and convert types
        new_points['x'] = points['x']
        new_points['y'] = points['y']
        new_points['z'] = points['z']
        new_points['intensity'] = points['intensity']
        new_points['ring'] = points['line'].astype(np.uint16)
        new_points['time'] = points['offset_time'].astype(np.float32) * 1e-9  # ns → s

        # Build new PointCloud2 message
        new_msg = PointCloud2()
        new_msg.header = msg.header
        new_msg.height = 1
        new_msg.width = num_points
        new_msg.is_dense = msg.is_dense
        new_msg.is_bigendian = False

        # ROS2 Humble requires keyword arguments for PointField
        new_msg.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1),
            PointField(name='ring', offset=16, datatype=PointField.UINT16, count=1),
            PointField(name='time', offset=18, datatype=PointField.FLOAT32, count=1)
        ]

        new_msg.point_step = 22  # 4+4+4+4+2+4
        new_msg.row_step = new_msg.point_step * num_points
        new_msg.data = new_points.tobytes()

        # Publish
        self.publisher.publish(new_msg)


def main(args=None):
    rclpy.init(args=args)
    node = LivoxToLioSam()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()