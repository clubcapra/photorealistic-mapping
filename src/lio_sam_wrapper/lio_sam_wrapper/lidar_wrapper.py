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
        # Make a shallow copy of the message 
        new_msg = PointCloud2() 
        new_msg.header = msg.header 
        new_msg.height = msg.height 
        new_msg.width = msg.width 
        new_msg.is_dense = msg.is_dense 
        new_msg.is_bigendian = msg.is_bigendian 
        new_msg.point_step = msg.point_step 
        new_msg.row_step = msg.row_step 
        new_msg.data = msg.data 
        # Copy and rename fields 
        new_fields = list(msg.fields) 
        if len(new_fields) > 6: 
            new_fields[5].name = 'ring' 
            new_fields[5].datatype = PointField.UINT16 
            new_fields[6].name = 'time' 
            new_fields[6].datatype = PointField.FLOAT32 
        else: 
            self.get_logger().warn( f'PointCloud2 has only {len(new_fields)} fields; ' 'cannot rename indices 5 and 6' ) 
            new_msg.fields = new_fields 
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