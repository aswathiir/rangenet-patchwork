#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py import point_cloud2
import numpy as np
import struct
import os
import onnxruntime as ort

class RangeNetNode(Node):
    def __init__(self):
        super().__init__('rangenet_node')

        # Load ONNX model
        model_path = os.path.expanduser("~/patchwork_ws/src/rangenet_ros2/models/darknet53/model.onnx")
        self.session = ort.InferenceSession(model_path)
        self.input_name = self.session.get_inputs()[0].name
        self.get_logger().info(f"Loaded model: {model_path}")
        self.get_logger().info(f"Model input shape: {self.session.get_inputs()[0].shape}")

        # ROS 2 Interfaces
        self.sub = self.create_subscription(
            PointCloud2, '/kitti/velo/pointcloud', self.callback, 10
        )
        self.pub = self.create_publisher(PointCloud2, '/semantic_points', 10)

        # Color map for semantic classes
        self.color_map = np.array([
            [245, 150, 100], [250, 80, 100], [150, 60, 30],
            [0, 0, 255], [255, 255, 0], [0, 255, 255],
            [0, 255, 0], [255, 0, 255], [255, 0, 0]
        ], dtype=np.uint8)

    def callback(self, msg):
        points = self.parse_pointcloud(msg)
        if points.shape[0] == 0:
            self.get_logger().warn("Empty point cloud received.")
            return

        range_img = self.create_range_image(points)
        input_tensor = range_img[np.newaxis, ...]  # Add batch dimension

        try:
            outputs = self.session.run(None, {self.input_name: input_tensor})
            labels = outputs[0][0].flatten()[:points.shape[0]].astype(np.int32)
            colors = self.color_map[labels % len(self.color_map)]
            final_pts = np.hstack([points, colors])
            self.publish_colored_pointcloud(msg.header, final_pts)
        except Exception as e:
            self.get_logger().error(f"Inference failed: {str(e)}")

    def parse_pointcloud(self, msg):
        try:
            gen = point_cloud2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
            xyz = np.array([[p[0], p[1], p[2]] for p in gen], dtype=np.float32)
            return xyz
        except Exception as e:
            self.get_logger().error(f"Failed to parse point cloud: {str(e)}")
            return np.empty((0, 3), dtype=np.float32)

    def create_range_image(self, xyz, height=64, width=2048):
        range_image = np.zeros((5, height, width), dtype=np.float32)
        if xyz.shape[0] == 0:
            return range_image

        r = np.linalg.norm(xyz, axis=1)
        valid = r > 1e-6
        xyz = xyz[valid]
        r = r[valid]
        if xyz.shape[0] == 0:
            return range_image

        xyz_norm = xyz / r[:, np.newaxis]
        phi = np.arcsin(np.clip(xyz_norm[:, 2], -1, 1))
        theta = np.arctan2(xyz_norm[:, 1], xyz_norm[:, 0])

        row = ((phi - phi.min()) / (phi.max() - phi.min() + 1e-6)) * (height - 1)
        col = ((theta + np.pi) / (2 * np.pi)) * (width - 1)

        row = np.round(row).astype(int)
        col = np.round(col).astype(int)

        mask = (row >= 0) & (row < height) & (col >= 0) & (col < width)
        row, col, xyz = row[mask], col[mask], xyz[mask]

        range_image[0, row, col] = r[mask]
        range_image[1:4, row, col] = xyz.T
        range_image[4, row, col] = 0.5  # Dummy reflectance

        return range_image

    def publish_colored_pointcloud(self, header, points):
        try:
            fields = [
                PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
                PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
                PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
                PointField(name='rgb', offset=12, datatype=PointField.UINT32, count=1)
            ]

            cloud_points = []
            for pt in points:
                x, y, z, r, g, b = pt
                rgb = struct.unpack('I', struct.pack('BBBB', int(b), int(g), int(r), 0))[0]
                cloud_points.append([x, y, z, rgb])

            header.frame_id = 'map'
            cloud_msg = point_cloud2.create_cloud(header, fields, cloud_points)
            self.pub.publish(cloud_msg)
        except Exception as e:
            self.get_logger().error(f"Publishing failed: {str(e)}")

def main(args=None):
    rclpy.init(args=args)
    node = RangeNetNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

