#!/usr/bin/python3

import rclpy
from rclpy.node import Node
import open3d as o3d
import numpy as np
from sensor_msgs.msg import PointCloud2
# from sensor_msgs_py.point_cloud2 import read_points
# import matplotlib.pyplot as plot
import struct

class LoadProfile(Node):
    def __init__(self):
        super().__init__('pointcloud_processor')
        self.subscription = self.create_subscription(PointCloud2, '/depth/color/points', self.pointcloud_callback, 10)
        # self.publisher = self.create_publisher(PointCloud2, '/cropped_point_cloud_topic', 10)

    def pointcloud_callback(self, msg):
        # points = np.array(list(read_points(msg, field_names=("x", "y", "z"), skip_nans=True)), dtype=np.float32)
        points, colours = self.convert_pointcloud(msg)
        if points is not None and colours is not None:
            self.visualize_in_open3d(points, colours)
        # cloud = o3d.geometry.PointCloud()
        # cloud.points = o3d.utility.Vector3dVector(points)

        # Transform, crop, and profile
        # transformed_cloud = self.transform_to_base_frame(cloud)
        # cropped_cloud = self.crop_pointcloud(cloud)
        # self.publisher.publish(cropped_cloud)

    # def transform_to_base_frame(self, cloud):
    #     transformation = np.eye(4)
    #     transformation[:3, :3] = o3d.geometry.get_rotation_matrix_from_xyz([np.pi / 6, 0, 0])
    #     transformation[:3, 3] = [0, 0, -2]
    #     return cloud.transform(transformation)

    def convert_pointcloud(self, msg):
        points = []
        colors = []

        # Parse the PointCloud2 message
        point_step = msg.point_step
        data = msg.data
        for i in range(0, len(data), point_step):
            chunk = data[i:i + point_step]
            if len(chunk) < point_step:
                break

            # Unpack x, y, z, and rgb fields
            x, y, z = struct.unpack('fff', chunk[0:12])
            rgb = struct.unpack('I', chunk[16:20])[0]

            # Extract RGB components
            red = (rgb >> 16) & 0xFF
            green = (rgb >> 8) & 0xFF
            blue = rgb & 0xFF

            points.append([x, y, z])
            colors.append([red / 255.0, green / 255.0, blue / 255.0])

        # Convert points and colors to numpy arrays
        points = np.array(points)
        colors = np.array(colors)

        # Apply the filtering bounds:
        # X: -0.25 to 0.25
        # Y: -0.25 to 0.25
        # Z: 0 to 0.4
        mask = (
            (points[:, 0] >= 0) & (points[:, 0] <= 0.5) &  # X bounds
            (points[:, 1] >= 0) & (points[:, 1] <= 2) &  # Y bounds
            (points[:, 2] >= 0) & (points[:, 2] <= 4)        # Z bounds
        )

        # Filter points and colors using the mask
        filtered_points = points[mask]
        filtered_colors = colors[mask]
        return filtered_points, filtered_colors

    def visualize_in_open3d(self, points, colors):
        # Create Open3D point cloud
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(points)
        pcd.colors = o3d.utility.Vector3dVector(colors)

        # Visualize the point cloud in Open3D
        o3d.visualization.draw_geometries([pcd])

        # Compute and display the convex hull in red
        show_convex_hull = False
        if show_convex_hull:
            hull, _ = pcd.compute_convex_hull()
            hull.paint_uniform_color([1, 0, 0])  # Red color
            o3d.visualization.draw_geometries([pcd, hull])

        # Now, visualize the YZ profile (only Y and Z values) with matplotlib
        yz_points = points[:, [1, 2]]  # Extract Y and Z points (discard X)
        # Add a column of zeros for the X-axis to make it 3D
        yz_points_3d = np.hstack((np.zeros((yz_points.shape[0], 1)), yz_points))
        yz_colors = colors  # Keep the same colors for YZ view

        method = 'open3d'
        if method == 'open3d':
            # Create Open3D PointCloud from the profile points
            pcd_yz = o3d.geometry.PointCloud()
            pcd_yz.points = o3d.utility.Vector3dVector(yz_points_3d)

            # Colors remain the same
            pcd_yz.colors = o3d.utility.Vector3dVector(yz_colors)
            
            # Visualize the profile in Open3D
            o3d.visualization.draw_geometries([pcd_yz])

    # def crop_pointcloud(self, cloud):
    #     cbox = o3d.geometry.AxisAlignedBoundingBox(min_bound=(0, 0, 0), max_bound=(1, 1, 1))
    #     return cloud.crop(cbox)

def main():
    rclpy.init()
    node = LoadProfile()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down...')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()