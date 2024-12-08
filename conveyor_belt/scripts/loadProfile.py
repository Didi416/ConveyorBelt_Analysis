#!/usr/bin/python3

import rclpy
from rclpy.node import Node
import open3d as o3d
import numpy as np
from sensor_msgs.msg import PointCloud2
import tf2_ros
import tf2_geometry_msgs
from geometry_msgs.msg import PointStamped, TransformStamped
import matplotlib.pyplot as plt
from scipy.spatial import ConvexHull
import struct

class LoadProfile(Node):
    def __init__(self):
        super().__init__('pointcloud_processor')
        self.subscription = self.create_subscription(PointCloud2, '/depth/color/points', self.pointcloud_callback, 10)
        # self.publisher = self.create_publisher(PointCloud2, '/cropped_point_cloud_topic', 10)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.target_frame = 'base'
        self.source_frame = 'camera_depth_optical_frame'

    def pointcloud_callback(self, msg):
        # points = np.array(list(read_points(msg, field_names=("x", "y", "z"), skip_nans=True)), dtype=np.float32)
        points, colours = self.convert_pointcloud(msg)
        if points is not None and colours is not None:
            self.visualize_in_open3d(points, colours)

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

        self.get_logger().info(f"Hello")
        transformedPoints = self.transformPoints(points)
        # Convert points and colors to numpy arrays
        points = np.array(transformedPoints)
        colors = np.array(colors)

        # Apply the filtering bounds:
        mask = (
            (points[:, 0] >= 0) & (points[:, 0] <= 0.2) &    # X bounds
            (points[:, 1] >= -2) & (points[:, 1] <= 0.75) &  # Y bounds
            (points[:, 2] >= 0) & (points[:, 2] <= 5)        # Z bounds
        )

        # Filter points and colors using the mask
        filtered_points = points[mask]
        filtered_colors = colors[mask]
        return filtered_points, filtered_colors
    
    def transformPoints(self, points):
        try:
            # Lookup the transformation
            transform = self.tf_buffer.lookup_transform(
                self.target_frame,
                self.source_frame,
                rclpy.time.Time()  # Get the latest transform
            )

            # Transform each point
            transformed_points = []
            for point in points:
                point_msg = PointStamped()
                point_msg.header.frame_id = self.source_frame
                point_msg.point.x, point_msg.point.y, point_msg.point.z = point

                # Transform the point
                transformed_point_msg = tf2_geometry_msgs.do_transform_point(point_msg, transform)
                transformed_points.append((
                    transformed_point_msg.point.x,
                    transformed_point_msg.point.y,
                    transformed_point_msg.point.z
                ))
                self.get_logger().info(f"Point done")

            self.get_logger().info(f"Points: {points}")
            self.get_logger().info(f"Transformed Points: {transformed_points}")
            return transformed_points

        except Exception as e:
            self.get_logger().error(f"Error transforming points: {str(e)}")
            return None

    def visualize_in_open3d(self, points, colors):
        # # Create Open3D point cloud
        # pcd = o3d.geometry.PointCloud()
        # pcd.points = o3d.utility.Vector3dVector(points)
        # pcd.colors = o3d.utility.Vector3dVector(colors)

        # # Visualize the point cloud in Open3D
        # o3d.visualization.draw_geometries([pcd])

        # # Compute and display the convex hull in red
        # show_convex_hull = False
        # if show_convex_hull:
        #     hull, _ = pcd.compute_convex_hull()
        #     hull.paint_uniform_color([1, 0, 0])  # Red color
        #     o3d.visualization.draw_geometries([pcd, hull])

        # Now, visualize the YZ profile (only Y and Z values) with matplotlib
        yz_points = points[:, [1, 2]]  # Extract Y and Z points (discard X)
        # Add a column of zeros for the X-axis to make it 3D
        yz_points_3d = np.hstack((np.zeros((yz_points.shape[0], 1)), yz_points))
        yz_colors = colors  # Keep the same colors for YZ view

        method = 'matplotlib'
        if method == 'open3d':
            # Create Open3D PointCloud from the profile points
            pcd_yz = o3d.geometry.PointCloud()
            pcd_yz.points = o3d.utility.Vector3dVector(yz_points_3d)

            # Colors remain the same
            pcd_yz.colors = o3d.utility.Vector3dVector(yz_colors)
            
            # Visualize the profile in Open3D
            o3d.visualization.draw_geometries([pcd_yz])
        
        elif method == 'matplotlib':
            # Perform Convex Hull calculation
            hull = ConvexHull(yz_points)
            
            # Plot the points
            plt.figure(figsize=(6, 6))
            plt.scatter(yz_points[:, 0], yz_points[:, 1], c=colors, s=1)
            
            # Plot the convex hull
            for simplex in hull.simplices:
                plt.plot(yz_points[simplex, 0], yz_points[simplex, 1], 'r-', lw=2)
            
            plt.xlabel('Y')
            plt.ylabel('Z')
            plt.title('YZ Profile of PointCloud with Convex Hull')

            # Invert the Y-axis if needed
            # plt.gca().invert_yaxis()  # Invert Y-axis if required
            plt.show()

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