#!/usr/bin/env python3

import os
import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Header
import sensor_msgs_py.point_cloud2 as pc2
from plyfile import PlyData
from ament_index_python.packages import get_package_share_directory

class PointCloudPublisher(Node):
    def __init__(self):
        super().__init__('point_cloud_pub')
        # Publisher for PointCloud2 messages on the 'point_cloud' topic.
        self.publisher_ = self.create_publisher(PointCloud2, 'point_cloud', 10)
        # Publish every 3 seconds.
        timer_period = 3.0
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
        # Absolute path to your PLY file.
        self.ply_file_path = "/home/shan/mywork/RBY1/rby1_ws/src/point_cloud_pub/data/your_mesh.ply"
        self.get_logger().info(f'Reading PLY file from: {self.ply_file_path}')
        
        # Scaling factor for the point cloud.
        self.scaling_factor = 0.001  # Adjust as needed
        
        # Rotation: 180° in radians.
        # For a 180° rotation about the Z-axis, the transformation is (x, y, z) -> (-x, -y, z)
        self.rotation_angle = math.pi  # 180° in radians

    def timer_callback(self):
        try:
            # Read the PLY file.
            plydata = PlyData.read(self.ply_file_path)
            # Assume the PLY file has a 'vertex' element with x, y, z fields.
            vertex = plydata['vertex']
            # Apply scaling factor.
            points = [
                [v['x'] * self.scaling_factor, 
                 v['y'] * self.scaling_factor, 
                 v['z'] * self.scaling_factor] for v in vertex
            ]
            # Apply a 180° rotation about the Z-axis:
            # This rotates each point: (x, y, z) becomes (-x, -y, z).
            offset = 0.0  # Adjust this offset (in meters) as needed
            transformed_points  = [[-(p[0] + offset), p[1], -p[2]] for p in points]

            
            # Create a header for the PointCloud2 message.
            header = Header()
            header.stamp = self.get_clock().now().to_msg()
            header.frame_id = 'link_zvid'  # Ensure this frame matches your RViz fixed frame
            
            # Create and publish the PointCloud2 message.
            cloud = pc2.create_cloud_xyz32(header, transformed_points)
            self.publisher_.publish(cloud)
            self.get_logger().info(f'Published point cloud with {len(transformed_points)} points')
        except Exception as e:
            self.get_logger().error(f'Error reading ply file: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = PointCloudPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()



# #!/usr/bin/env python3

# import os

# import rclpy
# from rclpy.node import Node
# from sensor_msgs.msg import PointCloud2
# from std_msgs.msg import Header
# import sensor_msgs_py.point_cloud2 as pc2
# from plyfile import PlyData
# from ament_index_python.packages import get_package_share_directory

# class PointCloudPublisher(Node):
#     def __init__(self):
#         super().__init__('point_cloud_pub')
#         # Create a publisher for PointCloud2 messages on the 'point_cloud' topic.
#         self.publisher_ = self.create_publisher(PointCloud2, 'point_cloud', 10)
#         # Set a timer to publish every 3 seconds.
#         timer_period = 1.0
#         self.timer = self.create_timer(timer_period, self.timer_callback)
        
#         # Use absolute path or use get_package_share_directory as needed.
#         self.ply_file_path = "/home/shan/mywork/RBY1/rb-y1-main/rb-y1/rby1_ws/src/point_cloud_pub/meshes/your_mesh.ply"
#         self.get_logger().info(f'Reading PLY file from: {self.ply_file_path}')
        
#         # Set the scaling factor (adjust as needed)
#         self.scaling_factor = 0.001

#     def timer_callback(self):
#         try:
#             # Read the PLY file.
#             plydata = PlyData.read(self.ply_file_path)
#             # Assume the PLY file has a 'vertex' element with x, y, z fields.
#             vertex = plydata['vertex']
#             # Apply scaling factor to each coordinate.
#             points = [
#                 [v['x'] * self.scaling_factor, 
#                  v['y'] * self.scaling_factor, 
#                  v['z'] * self.scaling_factor] for v in vertex
#             ]
#             # print(points)

#             # Create a header for the PointCloud2 message.
#             header = Header()
#             header.stamp = self.get_clock().now().to_msg()
#             header.frame_id = 'link_zvid'  # Adjust the frame as needed

#             # Create the PointCloud2 message from the points.
#             cloud = pc2.create_cloud_xyz32(header, points)
#             self.publisher_.publish(cloud)
#             self.get_logger().info(f'Published point cloud with {len(points)} points')
#         except Exception as e:
#             self.get_logger().error(f'Error reading ply file: {e}')

# def main(args=None):
#     rclpy.init(args=args)
#     node = PointCloudPublisher()
#     rclpy.spin(node)
#     node.destroy_node()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()
