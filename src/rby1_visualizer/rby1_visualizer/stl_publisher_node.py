#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Pose, Vector3
from std_msgs.msg import ColorRGBA
from builtin_interfaces.msg import Duration
import os
from ament_index_python.packages import get_package_share_directory

class StlPublisher(Node):
    def __init__(self):
        super().__init__('stl_publisher_node')

        # --- Parameters ---
        self.declare_parameter('stl_file_path', 'meshes/NewBox5_STL.stl') # Relative path within package share
        self.declare_parameter('marker_topic', '/visualization_marker')
        self.declare_parameter('frame_id', 'base') # MUST exist in TF tree
        self.declare_parameter('publish_rate_hz', 1.0) # Hz
        self.declare_parameter('scale_x', 0.001)
        self.declare_parameter('scale_y', 0.001)
        self.declare_parameter('scale_z', 0.001)
        self.declare_parameter('color_r', 0.5)
        self.declare_parameter('color_g', 0.5)
        self.declare_parameter('color_b', 0.5)
        self.declare_parameter('color_a', 1.0) # Alpha (transparency)
        self.declare_parameter('position_x', 0.0)
        self.declare_parameter('position_y', 0.0)
        self.declare_parameter('position_z', 0.0)
        self.declare_parameter('orientation_x', 0.0)
        self.declare_parameter('orientation_y', 0.0)
        self.declare_parameter('orientation_z', 0.0)
        self.declare_parameter('orientation_w', 1.0) # Default: no rotation

        # --- Get parameter values ---
        stl_relative_path = self.get_parameter('stl_file_path').get_parameter_value().string_value
        marker_topic = self.get_parameter('marker_topic').get_parameter_value().string_value
        self.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value
        publish_rate = self.get_parameter('publish_rate_hz').get_parameter_value().double_value
        self.scale = Vector3(
            x=self.get_parameter('scale_x').get_parameter_value().double_value,
            y=self.get_parameter('scale_y').get_parameter_value().double_value,
            z=self.get_parameter('scale_z').get_parameter_value().double_value)
        self.color = ColorRGBA(
            r=float(self.get_parameter('color_r').get_parameter_value().double_value),
            g=float(self.get_parameter('color_g').get_parameter_value().double_value),
            b=float(self.get_parameter('color_b').get_parameter_value().double_value),
            a=float(self.get_parameter('color_a').get_parameter_value().double_value))
        self.pose = Pose()
        self.pose.position.x = self.get_parameter('position_x').get_parameter_value().double_value
        self.pose.position.y = self.get_parameter('position_y').get_parameter_value().double_value
        self.pose.position.z = self.get_parameter('position_z').get_parameter_value().double_value
        self.pose.orientation.x = self.get_parameter('orientation_x').get_parameter_value().double_value
        self.pose.orientation.y = self.get_parameter('orientation_y').get_parameter_value().double_value
        self.pose.orientation.z = self.get_parameter('orientation_z').get_parameter_value().double_value
        self.pose.orientation.w = self.get_parameter('orientation_w').get_parameter_value().double_value


        # --- Construct the mesh resource path ---
        # Assumes the STL is in the 'install/<pkg_name>/share/<pkg_name>/meshes' directory after building
        pkg_share_dir = get_package_share_directory('rby1_visualizer')
        # Use os.path.join for cross-platform compatibility
        absolute_stl_path = os.path.join(pkg_share_dir, stl_relative_path)

        # Method 1: Using package:// URI (Recommended if STL is installed with the package)
        self.mesh_resource_uri = f"package://rby1_visualizer/{stl_relative_path}"

        # Method 2: Using file:// URI (Use if STL is at an arbitrary absolute path)
        # self.mesh_resource_uri = f"file://{absolute_stl_path}"

        # Verify the file exists (optional but helpful)
        # Note: RViz looks for the file itself using the URI, this check is for the node publisher
        if not os.path.exists(absolute_stl_path):
             self.get_logger().error(f"STL file not found at: {absolute_stl_path}")
             self.get_logger().error(f"Mesh resource URI used: {self.mesh_resource_uri}")
             # Decide if you want to exit or just continue without publishing
             # return # Or raise an exception


        # --- Publisher and Timer ---
        self.publisher_ = self.create_publisher(Marker, marker_topic, 10)
        self.timer = self.create_timer(1.0 / publish_rate, self.timer_callback)
        self.marker_id = 0

        self.get_logger().info(f"Publishing STL marker from '{self.mesh_resource_uri}'")
        self.get_logger().info(f"  on topic '{marker_topic}'")
        self.get_logger().info(f"  relative to frame '{self.frame_id}'")


    def timer_callback(self):
        marker = Marker()

        marker.header.frame_id = self.frame_id
        marker.header.stamp = self.get_clock().now().to_msg()

        marker.ns = "stl_model" # Namespace
        marker.id = self.marker_id # Unique ID for this marker

        marker.type = Marker.MESH_RESOURCE # Specify mesh type
        marker.action = Marker.ADD # Add/Modify the marker

        # Set the pose (position and orientation)
        marker.pose = self.pose

        # Set the scale
        marker.scale = self.scale

        # Set the color (and transparency)
        marker.color = self.color

        # Set the mesh resource (path to the STL file)
        marker.mesh_resource = self.mesh_resource_uri
        marker.mesh_use_embedded_materials = False # Use marker.color instead of STL material

        # Set the lifetime (Duration() -> lasts forever, or use Duration(sec=N) )
        # Setting non-zero lifetime means it will republish on the timer
        # Setting zero means it only needs to be published once (but timer keeps running here)
        marker.lifetime = Duration() # Lasts forever

        # Publish the marker
        self.publisher_.publish(marker)

        # Optional: Log that we published
        # self.get_logger().info(f'Publishing marker {marker.id}')

        # Optional: Increment ID if you plan to publish multiple *different* markers
        # self.marker_id += 1

def main(args=None):
    rclpy.init(args=args)
    node = StlPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Cleanup
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()