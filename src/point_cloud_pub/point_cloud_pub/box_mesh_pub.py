import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker

class MeshMarkerPublisher(Node):
    def __init__(self):
        super().__init__('mesh_marker_publisher')
        self.publisher_ = self.create_publisher(Marker, 'visualization_marker', 10)
        timer_period = 1.0  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        marker = Marker()
        marker.header.frame_id = "base"  # Make sure this frame exists in your TF tree
        marker.header.stamp = self.get_clock().now().to_msg()

        marker.ns = "box_marker"
        marker.id = 0
        marker.type = Marker.CUBE  # Use a cube marker type for a box shape
        marker.action = Marker.ADD

        # Set the dimensions (in meters: 280mm -> 0.28m, 160mm -> 0.16m, 80mm -> 0.08m)
        marker.scale.x = 0.293
        marker.scale.y = 0.192
        marker.scale.z = 0.1185
    
        # Set the marker's pose
        marker.pose.position.x = 0.5
        marker.pose.position.y = 0.0
        marker.pose.position.z = 1.0
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.7071  # sin(pi/4)
        marker.pose.orientation.w = 0.7071  # cos(pi/4)

        # Set the color and opacity (alpha must be non-zero to see the mesh)
        marker.color.r = 0.5
        marker.color.g = 1.0
        marker.color.b = 0.5
        marker.color.a = 1.0

        # Lifetime 0 means the marker never auto-deletes
        marker.lifetime.sec = 0
        marker.lifetime.nanosec = 0

        # Publish the marker
        self.publisher_.publish(marker)
        self.get_logger().info("Published mesh marker.")

def main(args=None):
    rclpy.init(args=args)
    node = MeshMarkerPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
