#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import tf2_ros
from geometry_msgs.msg import TransformStamped, PoseStamped
from tf2_geometry_msgs import do_transform_pose
import sys

class TransformListenerNode(Node):
    def __init__(self):
        super().__init__('transform_listener_node')
        
        # Initialize the transform buffer and listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        # Create a timer to check for transforms periodically
        self.timer = self.create_timer(1.0, self.timer_callback)
        
        self.get_logger().info('Transform listener node started. Waiting for transform...')
    
    def timer_callback(self):
        # These frames should be passed as parameters in a real application
        target_frame = self.get_parameter('target_frame').get_parameter_value().string_value
        source_frame = self.get_parameter('source_frame').get_parameter_value().string_value
        
        try:
            # Get the transform from source_frame to target_frame
            transform = self.tf_buffer.lookup_transform(
                target_frame=target_frame,
                source_frame=source_frame,
                time=rclpy.time.Time()
            )
            
            # Log the transform
            self.get_logger().info(f'Transform from {source_frame} to {target_frame}:')
            self.get_logger().info(f'  Translation: x={transform.transform.translation.x:.3f}, y={transform.transform.translation.y:.3f}, z={transform.transform.translation.z:.3f}')
            self.get_logger().info(f'  Rotation: x={transform.transform.rotation.x:.3f}, y={transform.transform.rotation.y:.3f}, z={transform.transform.rotation.z:.3f}, w={transform.transform.rotation.w:.3f}')
            
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            self.get_logger().error(f'Could not get transform from {source_frame} to {target_frame}: {str(e)}')

def main(args=None):
    rclpy.init(args=args)
    
    node = TransformListenerNode()
    
    # Declare parameters with default values
    node.declare_parameter('target_frame', 'link_torso_5')
    node.declare_parameter('source_frame', 'link_ee_left')
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
