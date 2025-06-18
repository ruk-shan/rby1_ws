#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import tf2_ros
from geometry_msgs.msg import PoseStamped, TransformStamped
from tf2_geometry_msgs import do_transform_pose

class CameraToTorsoTransform(Node):
    def __init__(self):
        super().__init__('camera_to_torso_transform')
        
        # Initialize the transform buffer and listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        # Create a publisher for the transformed pose
        self.transformed_pose_pub = self.create_publisher(
            PoseStamped, 
            '/object_pose_in_torso', 
            10
        )
        
        # Create a subscriber to the camera's object pose
        self.object_pose_sub = self.create_subscription(
            PoseStamped,
            '/camera/object_pose',  # Update this to your actual topic
            self.pose_callback,
            10
        )
        
        self.get_logger().info('Camera to Torso Transform Node started')
        self.get_logger().info('Waiting for transform from link_camera_frame to link_torso_5...')
    
    def pose_callback(self, msg):
        try:
            # Get the transform from camera frame to torso frame
            transform = self.tf_buffer.lookup_transform(
                'link_torso_5',  # Target frame
                msg.header.frame_id,  # Source frame (should be link_camera_frame)
                rclpy.time.Time()
            )
            
            # Transform the pose
            transformed_pose = do_transform_pose(msg, transform)
            
            # Publish the transformed pose
            self.transformed_pose_pub.publish(transformed_pose)
            
            # Log the transformation for debugging
            pos = transformed_pose.pose.position
            orient = transformed_pose.pose.orientation
            self.get_logger().info(f'Transformed pose in link_torso_5 frame:')
            self.get_logger().info(f'  Position: x={pos.x:.3f}, y={pos.y:.3f}, z={pos.z:.3f}')
            self.get_logger().info(f'  Orientation: x={orient.x:.3f}, y={orient.y:.3f}, z={orient.z:.3f}, w={orient.w:.3f}')
            
        except (tf2_ros.LookupException, 
                tf2_ros.ConnectivityException, 
                tf2_ros.ExtrapolationException) as e:
            self.get_logger().error(f'Could not transform from {msg.header.frame_id} to link_torso_5: {str(e)}')

def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = CameraToTorsoTransform()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
