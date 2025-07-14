#!/usr/bin/env python3

#############################################################
# IMPORTS AND DEPENDENCIES
#############################################################
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformListener, Buffer, TransformException
from tf_transformations import euler_from_quaternion
import numpy as np
from scipy.spatial.transform import Rotation

#############################################################
# FORWARD KINEMATICS LISTENER NODE
#############################################################
class TF2Listener(Node):
    def __init__(self):
        super().__init__('ee_transform_listener')
        
        # Parameters
        self.declare_parameter('base_link', 'link_torso_5')
        self.declare_parameter('target_links', ['ee_left', 'ee_right'])
        self.declare_parameter('calculation_rate', 10.0)  # Hz
        
        # Get parameters
        self.base_link = self.get_parameter('base_link').value
        self.target_links = self.get_parameter('target_links').value
        self.calculation_rate = self.get_parameter('calculation_rate').value
        
        # TF2 setup
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # TIMER
        self.timer = self.create_timer(1.0 / self.calculation_rate, self.timer_callback)
        self.get_logger().info(f"Transform lookup timer set to {self.calculation_rate} Hz")
        self.get_logger().info(f"Looking for transforms from {self.base_link} to {self.target_links}")
        self.get_logger().info("Press Ctrl+C to stop")
        print("\n" + "="*60)
        print(f"  Transformations relative to: {self.base_link}")
        print("="*60)
    
    def timer_callback(self):
        # Define the links we're interested in
        base_link = 'link_torso_5'
        target_links = ['ee_left', 'ee_right']
        
        for target_link in target_links:
            try:
                # Get the transform from torso_5 to target link
                transform = self.tf_buffer.lookup_transform(
                    base_link,  # source frame
                    target_link,  # target frame
                    rclpy.time.Time(),
                    timeout=rclpy.duration.Duration(seconds=1.0)
                )
                
                # Extract position and orientation
                position = transform.transform.translation
                orientation = transform.transform.rotation
                
                # Convert quaternion to Euler angles (roll, pitch, yaw)
                euler = euler_from_quaternion([
                    orientation.x,
                    orientation.y,
                    orientation.z,
                    orientation.w
                ])
                
                # Print the transform in a clear format
                print(f"\n{'-'*60}")
                print(f"  Transform: {self.base_link} -> {target_link}")
                print(f"  {'Position (meters)':<25} [x, y, z]")
                print(f"  {' ':<25} [{position.x:>8.4f}, {position.y:>8.4f}, {position.z:>8.4f}]")
                
                print(f"\n  {'Orientation (quaternion)':<25} [x, y, z, w]")
                print(f"  {' ':<25} [{orientation.x:>8.4f}, {orientation.y:>8.4f}, {orientation.z:>8.4f}, {orientation.w:>8.4f}]")
                
                print(f"\n  {'Orientation (Euler)':<25} [roll, pitch, yaw] (radians)")
                print(f"  {' ':<25} [{euler[0]:>8.4f}, {euler[1]:>8.4f}, {euler[2]:>8.4f}]")
                
                print(f"  {' ':<25} [{(euler[0]*180/3.14159):>8.2f}, {(euler[1]*180/3.14159):>8.2f}, {(euler[2]*180/3.14159):>8.2f}] (degrees)")
                print(f"{'-'*60}")
                
            except Exception as e:
                self.get_logger().warn(f"Could not find transform from {self.base_link} to {target_link}: {e}")


#############################################################
# MAIN FUNCTION
#############################################################
def main(args=None):
    rclpy.init(args=args)
    
    try:
        # Uncomment the node you want to use
        
        # Option 1: Use TF2 listener (requires TF2 transforms to be published)
        node = TF2Listener()
        
        # Option 2: Use hardcoded transforms (for testing without TF2)
        # node = HardcodedTransforms()
        
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\nTransform printer stopped by user")
    except Exception as e:
        print(f"Error: {e}")
    finally:
        # Clean up
        rclpy.shutdown()

if __name__ == '__main__':
    main()