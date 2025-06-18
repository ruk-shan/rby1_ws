#!/usr/bin/env python3

#############################################################
# IMPORTS AND DEPENDENCIES
#############################################################
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from tf2_ros import Buffer, TransformListener
from geometry_msgs.msg import TransformStamped
import math
import numpy as np
from scipy.spatial.transform import Rotation

#############################################################
# FORWARD KINEMATICS LISTENER NODE
#############################################################
class TF2Listener(Node):
    def __init__(self):
        super().__init__('tf2_listener')
        
        #----------------------------------------------------
        # PARAMETERS
        #----------------------------------------------------
        # Declare parameters
        self.declare_parameter('base_link', 'link_torso_5')
        self.declare_parameter('target_links', ['ee_left', 'ee_right'])
        self.declare_parameter('calculation_rate', 1.0)  # Hz
        
        # Get parameters
        self.base_link = self.get_parameter('base_link').get_parameter_value().string_value
        self.target_links = self.get_parameter('target_links').get_parameter_value().string_array_value
        self.calculation_rate = self.get_parameter('calculation_rate').get_parameter_value().double_value
        
        #----------------------------------------------------
        # TF2 SETUP
        #----------------------------------------------------
        # Set up TF buffer and listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.get_logger().info("TF2 listener initialized")
        
        #----------------------------------------------------
        # TIMER
        #----------------------------------------------------
        # Create timer for transform lookup
        self.timer = self.create_timer(1.0 / self.calculation_rate, self.lookup_transforms)
        self.get_logger().info(f"Transform lookup timer set to {self.calculation_rate} Hz")
        
        self.get_logger().info(f"Looking for transforms from {self.base_link} to {self.target_links}")
    
    def lookup_transforms(self):
        """Look up transforms between base link and target links"""
        for target_link in self.target_links:
            try:
                # Look up the transform
                transform = self.tf_buffer.lookup_transform(
                    self.base_link,
                    target_link,
                    rclpy.time.Time(),
                    rclpy.duration.Duration(seconds=1.0)
                )
                
                # Extract position and orientation
                position = transform.transform.translation
                orientation = transform.transform.rotation
                
                # Convert quaternion to Euler angles (roll, pitch, yaw)
                quat = [orientation.x, orientation.y, orientation.z, orientation.w]
                rot = Rotation.from_quat(quat)
                euler = rot.as_euler('xyz', degrees=False)  # Get Euler angles in radians
                
                # Print the transform in terminal
                print(f"\n{'-'*50}")
                print(f"Transform from {self.base_link} to {target_link}:")
                print(f"Position (x, y, z): [{position.x:.6f}, {position.y:.6f}, {position.z:.6f}]")
                print(f"Orientation (quaternion x, y, z, w): [{orientation.x:.6f}, {orientation.y:.6f}, {orientation.z:.6f}, {orientation.w:.6f}]")
                print(f"Orientation (Euler xyz, radians): [{euler[0]:.6f}, {euler[1]:.6f}, {euler[2]:.6f}]")
                print(f"{'-'*50}")
                
            except Exception as e:
                self.get_logger().warn(f"Could not find transform from {self.base_link} to {target_link}: {e}")

#############################################################
# ALTERNATIVE: HARD-CODED TRANSFORMS FOR TESTING
#############################################################
class HardcodedTransforms(Node):
    def __init__(self):
        super().__init__('hardcoded_transforms')
        
        # Declare parameters
        self.declare_parameter('calculation_rate', 1.0)  # Hz
        self.calculation_rate = self.get_parameter('calculation_rate').get_parameter_value().double_value
        
        # Create timer
        self.timer = self.create_timer(1.0 / self.calculation_rate, self.print_transforms)
        self.get_logger().info("Hardcoded transforms initialized")
        
        # Define hardcoded transforms for testing
        # These values should be replaced with your actual values
        # self.transforms = {
        #     'ee_left': {
        #         'position': [0.3, 0.27, -0.15],
        #         'orientation': [-0.5213, -0.4777, -0.5213, 0.4777]  # x, y, z, w
        #     },
        #     'ee_right': {
        #         'position': [0.3, -0.27, -0.15],
        #         'orientation': [-0.5213, 0.4777, -0.5213, -0.4777]  # x, y, z, w
        #     }
        # }
    
    def print_transforms(self):
        """Print hardcoded transforms"""
        # base_link = 'link_torso_5'
        base_link = 'link_camera_frame'
        
        for target_link, transform in self.transforms.items():
            position = transform['position']
            orientation = transform['orientation']
            
            # Convert quaternion to Euler angles (roll, pitch, yaw)
            rot = Rotation.from_quat(orientation)
            euler = rot.as_euler('xyz', degrees=False)  # Get Euler angles in radians
            
            print(f"\n{'-'*50}")
            print(f"Transform from {base_link} to {target_link}:")
            print(f"Position (x, y, z): [{position[0]:.6f}, {position[1]:.6f}, {position[2]:.6f}]")
            print(f"Orientation (quaternion x, y, z, w): [{orientation[0]:.6f}, {orientation[1]:.6f}, {orientation[2]:.6f}, {orientation[3]:.6f}]")
            print(f"Orientation (Euler xyz, radians): [{euler[0]:.6f}, {euler[1]:.6f}, {euler[2]:.6f}]")
            print(f"{'-'*50}")

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