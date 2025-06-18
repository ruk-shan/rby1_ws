#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Quaternion, TransformStamped, Twist, Vector3
from tf2_ros import TransformBroadcaster
import math

class OdomPublisher(Node):
    def __init__(self):
        super().__init__('odom_publisher')
        
        # Set logging level to DEBUG
        self.get_logger().set_level(rclpy.logging.LoggingSeverity.DEBUG)
        
        # Robot parameters (from URDF)
        self.wheel_radius = 0.1  # meters
        self.wheel_base = 0.53   # meters (distance between wheels)
        
        # Initialize variables
        self.left_wheel_pos = 0.0
        self.right_wheel_pos = 0.0
        self.last_left_pos = 0.0
        self.last_right_pos = 0.0
        self.last_time = self.get_clock().now()
        
        # Robot pose
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        
        # Robot velocities
        self.vx = 0.0
        self.vth = 0.0
        
        # Create publisher and subscriber
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        self.joint_sub = self.create_subscription(
            JointState,
            'joint_states',
            self.joint_states_callback,
            10)
        
        # For broadcasting transform
        self.tf_broadcaster = TransformBroadcaster(self)
        
        self.get_logger().info('Odometry publisher initialized')
    
    def joint_states_callback(self, msg):
        # Get current time
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9  # Convert to seconds
        
        if dt <= 0.0:
            return  # Avoid division by zero
        
        # Find wheel positions in the message
        try:
            # Debug: Print all available joint names
            self.get_logger().debug(f'Available joints: {msg.name}')
            
            left_idx = msg.name.index('left_wheel')
            right_idx = msg.name.index('right_wheel')
            
            self.left_wheel_pos = msg.position[left_idx]
            self.right_wheel_pos = msg.position[right_idx]
            
            # Debug: Print raw wheel positions
            self.get_logger().debug(f'Left wheel pos: {self.left_wheel_pos}, Right wheel pos: {self.right_wheel_pos}')
            
            # Calculate wheel displacements (negative sign to fix direction)
            d_left = -(self.left_wheel_pos - self.last_left_pos) * self.wheel_radius
            d_right = -(self.right_wheel_pos - self.last_right_pos) * self.wheel_radius
            
            # Debug: Print wheel displacements
            self.get_logger().debug(f'd_left: {d_left}, d_right: {d_right}')
            
            # Update last positions
            self.last_left_pos = self.left_wheel_pos
            self.last_right_pos = self.right_wheel_pos
            
            # Calculate linear and angular displacements
            # For differential drive:
            # d_center = average of both wheel movements
            # d_theta = (right_wheel_distance - left_wheel_distance) / wheel_base
            d_center = (d_right + d_left) / 2.0
            d_theta = (d_right - d_left) / self.wheel_base
            
            # Debug: Print calculated displacements
            self.get_logger().debug(f'd_center: {d_center}, d_theta: {d_theta}')
            
            # Update pose
            self.theta += d_theta
            self.x += d_center * math.cos(self.theta)
            self.y += d_center * math.sin(self.theta)
            
            # Debug: Print final pose
            self.get_logger().debug(f'New pose - x: {self.x}, y: {self.y}, theta: {self.theta}')
            
            # Publish odometry
            self.publish_odometry(current_time)
            
            # Calculate velocities
            self.vx = d_center / dt
            self.vth = d_theta / dt
            
            # Publish odometry message
            self.publish_odometry(current_time)
            
        except ValueError as e:
            self.get_logger().warn(f'Could not find wheel joints: {str(e)}')
        
        # Update last time
        self.last_time = current_time
    
    def publish_odometry(self, current_time):
        # Create and publish odometry message
        odom_msg = Odometry()
        odom_msg.header.stamp = current_time.to_msg()
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = 'base'
        
        # Set position
        odom_msg.pose.pose.position.x = self.x
        odom_msg.pose.pose.position.y = self.y
        odom_msg.pose.pose.position.z = 0.0
        
        # Convert yaw to quaternion
        q = self.quaternion_from_euler(0, 0, self.theta)
        odom_msg.pose.pose.orientation = q
        
        # Set velocity
        odom_msg.twist.twist = Twist(
            linear=Vector3(x=self.vx, y=0.0, z=0.0),
            angular=Vector3(x=0.0, y=0.0, z=self.vth)
        )
        
        # Publish the message
        self.odom_pub.publish(odom_msg)
        
        # Broadcast transform (base -> odom)
        # Note: We're actually publishing the inverse of the robot's pose
        # This is because RViz expects the transform from the child frame to the parent frame
        t = TransformStamped()
        t.header.stamp = current_time.to_msg()
        t.header.frame_id = 'base'  # Child frame (robot's base)
        t.child_frame_id = 'odom'    # Parent frame (odometry frame)
        
        # Set the transform to be the inverse of the robot's pose
        # This is equivalent to the transform from base to odom
        t.transform.translation.x = -self.x * math.cos(self.theta) - self.y * math.sin(self.theta)
        t.transform.translation.y = self.x * math.sin(self.theta) - self.y * math.cos(self.theta)
        t.transform.translation.z = 0.0
        
        # Invert the quaternion for the transform
        t.transform.rotation.w = q.w
        t.transform.rotation.x = -q.x
        t.transform.rotation.y = -q.y
        t.transform.rotation.z = -q.z
        
        self.tf_broadcaster.sendTransform(t)
    
    @staticmethod
    def quaternion_from_euler(roll, pitch, yaw):
        """
        Convert Euler angles to quaternion
        """
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)
        
        q = Quaternion()
        q.w = cr * cp * cy + sr * sp * sy
        q.x = sr * cp * cy - cr * sp * sy
        q.y = cr * sp * cy + sr * cp * sy
        q.z = cr * cp * sy - sr * sp * cy
        
        return q

def main(args=None):
    rclpy.init(args=args)
    odom_publisher = OdomPublisher()
    
    try:
        rclpy.spin(odom_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        odom_publisher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
