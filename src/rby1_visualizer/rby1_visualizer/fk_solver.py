#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped, TransformStamped
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
import numpy as np
from urdf_parser_py.urdf import URDF
import tf2_geometry_msgs
import tf_transformations
import os
import yaml
import json
from std_msgs.msg import String
from ament_index_python.packages import get_package_share_directory


class ForwardKinematicsSolver(Node):
    """
    A ROS2 node that calculates forward kinematics for a robot from a URDF file.
    It subscribes to joint states and publishes the Cartesian positions of links
    relative to a specified reference link.
    """

    def __init__(self):
        super().__init__('forward_kinematics_solver')
        
        # Declare parameters
        default_urdf_path = os.path.join(get_package_share_directory('rby1_visualizer'), 'rby1_urdf', 'model.urdf')
        self.declare_parameter('urdf_path', default_urdf_path)
        self.declare_parameter('reference_link', 'base')  # Using 'base' as the reference link based on the URDF
        # Default target links based on the robot structure
        default_target_links = [
            'link_right_arm_3',  # Right arm end effector
            'link_left_arm_3',   # Left arm end effector
            'link_head_2',       # Head
            'link_torso_5',      # Torso
            'wheel_r',           # Right wheel
            'wheel_l'            # Left wheel
        ]
        self.declare_parameter('target_links', default_target_links)
        self.declare_parameter('publish_rate', 10.0)  # Hz
        
        # Get parameters
        self.urdf_path = self.get_parameter('urdf_path').value
        self.reference_link = self.get_parameter('reference_link').value
        self.target_links = self.get_parameter('target_links').value
        self.publish_rate = self.get_parameter('publish_rate').value
        
        # Load URDF
        if not self.urdf_path or not os.path.exists(self.urdf_path):
            # Try to get from parameter server if not specified or file doesn't exist
            try:
                self.get_logger().info('Trying to get URDF from parameter server')
                from rclpy.parameter import Parameter
                robot_description = self.get_parameter('robot_description').value
                if not robot_description:
                    self.get_logger().error('No robot_description parameter found')
                    raise ValueError('No robot_description parameter found')
                # Save to temporary file
                import tempfile
                with tempfile.NamedTemporaryFile(delete=False, suffix='.urdf') as f:
                    f.write(robot_description.encode())
                    self.urdf_path = f.name
            except Exception as e:
                # If parameter server fails, try the default path
                default_urdf_path = os.path.join(get_package_share_directory('rby1_visualizer'), 'rby1_urdf', 'model.urdf')
                if os.path.exists(default_urdf_path):
                    self.urdf_path = default_urdf_path
                    self.get_logger().info(f'Using default URDF path: {default_urdf_path}')
                else:
                    self.get_logger().error(f'Failed to get URDF from parameter server: {e}')
                    self.get_logger().error(f'Default URDF path not found: {default_urdf_path}')
                    raise
        
        self.get_logger().info(f'Loading URDF from {self.urdf_path}')
        self.robot = URDF.from_xml_file(self.urdf_path)
        
        # Set up TF listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Create publishers for each target link
        self.pose_publishers = {}
        self.json_publishers = {}
        for link in self.target_links:
            self.pose_publishers[link] = self.create_publisher(
                PoseStamped, 
                f'fk/{link}_pose', 
                10
            )
            self.json_publishers[link] = self.create_publisher(
                String,
                f'fk/{link}_pose_json',
                10
            )
        
        # Create a publisher for all link poses as JSON
        self.all_poses_publisher = self.create_publisher(
            String,
            'fk/all_poses_json',
            10
        )
        
        # Subscribe to joint states
        self.joint_states_sub = self.create_subscription(
            JointState,
            'joint_states',
            self.joint_states_callback,
            10
        )
        
        # Create timer for publishing at a fixed rate
        self.timer = self.create_timer(1.0/self.publish_rate, self.publish_poses)
        
        # Store the latest joint states
        self.latest_joint_states = None
        
        self.get_logger().info('Forward Kinematics Solver initialized')
        self.get_logger().info(f'Reference link: {self.reference_link}')
        self.get_logger().info(f'Target links: {self.target_links}')
    
    def joint_states_callback(self, msg):
        """Store the latest joint states message."""
        self.latest_joint_states = msg
    
    def get_link_pose(self, target_link):
        """
        Get the pose of a target link relative to the reference link using TF2.
        
        Args:
            target_link (str): The name of the target link
            
        Returns:
            geometry_msgs.msg.PoseStamped: The pose of the target link
            relative to the reference link
        """
        if not self.latest_joint_states:
            self.get_logger().warn('No joint states received yet')
            return None
        
        try:
            # Look up the transform from reference link to target link
            transform = self.tf_buffer.lookup_transform(
                self.reference_link,
                target_link,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=1.0)
            )
            
            # Convert transform to pose
            pose = PoseStamped()
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.header.frame_id = self.reference_link
            
            # Set position
            pose.pose.position.x = transform.transform.translation.x
            pose.pose.position.y = transform.transform.translation.y
            pose.pose.position.z = transform.transform.translation.z
            
            # Set orientation
            pose.pose.orientation = transform.transform.rotation
            
            return pose
            
        except TransformException as ex:
            self.get_logger().warn(f'Could not transform {target_link} to {self.reference_link}: {ex}')
            return None
    
    def pose_to_dict(self, pose):
        """Convert a PoseStamped message to a dictionary."""
        if pose is None:
            return None
        
        # Extract position and orientation
        pos = pose.pose.position
        quat = pose.pose.orientation
        
        # Convert quaternion to Euler angles (roll, pitch, yaw)
        quaternion = [quat.x, quat.y, quat.z, quat.w]
        euler = tf_transformations.euler_from_quaternion(quaternion)
        
        return {
            'position': {
                'x': pos.x,
                'y': pos.y,
                'z': pos.z
            },
            'orientation': {
                'x': quat.x,
                'y': quat.y,
                'z': quat.z,
                'w': quat.w
            },
            'euler': {
                'roll': euler[0],
                'pitch': euler[1],
                'yaw': euler[2]
            },
            'frame_id': pose.header.frame_id,
            'timestamp': {
                'sec': pose.header.stamp.sec,
                'nanosec': pose.header.stamp.nanosec
            }
        }
    
    def publish_poses(self):
        """Publish the poses of all target links."""
        if not self.latest_joint_states:
            return
        
        all_poses = {}
        
        for link in self.target_links:
            pose = self.get_link_pose(link)
            if pose:
                # Publish PoseStamped message
                self.pose_publishers[link].publish(pose)
                
                # Convert to dictionary and publish as JSON
                pose_dict = self.pose_to_dict(pose)
                if pose_dict:
                    json_msg = String()
                    json_msg.data = json.dumps(pose_dict)
                    self.json_publishers[link].publish(json_msg)
                    
                    # Add to all poses dictionary
                    all_poses[link] = pose_dict
        
        # Publish all poses as a single JSON message
        if all_poses:
            all_poses_msg = String()
            all_poses_msg.data = json.dumps(all_poses)
            self.all_poses_publisher.publish(all_poses_msg)


def main(args=None):
    rclpy.init(args=args)
    
    fk_solver = ForwardKinematicsSolver()
    
    try:
        rclpy.spin(fk_solver)
    except KeyboardInterrupt:
        pass
    finally:
        fk_solver.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
