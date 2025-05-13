#!/usr/bin/env python3

"""
Simple ArUco marker detector using OpenCV.
This script captures video from a USB camera and detects ArUco markers in real-time.
It displays arrows showing the orientation and prints pose information.
It also publishes marker data as a ROS2 topic in JSON format.
"""

import cv2
import numpy as np
import argparse
import sys
import time
import math
import json
from scipy.spatial.transform import Rotation

# ROS2 imports
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, ColorRGBA
from geometry_msgs.msg import PoseArray, Pose, Point, Vector3
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
from visualization_msgs.msg import Marker, MarkerArray
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy, QoSHistoryPolicy
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge, CvBridgeError

# Dictionary of available ArUco dictionaries in OpenCV
ARUCO_DICT = {
    "DICT_4X4_50": cv2.aruco.DICT_4X4_50,
    "DICT_4X4_100": cv2.aruco.DICT_4X4_100,
    "DICT_4X4_250": cv2.aruco.DICT_4X4_250,
    "DICT_4X4_1000": cv2.aruco.DICT_4X4_1000,
    "DICT_5X5_50": cv2.aruco.DICT_5X5_50,
    "DICT_5X5_100": cv2.aruco.DICT_5X5_100,
    "DICT_5X5_250": cv2.aruco.DICT_5X5_250,
    "DICT_5X5_1000": cv2.aruco.DICT_5X5_1000,
    "DICT_6X6_50": cv2.aruco.DICT_6X6_50,
    "DICT_6X6_100": cv2.aruco.DICT_6X6_100,
    "DICT_6X6_250": cv2.aruco.DICT_6X6_250,
    "DICT_6X6_1000": cv2.aruco.DICT_6X6_1000,
    "DICT_7X7_50": cv2.aruco.DICT_7X7_50,
    "DICT_7X7_100": cv2.aruco.DICT_7X7_100,
    "DICT_7X7_250": cv2.aruco.DICT_7X7_250,
    "DICT_7X7_1000": cv2.aruco.DICT_7X7_1000,
    "DICT_ARUCO_ORIGINAL": cv2.aruco.DICT_ARUCO_ORIGINAL,
}

def rotation_matrix_to_euler_angles(R):
    """Convert a rotation matrix to Euler angles (roll, pitch, yaw)."""
    # Check if the rotation matrix is valid
    sy = math.sqrt(R[0, 0] * R[0, 0] + R[1, 0] * R[1, 0])
    
    # Regular case
    if sy > 1e-6:
        roll = math.atan2(R[2, 1], R[2, 2])
        pitch = math.atan2(-R[2, 0], sy)
        yaw = math.atan2(R[1, 0], R[0, 0])
    # Gimbal lock case
    else:
        roll = math.atan2(-R[1, 2], R[1, 1])
        pitch = math.atan2(-R[2, 0], sy)
        yaw = 0
    
    return np.array([roll, pitch, yaw])

class ArUcoDetectorNode(Node):
    """ROS2 node for detecting ArUco markers."""
    
    def __init__(self, args):
        super().__init__('aruco_detector_node')
        
        # Initialize throttle timestamps for logging
        self._last_throttle_timestamps = {}
        
        # Store arguments
        self.args = args
        self.marker_size = args.marker_size
        self.display = args.display
        self.camera_topic = args.camera_topic
        
        # Initialize bridge for converting ROS images to OpenCV format
        self.bridge = CvBridge()
        
        # Latest camera frame
        self.current_frame = None
        self.frame_available = False
        
        # Subscribe to camera topic
        self.image_subscription = self.create_subscription(
            Image,
            self.camera_topic,
            self.image_callback,
            10)
        
        # Subscribe to camera info topic for calibration parameters
        camera_info_topic = self.camera_topic.rsplit('/', 1)[0] + '/camera_info'
        self.get_logger().info(f"Subscribing to camera info topic: {camera_info_topic}")
        self.camera_info_subscription = self.create_subscription(
            CameraInfo,
            camera_info_topic,
            self.camera_info_callback,
            10)
        
        # Initialize ArUco detector
        aruco_dict = cv2.aruco.getPredefinedDictionary(ARUCO_DICT[args.dict])
        self.aruco_params = cv2.aruco.DetectorParameters()
        self.detector = cv2.aruco.ArucoDetector(aruco_dict, self.aruco_params)
        
        # Initialize camera matrix and distortion coefficients with defaults
        # These will be updated when camera_info is received
        self.camera_matrix = None
        self.dist_coeffs = None
        self.camera_info_received = False
        
        # Default image dimensions (will be updated when first image is received)
        self.image_width = 640
        self.image_height = 480
        
        # Define axis for pose visualization
        self.axis_length = self.marker_size * 2  # Length of the axis arrows
        
        # Create publishers
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # Publisher for JSON marker data
        self.json_pub = self.create_publisher(String, 'aruco_markers_json', qos_profile)
        
        # Publisher for PoseArray (standard ROS2 message)
        self.pose_array_pub = self.create_publisher(PoseArray, 'aruco_markers_poses', qos_profile)
        
        # Publisher for visualization markers (for RViz2)
        self.marker_pub = self.create_publisher(MarkerArray, 'aruco_markers_visual', qos_profile)
        
        # Setup transform broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Create timer for detection loop
        self.timer = self.create_timer(1.0/30.0, self.detect_markers)  # 30 Hz
        
        # Log info
        self.get_logger().info("ArUco marker detector initialized")
        self.get_logger().info(f"Dictionary: {args.dict}")
        self.get_logger().info(f"Camera Topic: {self.camera_topic}")
        self.get_logger().info(f"Marker Size: {self.marker_size} meters")
        self.get_logger().info("Press 'q' in the display window to quit")
    
    def info_throttle(self, period_seconds, msg):
        """Log info messages at a throttled rate."""
        current_time = self.get_clock().now().to_msg().sec
        key = msg  # Use message as the key
        
        if key not in self._last_throttle_timestamps or \
           current_time - self._last_throttle_timestamps[key] >= period_seconds:
            self.get_logger().info(msg)
            self._last_throttle_timestamps[key] = current_time
    
    def image_callback(self, msg):
        """Process incoming camera images."""
        try:
            # Convert ROS Image message to OpenCV image
            self.current_frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            
            # Update image dimensions
            if self.current_frame is not None:
                self.image_height, self.image_width = self.current_frame.shape[:2]
                self.frame_available = True
                
                # If we don't have camera calibration yet, use default values based on image size
                if not self.camera_info_received:
                    self.get_logger().info("Using default camera calibration based on image size")
                    focal_length = self.image_width
                    center = [self.image_width / 2, self.image_height / 2]
                    self.camera_matrix = np.array(
                        [[focal_length, 0, center[0]],
                         [0, focal_length, center[1]],
                         [0, 0, 1]], dtype=np.float32
                    )
                    self.dist_coeffs = np.zeros((4, 1))
                    
        except CvBridgeError as e:
            self.get_logger().error(f"CvBridge Error: {e}")
    
    def camera_info_callback(self, msg):
        """Process camera calibration information."""
        if not self.camera_info_received:
            self.get_logger().info("Received camera calibration information")
            
            # Extract camera matrix (K) and distortion coefficients (D)
            self.camera_matrix = np.array(msg.k).reshape(3, 3)
            self.dist_coeffs = np.array(msg.d)
            
            self.camera_info_received = True
            
            # Log camera calibration information
            self.get_logger().info(f"Camera Matrix:\n{self.camera_matrix}")
            self.get_logger().info(f"Distortion Coefficients: {self.dist_coeffs}")
    
    def detect_markers(self):
        """Detect ArUco markers in the camera frame and publish data."""
        # Check if we have received a frame from the camera
        if not self.frame_available:
            self.get_logger().info_throttle(5.0, "Waiting for camera frames...")
            return
        
        # Make a copy of the current frame to avoid race conditions
        frame = self.current_frame.copy()
        
        # Check if camera matrix is available
        if self.camera_matrix is None:
            self.get_logger().info_throttle(5.0, "Waiting for camera calibration...")
            return
        
        # Convert to grayscale for better detection
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        
        # Detect ArUco markers
        corners, ids, rejected = self.detector.detectMarkers(gray)
        
        # Create dictionary to store marker data for JSON
        markers_dict = {}
        
        # Create pose array message
        pose_array_msg = PoseArray()
        pose_array_msg.header.frame_id = "camera"
        pose_array_msg.header.stamp = self.get_clock().now().to_msg()
        
        # Lists to store all poses for visualization
        all_tvecs = []
        all_rvecs = []
        
        # Draw detected markers and IDs
        if ids is not None:
            # Draw markers
            cv2.aruco.drawDetectedMarkers(frame, corners, ids)
            
            # For each marker, estimate pose and draw axis
            for i, marker_corners in enumerate(corners):
                marker_id = int(ids[i][0])  # Convert to int for JSON serialization
                
                # Estimate pose of the marker
                rvec, tvec, _ = cv2.aruco.estimatePoseSingleMarkers(
                    marker_corners, self.marker_size, self.camera_matrix, self.dist_coeffs
                )
                
                # Store vectors for visualization
                all_rvecs.append(rvec)
                all_tvecs.append(tvec)
                
                # Draw axis for the marker
                cv2.drawFrameAxes(frame, self.camera_matrix, self.dist_coeffs, rvec, tvec, self.axis_length)
                
                # Get the center of the marker
                center = np.mean(marker_corners[0], axis=0).astype(int)
                
                # Draw the marker ID
                cv2.putText(frame, f"ID:{marker_id}", 
                           (center[0], center[1] - 15), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                
                # Convert rotation vector to Euler angles (in degrees)
                rot_matrix, _ = cv2.Rodrigues(rvec[0])
                euler_angles = rotation_matrix_to_euler_angles(rot_matrix) * 180 / np.pi
                
                # Store marker data in dictionary (convert numpy values to Python types for JSON)
                markers_dict[str(marker_id)] = {
                    "position": {
                        "x": float(tvec[0][0][0]),
                        "y": float(tvec[0][0][1]),
                        "z": float(tvec[0][0][2])
                    },
                    "rotation": {
                        "roll": float(euler_angles[0]),
                        "pitch": float(euler_angles[1]),
                        "yaw": float(euler_angles[2])
                    }
                }
                
                # Add to pose array message
                pose = Pose()
                pose.position.x = float(tvec[0][0][0])
                pose.position.y = float(tvec[0][0][1])
                pose.position.z = float(tvec[0][0][2])
                
                # Convert rotation matrix to quaternion
                r = Rotation.from_matrix(rot_matrix)
                quat = r.as_quat()  # Returns x, y, z, w
                
                # Set orientation as quaternion
                pose.orientation.x = float(quat[0])
                pose.orientation.y = float(quat[1])
                pose.orientation.z = float(quat[2])
                pose.orientation.w = float(quat[3])
                
                pose_array_msg.poses.append(pose)
                
                # Publish transform for this marker
                transform = TransformStamped()
                transform.header.stamp = self.get_clock().now().to_msg()
                transform.header.frame_id = "camera"
                transform.child_frame_id = f"aruco_marker_{marker_id}"
                
                # Set translation
                transform.transform.translation.x = float(tvec[0][0][0])
                transform.transform.translation.y = float(tvec[0][0][1])
                transform.transform.translation.z = float(tvec[0][0][2])
                
                # Convert rotation matrix to quaternion
                r = Rotation.from_matrix(rot_matrix)
                quat = r.as_quat()  # Returns x, y, z, w
                
                # Set rotation as quaternion
                transform.transform.rotation.x = float(quat[0])
                transform.transform.rotation.y = float(quat[1])
                transform.transform.rotation.z = float(quat[2])
                transform.transform.rotation.w = float(quat[3])
                
                # Broadcast transform
                self.tf_broadcaster.sendTransform(transform)
                
                # Print pose information
                self.get_logger().info(f"Marker ID:{marker_id}")
                self.get_logger().info(f"Position (x,y,z): {tvec[0][0][0]:.3f}, {tvec[0][0][1]:.3f}, {tvec[0][0][2]:.3f} meters")
                self.get_logger().info(f"Rotation (roll,pitch,yaw): {euler_angles[0]:.1f}, {euler_angles[1]:.1f}, {euler_angles[2]:.1f} degrees")
                
                # Draw position text on frame
                pos_text = f"Pos:{tvec[0][0][0]:.2f}, {tvec[0][0][1]:.2f}, {tvec[0][0][2]:.2f}m"
                cv2.putText(frame, pos_text, 
                           (center[0], center[1] + 15), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 0, 0), 1)
                
                # Draw rotation text on frame
                rot_text = f"Rot:{euler_angles[0]:.1f}, {euler_angles[1]:.1f}, {euler_angles[2]:.1f}\u00b0"
                cv2.putText(frame, rot_text, 
                           (center[0], center[1] + 35), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 0, 0), 1)
        
        # Publish JSON data if markers were detected
        if markers_dict:
            # Convert dictionary to JSON string
            json_str = json.dumps(markers_dict)
            
            # Create ROS2 String message
            msg = String()
            msg.data = json_str
            
            # Publish JSON message
            self.json_pub.publish(msg)
            
            # Publish pose array
            self.pose_array_pub.publish(pose_array_msg)
            
            # Create and publish visualization markers for RViz2
            self.publish_visualization_markers(corners, ids, tvecs=all_tvecs, rvecs=all_rvecs)
        
        # Add FPS counter
        fps = 30  # Assume 30 FPS for ROS2 camera topics
        cv2.putText(frame, f"FPS: {fps}", (10, 30), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
        
        # Display the result
        if self.display:
            cv2.imshow('ArUco Marker Detection', frame)
            
            # Check if 'q' is pressed
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                self.get_logger().info("Shutting down...")
                rclpy.shutdown()
    
    def publish_visualization_markers(self, corners, ids, tvecs, rvecs):
        """Create and publish visualization markers for RViz2."""
        marker_array = MarkerArray()
        now = self.get_clock().now().to_msg()
        
        # Create a cube marker for each detected ArUco marker
        for i, marker_id in enumerate(ids):
            # Create cube marker
            marker = Marker()
            marker.header.frame_id = "camera"
            marker.header.stamp = now
            marker.ns = "aruco_markers"
            marker.id = int(marker_id[0])
            marker.type = Marker.CUBE
            marker.action = Marker.ADD
            
            # Set marker position from translation vector
            marker.pose.position.x = float(tvecs[i][0][0][0])
            marker.pose.position.y = float(tvecs[i][0][0][1])
            marker.pose.position.z = float(tvecs[i][0][0][2])
            
            # Convert rotation vector to rotation matrix
            rot_matrix, _ = cv2.Rodrigues(rvecs[i][0])
            
            # Convert rotation matrix to quaternion
            r = Rotation.from_matrix(rot_matrix)
            quat = r.as_quat()  # Returns x, y, z, w
            
            # Set marker orientation as quaternion
            marker.pose.orientation.x = float(quat[0])
            marker.pose.orientation.y = float(quat[1])
            marker.pose.orientation.z = float(quat[2])
            marker.pose.orientation.w = float(quat[3])
            
            # Set marker scale - slightly larger than the actual marker
            marker.scale.x = self.marker_size * 1.2
            marker.scale.y = self.marker_size * 1.2
            marker.scale.z = 0.01  # Very thin in Z direction
            
            # Set marker color - different color for each marker
            marker.color.r = (marker_id[0] * 40 % 255) / 255.0
            marker.color.g = (marker_id[0] * 20 % 255) / 255.0
            marker.color.b = (marker_id[0] * 60 % 255) / 255.0
            marker.color.a = 0.7  # Semi-transparent
            
            # Add to marker array
            marker_array.markers.append(marker)
            
            # Create a text marker for the ID
            text_marker = Marker()
            text_marker.header.frame_id = "camera"
            text_marker.header.stamp = now
            text_marker.ns = "aruco_marker_texts"
            text_marker.id = int(marker_id[0]) + 1000  # Offset to avoid ID collision
            text_marker.type = Marker.TEXT_VIEW_FACING
            text_marker.action = Marker.ADD
            
            # Position text slightly above the marker
            text_marker.pose.position.x = float(tvecs[i][0][0][0])
            text_marker.pose.position.y = float(tvecs[i][0][0][1])
            text_marker.pose.position.z = float(tvecs[i][0][0][2]) + self.marker_size * 0.7
            
            # Set text marker properties
            text_marker.text = f"ID:{marker_id[0]}"
            text_marker.scale.z = 0.05  # Text height
            text_marker.color.r = 1.0
            text_marker.color.g = 1.0
            text_marker.color.b = 1.0
            text_marker.color.a = 1.0
            
            # Add to marker array
            marker_array.markers.append(text_marker)
            
            # Create axis markers (arrows for x, y, z)
            colors = [(1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0)]  # RGB for XYZ
            axes = [(self.marker_size, 0, 0), (0, self.marker_size, 0), (0, 0, self.marker_size)]  # X, Y, Z axes
            axis_names = ['X', 'Y', 'Z']
            
            for axis_idx, (axis, color, name) in enumerate(zip(axes, colors, axis_names)):
                arrow_marker = Marker()
                arrow_marker.header.frame_id = "camera"
                arrow_marker.header.stamp = now
                arrow_marker.ns = f"aruco_marker_{name}_axis"
                arrow_marker.id = int(marker_id[0]) * 10 + axis_idx  # Unique ID for each axis
                arrow_marker.type = Marker.ARROW
                arrow_marker.action = Marker.ADD
                
                # Position arrow at marker position
                arrow_marker.pose.position.x = float(tvecs[i][0][0][0])
                arrow_marker.pose.position.y = float(tvecs[i][0][0][1])
                arrow_marker.pose.position.z = float(tvecs[i][0][0][2])
                
                # Set arrow orientation from rotation matrix
                arrow_marker.pose.orientation.x = float(quat[0])
                arrow_marker.pose.orientation.y = float(quat[1])
                arrow_marker.pose.orientation.z = float(quat[2])
                arrow_marker.pose.orientation.w = float(quat[3])
                
                # Set arrow points - these are relative to the marker's coordinate frame
                arrow_marker.points.append(Point(x=0.0, y=0.0, z=0.0))
                arrow_marker.points.append(Point(x=float(axis[0]), y=float(axis[1]), z=float(axis[2])))
                
                # Set arrow scale
                arrow_marker.scale.x = 0.01  # Shaft diameter
                arrow_marker.scale.y = 0.02  # Head diameter
                arrow_marker.scale.z = 0.03  # Head length
                
                # Set arrow color
                arrow_marker.color.r = float(color[0])
                arrow_marker.color.g = float(color[1])
                arrow_marker.color.b = float(color[2])
                arrow_marker.color.a = 1.0
                
                # Add to marker array
                marker_array.markers.append(arrow_marker)
        
        # Publish marker array
        self.marker_pub.publish(marker_array)
    
    def shutdown(self):
        """Release resources when the node is shutdown."""
        cv2.destroyAllWindows()
        self.get_logger().info("Resources released")


def main():
    # Parse command-line arguments
    parser = argparse.ArgumentParser(description='ArUco marker detector with ROS2 publishing')
    parser.add_argument('--camera-topic', type=str, default='/camera/image_raw', 
                        help='ROS2 camera topic to subscribe to (default: /camera/image_raw)')
    parser.add_argument('--dict', type=str, default='DICT_4X4_50', 
                        choices=ARUCO_DICT.keys(), help='ArUco dictionary to use')
    parser.add_argument('--display', action='store_true', help='Display detection results')
    parser.add_argument('--marker-size', type=float, default=0.05,
                        help='Size of the ArUco marker in meters (default: 0.05)')
    args = parser.parse_args()
    
    # Initialize ROS2
    rclpy.init(args=None)
    
    # Create and run the node
    node = ArUcoDetectorNode(args)
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\nDetection stopped by user")
    except Exception as e:
        print(f"Error: {e}")
    finally:
        # Cleanup
        node.shutdown()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
