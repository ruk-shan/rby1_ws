#!/usr/bin/env python3

"""
ArUco marker detector for Zivid 2+ M60 camera.
This script detects ArUco markers in real-time from the Zivid camera stream.
It displays arrows showing the orientation and publishes marker data as ROS2 topics.
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
from sensor_msgs.msg import Image
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
    # Check if the rotation matrix has a valid determinant
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

class ZividArucoDetectorNode(Node):
    """ROS2 node for detecting ArUco markers with Zivid camera."""
    
    def __init__(self, args):
        super().__init__('aruco_detector_node')
        
        # Initialize the log time dictionary for throttled logging
        self._last_log_time = {}
        
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
        
        # Initialize ArUco detector
        aruco_dict = cv2.aruco.getPredefinedDictionary(ARUCO_DICT[args.dict])
        self.aruco_params = cv2.aruco.DetectorParameters()
        self.detector = cv2.aruco.ArucoDetector(aruco_dict, self.aruco_params)
        
        # Initialize camera matrix and distortion coefficients for Zivid 2+ M60
        # Based on the provided specifications
        self.image_width = 2448
        self.image_height = 2048
        
        # Calculate focal length based on field of view
        # Field of view is 570mm x 460mm at 600mm distance
        # focal_length_x = (image_width * working_distance) / field_of_view_width
        self.fx = (self.image_width * 600) / 570
        self.fy = (self.image_height * 600) / 460
        self.cx = self.image_width / 2.0
        self.cy = self.image_height / 2.0
        
        # Initialize camera matrix with calculated values
        self.camera_matrix = np.array([
            [self.fx, 0, self.cx],
            [0, self.fy, self.cy],
            [0, 0, 1]
        ], dtype=np.float32)
        
        # Initialize distortion coefficients (minimal distortion since factory calibrated)
        self.dist_coeffs = np.zeros((5, 1), dtype=np.float32)
        
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
        self.marker_pub = self.create_publisher(MarkerArray, 'aruco_markers_viz', qos_profile)
        
        # Setup transform broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Create timer for detection loop
        self.timer = self.create_timer(1.0/30.0, self.detect_markers)  # 30 Hz
        
        # Log info
        self.get_logger().info("Zivid ArUco marker detector initialized")
        self.get_logger().info(f"Dictionary: {args.dict}")
        self.get_logger().info(f"Camera Topic: {self.camera_topic}")
        self.get_logger().info(f"Marker Size: {self.marker_size} meters")
        self.get_logger().info(f"Camera Matrix: \n{self.camera_matrix}")
        self.get_logger().info("Press 'q' in the display window to quit")
    
    def _log_throttled(self, log_id, period_seconds, msg):
        """Log a message at INFO level, but throttled to not exceed once per period_seconds."""
        current_time = time.time()
        
        if log_id not in self._last_log_time or \
           (current_time - self._last_log_time[log_id]) > period_seconds:
            self.get_logger().info(msg)
            self._last_log_time[log_id] = current_time
    
    def image_callback(self, msg):
        """Process incoming camera images."""
        try:
            # Convert ROS Image message to OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            self.current_frame = cv_image
            self.frame_available = True
        except CvBridgeError as e:
            self.get_logger().error(f"CvBridge Error: {e}")
    
    def detect_markers(self):
        """Detect ArUco markers in the camera frame and publish data."""
        # Check if we have received a frame from the camera
        if not self.frame_available:
            self._log_throttled("waiting_frames", 5.0, "Waiting for camera frames...")
            return
        
        # Make a copy of the current frame to avoid race conditions
        frame = self.current_frame.copy()
        
        # Convert to grayscale for better detection
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        
        # Detect ArUco markers
        corners, ids, rejected = self.detector.detectMarkers(gray)
        
        # Create dictionary to store marker data for JSON
        markers_dict = {}
        
        # Create pose array message
        pose_array_msg = PoseArray()
        pose_array_msg.header.frame_id = "link_zivid"
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
                obj_points = np.array([
                    [-self.marker_size/2, self.marker_size/2, 0],
                    [self.marker_size/2, self.marker_size/2, 0],
                    [self.marker_size/2, -self.marker_size/2, 0],
                    [-self.marker_size/2, -self.marker_size/2, 0]
                ], dtype=np.float32)
                
                # Use solvePnP for pose estimation
                ret, rvec, tvec = cv2.solvePnP(
                    obj_points,
                    marker_corners.astype(np.float32).reshape(4, 2),
                    self.camera_matrix,
                    self.dist_coeffs,
                    flags=cv2.SOLVEPNP_IPPE_SQUARE
                )
                
                if not ret:
                    self.get_logger().warn(f'Failed to estimate pose for marker {marker_id}')
                    continue
                
                # Ensure rvec and tvec are in the correct shape (3,1)
                rvec = np.array(rvec, dtype=np.float32).reshape(3, 1)
                tvec = np.array(tvec, dtype=np.float32).reshape(3, 1)
                
                # Store vectors for visualization (make a copy to avoid reference issues)
                all_rvecs.append(rvec.copy())
                all_tvecs.append(tvec.copy())
                
                # Draw the axis of the marker
                try:
                    cv2.drawFrameAxes(frame, self.camera_matrix, self.dist_coeffs, rvec, tvec, self.axis_length)
                except Exception as e:
                    self.get_logger().error(f'Error drawing axes: {e}')
                
                # Get the center of the marker
                center = np.mean(marker_corners[0], axis=0).astype(int)
                
                # Draw the marker ID
                cv2.putText(frame, f"ID:{marker_id}", 
                           (center[0], center[1] - 15), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                
                # Convert rotation vector to Euler angles (in degrees)
                try:
                    rot_matrix, _ = cv2.Rodrigues(rvec)
                    euler_angles = rotation_matrix_to_euler_angles(rot_matrix) * 180 / np.pi
                except Exception as e:
                    self.get_logger().error(f'Error processing rotation for marker {marker_id}: {e}')
                    continue
                
                # Store marker data in dictionary (convert numpy values to Python types for JSON)
                markers_dict[str(marker_id)] = {
                    "position": {
                        "x": float(tvec[0][0]),
                        "y": float(tvec[1][0]),
                        "z": float(tvec[2][0])
                    },
                    "rotation": {
                        "roll": float(euler_angles[0]),
                        "pitch": float(euler_angles[1]),
                        "yaw": float(euler_angles[2])
                    },
                    "timestamp": time.time()
                }
                
                # Add to pose array message
                pose = Pose()
                pose.position.x = float(tvec[0][0])
                pose.position.y = float(tvec[1][0])
                pose.position.z = float(tvec[2][0])
                
                # Convert rotation matrix to quaternion
                r = Rotation.from_matrix(rot_matrix)
                q = r.as_quat()
                
                pose.orientation.x = float(q[0])
                pose.orientation.y = float(q[1])
                pose.orientation.z = float(q[2])
                pose.orientation.w = float(q[3])
                
                pose_array_msg.poses.append(pose)
                
                # Publish transform for this marker
                transform = TransformStamped()
                transform.header.stamp = self.get_clock().now().to_msg()
                transform.header.frame_id = "link_zivid"
                transform.child_frame_id = f"aruco_marker_{marker_id}"
                
                # Set translation
                transform.transform.translation.x = float(tvec[0][0])
                transform.transform.translation.y = float(tvec[1][0])
                transform.transform.translation.z = float(tvec[2][0])
                
                # Set rotation as quaternion
                transform.transform.rotation.x = float(q[0])
                transform.transform.rotation.y = float(q[1])
                transform.transform.rotation.z = float(q[2])
                transform.transform.rotation.w = float(q[3])
                
                # Broadcast transform
                self.tf_broadcaster.sendTransform(transform)
            
            # Publish marker data as JSON
            json_msg = String()
            json_msg.data = json.dumps(markers_dict)
            self.json_pub.publish(json_msg)
            
            # Publish pose array
            self.pose_array_pub.publish(pose_array_msg)
            
            # Publish visualization markers for RViz2
            if len(all_tvecs) > 0:
                self.publish_visualization_markers(corners, ids, all_tvecs, all_rvecs)
        
        # Display the frame if requested
        if self.display:
            cv2.imshow('ArUco Marker Detection', frame)
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                self.shutdown()
                sys.exit(0)
    
    def publish_visualization_markers(self, corners, ids, tvecs, rvecs):
        """Create and publish visualization markers for RViz2."""
        now = self.get_clock().now().to_msg()
        marker_array = MarkerArray()
        
        # Define colors for axes
        colors = [
            ColorRGBA(r=1.0, g=0.0, b=0.0, a=1.0),  # Red for X-axis
            ColorRGBA(r=0.0, g=1.0, b=0.0, a=1.0),  # Green for Y-axis
            ColorRGBA(r=0.0, g=0.0, b=1.0, a=1.0)   # Blue for Z-axis
        ]
        
        axis_names = ["x", "y", "z"]
        
        # Create a marker for each detected ArUco marker
        for i, marker_id in enumerate(ids):
            # Create cube marker
            marker = Marker()
            marker.header.frame_id = "link_zivid"
            marker.header.stamp = now
            marker.ns = "aruco_markers"
            marker.id = int(marker_id[0])
            marker.type = Marker.CUBE
            marker.action = Marker.ADD
            
            # Set position from translation vector
            marker.pose.position.x = float(tvecs[i][0][0])
            marker.pose.position.y = float(tvecs[i][1][0])
            marker.pose.position.z = float(tvecs[i][2][0])
            
            # Set orientation from rotation vector
            try:
                rot_matrix, _ = cv2.Rodrigues(rvecs[i])
                euler_angles = rotation_matrix_to_euler_angles(rot_matrix)
            except Exception as e:
                self.get_logger().error(f'Error processing rotation for marker {marker_id}: {e}')
                continue
            
            # Convert rotation matrix to quaternion
            r = Rotation.from_matrix(rot_matrix)
            q = r.as_quat()
            
            marker.pose.orientation.x = float(q[0])
            marker.pose.orientation.y = float(q[1])
            marker.pose.orientation.z = float(q[2])
            marker.pose.orientation.w = float(q[3])
            
            # Set scale (slightly smaller than actual marker)
            marker.scale.x = self.marker_size * 0.8
            marker.scale.y = self.marker_size * 0.8
            marker.scale.z = self.marker_size * 0.1
            
            # Set color (semi-transparent white)
            marker.color.r = 1.0
            marker.color.g = 1.0
            marker.color.b = 1.0
            marker.color.a = 0.5
            
            marker_array.markers.append(marker)
            
            # Create a text marker for the ID
            text_marker = Marker()
            text_marker.header.frame_id = "link_zivid"
            text_marker.header.stamp = now
            text_marker.ns = "aruco_marker_texts"
            text_marker.id = int(marker_id[0]) + 1000  # Offset to avoid ID collision
            text_marker.type = Marker.TEXT_VIEW_FACING
            text_marker.action = Marker.ADD
            
            # Position text above the marker
            text_marker.pose.position.x = float(tvecs[i][0][0])
            text_marker.pose.position.y = float(tvecs[i][1][0])
            text_marker.pose.position.z = float(tvecs[i][2][0]) + self.marker_size * 0.6
            
            # Set text
            text_marker.text = f"ID:{int(marker_id[0])}"
            
            # Set scale (text height in meters)
            text_marker.scale.z = self.marker_size * 0.6
            
            # Set color (white)
            text_marker.color.r = 1.0
            text_marker.color.g = 1.0
            text_marker.color.b = 1.0
            text_marker.color.a = 1.0
            
            marker_array.markers.append(text_marker)
            
            # Create arrow markers for each axis
            axes = [
                np.array([self.axis_length, 0, 0]),
                np.array([0, self.axis_length, 0]),
                np.array([0, 0, self.axis_length])
            ]
            
            for axis_idx, (axis, color, name) in enumerate(zip(axes, colors, axis_names)):
                arrow_marker = Marker()
                arrow_marker.header.frame_id = "link_zivid"
                arrow_marker.header.stamp = now
                arrow_marker.ns = f"aruco_marker_{name}_axis"
                arrow_marker.id = int(marker_id[0]) * 10 + axis_idx  # Unique ID for each axis
                arrow_marker.type = Marker.ARROW
                arrow_marker.action = Marker.ADD
                
                # Start point at marker center
                arrow_marker.points.append(Point(
                    x=float(tvecs[i][0][0]),
                    y=float(tvecs[i][1][0]),
                    z=float(tvecs[i][2][0])
                ))
                
                # Transform axis vector by rotation matrix
                axis_transformed = np.dot(rot_matrix, axis)
                
                # End point
                arrow_marker.points.append(Point(
                    x=float(tvecs[i][0][0] + axis_transformed[0]),
                    y=float(tvecs[i][1][0] + axis_transformed[1]),
                    z=float(tvecs[i][2][0] + axis_transformed[2])
                ))
                
                # Set scale (shaft diameter and head size)
                arrow_marker.scale.x = self.marker_size * 0.1  # Shaft diameter
                arrow_marker.scale.y = self.marker_size * 0.2  # Head diameter
                arrow_marker.scale.z = self.marker_size * 0.3  # Head length
                
                # Set color
                arrow_marker.color = color
                
                marker_array.markers.append(arrow_marker)
        
        # Publish marker array
        self.marker_pub.publish(marker_array)
    
    def shutdown(self):
        """Release resources when the node is shutdown."""
        if self.display:
            cv2.destroyAllWindows()
        self.get_logger().info("Resources released")

def main():
    # Parse command-line arguments
    parser = argparse.ArgumentParser(description='ArUco marker detector for Zivid camera')
    parser.add_argument('--camera-topic', type=str, default='/camera/color/image_raw', 
                        help='ROS2 camera topic to subscribe to (default: /camera/color/image_raw)')
    parser.add_argument('--dict', type=str, default='DICT_4X4_50', 
                        choices=ARUCO_DICT.keys(), help='ArUco dictionary to use')
    parser.add_argument('--marker-size', type=float, default=0.05, 
                        help='Size of the ArUco marker in meters')
    parser.add_argument('--display', action='store_true', 
                        help='Display the camera feed with detected markers')
    
    args = parser.parse_args()
    
    # Initialize ROS2
    rclpy.init()
    
    # Create and run the node
    node = ZividArucoDetectorNode(args)
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
