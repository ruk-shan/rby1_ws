#!/usr/bin/env python3

# Copyright 2023
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""
ROS2 Node for ArUco marker detection with pose estimation.
"""

import sys
import time
import numpy as np
import cv2
from cv_bridge import CvBridge

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rcl_interfaces.msg import ParameterDescriptor, ParameterType
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Pose, PoseArray
from ros2_aruco_interfaces.msg import ArucoMarkers
from rclpy.qos import qos_profile_sensor_data, QoSProfile
from rclpy.qos import QoSReliabilityPolicy, QoSDurabilityPolicy, QoSHistoryPolicy

# The different ArUco dictionaries built into OpenCV
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


class ArucoPoseNode(Node):
    """
    ROS2 Node for detecting ArUco markers and estimating their 3D poses.
    """

    def __init__(self):
        super().__init__("aruco_pose_node")

        # Declare and get parameters
        self.declare_parameter(
            name="aruco_dictionary_id",
            value="DICT_4X4_50",
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_STRING,
                description="Dictionary that was used to generate markers.",
            ),
        )
        self.declare_parameter(
            name="image_topic",
            value="/image_raw",
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_STRING,
                description="Image topic to subscribe to.",
            ),
        )
        self.declare_parameter(
            name="camera_info_topic",
            value="/camera_info",
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_STRING,
                description="Camera info topic to subscribe to.",
            ),
        )
        self.declare_parameter(
            name="marker_size",
            value=0.05,  # 5cm default
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_DOUBLE,
                description="Size of the markers in meters.",
            ),
        )
        self.declare_parameter(
            name="max_fps",
            value=30.0,
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_DOUBLE,
                description="Maximum processing rate in frames per second.",
            ),
        )

        # Get parameters
        dictionary_id_name = self.get_parameter("aruco_dictionary_id").get_parameter_value().string_value
        image_topic = self.get_parameter("image_topic").get_parameter_value().string_value
        camera_info_topic = self.get_parameter("camera_info_topic").get_parameter_value().string_value
        self.marker_size = self.get_parameter("marker_size").get_parameter_value().double_value
        self.max_fps = self.get_parameter("max_fps").get_parameter_value().double_value

        # Log parameters
        self.get_logger().info(f"Marker type: {dictionary_id_name}")
        self.get_logger().info(f"Image topic: {image_topic}")
        self.get_logger().info(f"Camera info topic: {camera_info_topic}")
        self.get_logger().info(f"Marker size: {self.marker_size} meters")
        self.get_logger().info(f"Max FPS: {self.max_fps}")

        # Check if the specified dictionary is valid
        if dictionary_id_name not in ARUCO_DICT:
            self.get_logger().error(f"Invalid ArUco dictionary: {dictionary_id_name}")
            self.get_logger().error(f"Valid options: {', '.join(ARUCO_DICT.keys())}")
            return

        # Set up QoS profiles
        sensor_qos = QoSProfile(
            depth=1,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
        )

        reliable_qos = QoSProfile(
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
        )

        # Set up subscriptions and publishers
        self.create_subscription(Image, image_topic, self.image_callback, sensor_qos)
        self.create_subscription(CameraInfo, camera_info_topic, self.camera_info_callback, reliable_qos)
        self.markers_pub = self.create_publisher(ArucoMarkers, "aruco_markers", reliable_qos)
        self.poses_pub = self.create_publisher(PoseArray, "aruco_poses", reliable_qos)
        self.image_pub = self.create_publisher(Image, "aruco_detections", sensor_qos)

        # Store dictionary name and initialize detector with optimized parameters
        self.dictionary_id_name = dictionary_id_name
        self.aruco_dictionary = cv2.aruco.getPredefinedDictionary(ARUCO_DICT[dictionary_id_name])
        self.aruco_parameters = cv2.aruco.DetectorParameters()
        
        # Optimize detection parameters
        self.aruco_parameters.adaptiveThreshConstant = 7
        self.aruco_parameters.adaptiveThreshWinSizeMin = 3
        self.aruco_parameters.adaptiveThreshWinSizeMax = 23
        self.aruco_parameters.adaptiveThreshWinSizeStep = 10
        self.aruco_parameters.minMarkerPerimeterRate = 0.03
        self.aruco_parameters.maxMarkerPerimeterRate = 0.3
        self.aruco_parameters.polygonalApproxAccuracyRate = 0.05
        self.aruco_parameters.minCornerDistanceRate = 0.05
        self.aruco_parameters.minMarkerDistanceRate = 0.05
        self.aruco_parameters.minDistanceToBorder = 3
        
        self.detector = cv2.aruco.ArucoDetector(self.aruco_dictionary, self.aruco_parameters)
        
        # Initialize utilities
        self.bridge = CvBridge()
        self.last_process_time = self.get_clock().now()
        
        # Camera calibration parameters
        self.camera_matrix = None
        self.dist_coeffs = None
        self.camera_info_received = False

    def camera_info_callback(self, camera_info_msg):
        """Process camera calibration information."""
        if not self.camera_info_received:
            # Extract camera matrix and distortion coefficients
            self.camera_matrix = np.array(camera_info_msg.k).reshape(3, 3)
            self.dist_coeffs = np.array(camera_info_msg.d)
            self.camera_info_received = True
            self.get_logger().info("Camera calibration information received")
            self.get_logger().info(f"Camera Matrix:\n{self.camera_matrix}")
            self.get_logger().info(f"Distortion Coefficients: {self.dist_coeffs}")

    def image_callback(self, img_msg):
        """Process incoming images and detect ArUco markers."""
        # Rate limiting
        now = self.get_clock().now()
        if (now - self.last_process_time).nanoseconds < (1e9 / self.max_fps):
            return
        self.last_process_time = now

        # Convert ROS Image message to OpenCV image
        try:
            cv_image = self.bridge.imgmsg_to_cv2(img_msg, "bgr8")
            self.get_logger().debug(f'Received image: {cv_image.shape}')
        except Exception as e:
            self.get_logger().error(f'Could not convert image: {str(e)}')
            return

        # Initialize the output messages
        markers = ArucoMarkers()
        markers.header = img_msg.header
        poses = PoseArray()
        poses.header = img_msg.header

        # Preprocess image
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        gray = cv2.GaussianBlur(gray, (5, 5), 0)
        
        # Detect ArUco markers in the image
        corners, marker_ids, _ = self.detector.detectMarkers(cv_image)
        if marker_ids is None:  # If no markers found, try preprocessed image
            corners, marker_ids, _ = self.detector.detectMarkers(gray)
            
        # Filter out false positives
        if marker_ids is not None and len(marker_ids) > 0:
            valid_indices = []
            for i, corner in enumerate(corners):
                # Calculate perimeter of marker
                perimeter = cv2.arcLength(corner, True)
                # Calculate area
                area = cv2.contourArea(corner)
                # Calculate aspect ratio
                x, y, w, h = cv2.boundingRect(corner)
                aspect_ratio = float(w) / h if h > 0 else 0
                
                # Validate marker based on geometric properties
                if perimeter > 40 and area > 100 and 0.7 < aspect_ratio < 1.3:
                    valid_indices.append(i)
                else:
                    self.get_logger().debug(f'Filtered out marker {marker_ids[i][0]} - perimeter: {perimeter:.1f}, area: {area:.1f}, aspect: {aspect_ratio:.2f}')
            
            if valid_indices:
                corners = [corners[i] for i in valid_indices]
                marker_ids = np.array([marker_ids[i] for i in valid_indices])
            else:
                corners = []
                marker_ids = None

        # Create debug image
        debug_image = cv_image.copy()
        
        # Add debug info
        cv2.putText(debug_image, f'Dictionary: {self.dictionary_id_name}', (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        cv2.putText(debug_image, f'Image size: {cv_image.shape[1]}x{cv_image.shape[0]}', (10, 60),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        
        # Show detection status
        if marker_ids is not None and len(marker_ids) > 0:
            cv2.putText(debug_image, f'Status: {len(marker_ids)} markers found', (10, 90),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        else:
            cv2.putText(debug_image, 'Status: No valid markers found', (10, 90),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

        # Process detections if any markers were found
        if marker_ids is not None and len(marker_ids) > 0:
            marker_ids = marker_ids.flatten()
            
            # Draw markers and estimate pose if camera calibration is available
            if self.camera_info_received:
                # Estimate pose for each marker
                rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
                    corners, self.marker_size, self.camera_matrix, self.dist_coeffs
                )
                
                # Process each marker
                for i, (marker_id, rvec, tvec) in enumerate(zip(marker_ids, rvecs, tvecs)):
                    # Draw axis for each marker
                    cv2.aruco.drawAxis(debug_image, self.camera_matrix, self.dist_coeffs, 
                                      rvec, tvec, self.marker_size / 2)
                    
                    # Convert rotation vector to rotation matrix
                    rot_mat, _ = cv2.Rodrigues(rvec)
                    
                    # Log pose information
                    self.get_logger().info('='*50)
                    self.get_logger().info(f'Marker ID: {marker_id}')
                    self.get_logger().info(f'Position (x,y,z): ({tvec[0][0]:.3f}, {tvec[0][1]:.3f}, {tvec[0][2]:.3f}) meters')
                    
                    # Calculate and print rotation in degrees
                    rot_degrees = np.degrees(np.array([
                        np.arctan2(rot_mat[2, 1], rot_mat[2, 2]),  # Roll
                        np.arcsin(-rot_mat[2, 0]),                 # Pitch
                        np.arctan2(rot_mat[1, 0], rot_mat[0, 0])   # Yaw
                    ]))
                    self.get_logger().info(f'Rotation (roll,pitch,yaw): ({rot_degrees[0]:.1f}, {rot_degrees[1]:.1f}, {rot_degrees[2]:.1f}) degrees')
                    
                    # Create pose message
                    pose = Pose()
                    pose.position.x = float(tvec[0][0])
                    pose.position.y = float(tvec[0][1])
                    pose.position.z = float(tvec[0][2])
                    
                    # Convert rotation matrix to quaternion
                    # Using simplified method since we already have rotation matrix
                    trace = rot_mat[0, 0] + rot_mat[1, 1] + rot_mat[2, 2]
                    if trace > 0:
                        s = 0.5 / np.sqrt(trace + 1.0)
                        pose.orientation.w = 0.25 / s
                        pose.orientation.x = (rot_mat[2, 1] - rot_mat[1, 2]) * s
                        pose.orientation.y = (rot_mat[0, 2] - rot_mat[2, 0]) * s
                        pose.orientation.z = (rot_mat[1, 0] - rot_mat[0, 1]) * s
                    else:
                        if rot_mat[0, 0] > rot_mat[1, 1] and rot_mat[0, 0] > rot_mat[2, 2]:
                            s = 2.0 * np.sqrt(1.0 + rot_mat[0, 0] - rot_mat[1, 1] - rot_mat[2, 2])
                            pose.orientation.w = (rot_mat[2, 1] - rot_mat[1, 2]) / s
                            pose.orientation.x = 0.25 * s
                            pose.orientation.y = (rot_mat[0, 1] + rot_mat[1, 0]) / s
                            pose.orientation.z = (rot_mat[0, 2] + rot_mat[2, 0]) / s
                        elif rot_mat[1, 1] > rot_mat[2, 2]:
                            s = 2.0 * np.sqrt(1.0 + rot_mat[1, 1] - rot_mat[0, 0] - rot_mat[2, 2])
                            pose.orientation.w = (rot_mat[0, 2] - rot_mat[2, 0]) / s
                            pose.orientation.x = (rot_mat[0, 1] + rot_mat[1, 0]) / s
                            pose.orientation.y = 0.25 * s
                            pose.orientation.z = (rot_mat[1, 2] + rot_mat[2, 1]) / s
                        else:
                            s = 2.0 * np.sqrt(1.0 + rot_mat[2, 2] - rot_mat[0, 0] - rot_mat[1, 1])
                            pose.orientation.w = (rot_mat[1, 0] - rot_mat[0, 1]) / s
                            pose.orientation.x = (rot_mat[0, 2] + rot_mat[2, 0]) / s
                            pose.orientation.y = (rot_mat[1, 2] + rot_mat[2, 1]) / s
                            pose.orientation.z = 0.25 * s
                    
                    # Add pose to pose array
                    poses.poses.append(pose)
                    
                    # Add marker ID to markers message
                    markers.marker_ids.append(int(marker_id))
                    
                    # Draw the marker outline
                    corners_array = corners[i][0].astype(int)
                    cv2.polylines(debug_image, [corners_array], True, (0, 255, 0), 2)
                    
                    # Draw the marker ID
                    center = corners_array.mean(axis=0).astype(int)
                    cv2.putText(debug_image, f"ID: {marker_id}", (center[0], center[1]),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
            else:
                # If no camera calibration, just draw the markers
                self.get_logger().warn("No camera calibration available. Cannot estimate pose.")
                cv2.aruco.drawDetectedMarkers(debug_image, corners, marker_ids)
                
                # Add marker IDs to markers message
                for marker_id in marker_ids:
                    markers.marker_ids.append(int(marker_id))

        # Always publish debug image
        try:
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(debug_image, "bgr8"))
        except Exception as e:
            self.get_logger().error(f'Error publishing image: {str(e)}')

        # Publish marker detections and poses
        self.markers_pub.publish(markers)
        if self.camera_info_received and len(poses.poses) > 0:
            self.poses_pub.publish(poses)


def main():
    rclpy.init()
    node = ArucoPoseNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
