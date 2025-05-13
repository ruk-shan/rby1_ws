#!/usr/bin/env python3

"""
This node locates Aruco AR markers in images and publishes their ids and poses.

Subscriptions:
   /camera/image_raw (sensor_msgs.msg.Image)
   /camera/camera_info (sensor_msgs.msg.CameraInfo)

Published Topics:
    /aruco_markers (ros2_aruco_interfaces.msg.ArucoMarkers)
       Provides an array of all poses along with the corresponding marker ids
    /aruco_detections (sensor_msgs.msg.Image)
       Shows the detected markers in the image

Parameters:
    aruco_dictionary_id - dictionary that was used to generate markers
                          (default DICT_4X4_50)
    image_topic - image topic to subscribe to (default /camera/image_raw)
    camera_info_topic - camera info topic to subscribe to
                         (default /camera/camera_info)
    max_fps - maximum processing FPS (default 30.0)
"""

import numpy as np
import cv2
from cv_bridge import CvBridge
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
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

class ArucoNode(Node):
    def __init__(self):
        super().__init__("aruco_node")

        # Declare parameters with default values
        self.declare_parameters(
            namespace='',
            parameters=[
                ('marker_size', 0.0625),
                ('aruco_dictionary_id', 'DICT_4X4_50'),
                ('image_topic', '/camera/image_raw'),
                ('max_fps', 30.0),  # Maximum processing FPS
            ]
        )

        # Get parameters
        self.marker_size = self.get_parameter('marker_size').value
        dictionary_id_name = self.get_parameter('aruco_dictionary_id').value
        image_topic = self.get_parameter('image_topic').value
        self.max_fps = self.get_parameter('max_fps').value

        # Log configuration
        self.get_logger().info(f"Marker type: {dictionary_id_name}")
        self.get_logger().info(f"Image topic: {image_topic}")
        self.get_logger().info(f"Max FPS: {self.max_fps}")

        # Validate dictionary
        if dictionary_id_name not in ARUCO_DICT:
            self.get_logger().error(f"ArUco tag type '{dictionary_id_name}' not supported")
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
        self.markers_pub = self.create_publisher(ArucoMarkers, "aruco_markers", reliable_qos)
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

    def image_callback(self, img_msg):
        # Rate limiting
        now = self.get_clock().now()
        if (now - self.last_process_time).nanoseconds < (1e9 / self.max_fps):
            return
        self.last_process_time = now

        # Convert ROS Image message to OpenCV image
        try:
            cv_image = self.bridge.imgmsg_to_cv2(img_msg, "bgr8")
            self.get_logger().info(f'Received image: {cv_image.shape}')
        except Exception as e:
            self.get_logger().error(f'Could not convert image: {str(e)}')
            return

        # Initialize the output messages
        markers = ArucoMarkers()
        markers.header = img_msg.header

        # Preprocess image
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        gray = cv2.GaussianBlur(gray, (5, 5), 0)
        
        # Adaptive thresholding
        thresh = cv2.adaptiveThreshold(
            gray,
            255,
            cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
            cv2.THRESH_BINARY_INV,
            11,
            2
        )
        
        # Detect ArUco markers in both original and preprocessed images
        self.get_logger().info('Attempting marker detection...')
        corners, marker_ids, _ = self.detector.detectMarkers(cv_image)
        if marker_ids is None:  # If no markers found, try preprocessed image
            corners, marker_ids, _ = self.detector.detectMarkers(thresh)
            
        self.get_logger().info(f'Detection result - corners: {type(corners)}, marker_ids: {type(marker_ids)}')
        
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
                    self.get_logger().info(f'Filtered out marker {marker_ids[i][0]} - perimeter: {perimeter:.1f}, area: {area:.1f}, aspect: {aspect_ratio:.2f}')
            
            if valid_indices:
                corners = [corners[i] for i in valid_indices]
                marker_ids = np.array([marker_ids[i] for i in valid_indices])
            else:
                corners = []
                marker_ids = None
        
        # Create debug image
        debug_image = cv_image.copy()
        
        # Add preprocessed image as small overlay
        h, w = cv_image.shape[:2]
        overlay_h = h // 4
        overlay_w = w // 4
        thresh_small = cv2.resize(thresh, (overlay_w, overlay_h))
        thresh_color = cv2.cvtColor(thresh_small, cv2.COLOR_GRAY2BGR)
        debug_image[h-overlay_h:h, w-overlay_w:w] = thresh_color

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
            output_image = cv_image.copy()

            # Process each marker
            for i, (marker_corner, marker_id) in enumerate(zip(corners, marker_ids)):
                # Get marker corners
                corners_array = marker_corner[0].astype(int)  # Convert to integers once
                center = corners_array.mean(axis=0).astype(int)

                # Draw marker outline
                cv2.polylines(output_image, [corners_array], True, (0, 255, 0), 2)

                # Draw center point
                cv2.circle(output_image, tuple(center), 4, (0, 0, 255), -1)

                # Draw ID
                cv2.putText(output_image, str(marker_id),
                          tuple(corners_array[0] + [-10, -10]),
                          cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

                # Add to markers message
                markers.marker_ids.append(marker_id)

                # Log detection info for each marker
                self.get_logger().info('='*50)
                self.get_logger().info(f'Marker {i+1}/{len(marker_ids)}:')
                self.get_logger().info(f'  ID: {marker_id}')
                self.get_logger().info(f'  Center: (X={center[0]}, Y={center[1]})')
                self.get_logger().info(f'  Corners:')
                self.get_logger().info(f'    Top Left:     ({corners_array[0][0]}, {corners_array[0][1]})')
                self.get_logger().info(f'    Top Right:    ({corners_array[1][0]}, {corners_array[1][1]})')
                self.get_logger().info(f'    Bottom Right: ({corners_array[2][0]}, {corners_array[2][1]})')
                self.get_logger().info(f'    Bottom Left:  ({corners_array[3][0]}, {corners_array[3][1]})')
        else:
            self.get_logger().info('No markers detected')

        # Always publish debug image
        try:
            if marker_ids is not None and len(marker_ids) > 0:
                # Use the annotated image with markers
                self.image_pub.publish(self.bridge.cv2_to_imgmsg(output_image, "bgr8"))
            else:
                # Use debug image without markers
                self.image_pub.publish(self.bridge.cv2_to_imgmsg(debug_image, "bgr8"))
        except Exception as e:
            self.get_logger().error(f'Error publishing image: {str(e)}')

        # Publish marker detections
        self.markers_pub.publish(markers)


def main():
    rclpy.init()
    node = ArucoNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
