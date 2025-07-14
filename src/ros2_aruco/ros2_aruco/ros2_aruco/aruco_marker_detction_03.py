"""
Connects to a physical Zivid camera, captures a new frame, detects an
ArUco marker, and visualizes it in RViz.
"""

import datetime
import numpy as np
import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point, Quaternion, Pose, Vector3, PoseStamped, TransformStamped
from std_msgs.msg import ColorRGBA
import zivid
from tf2_ros import TransformBroadcaster, Buffer, TransformListener
import math
import cv2
import cv2.aruco as aruco
from scipy.spatial.transform import Rotation as R

def quaternion_to_euler(x, y, z, w):
    """
    Convert a quaternion into euler angles (roll, pitch, yaw)
    roll is rotation around x in radians (counterclockwise)
    pitch is rotation around y in radians (counterclockwise)
    yaw is rotation around z in radians (counterclockwise)
    """
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll_x = math.atan2(t0, t1)
    
    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch_y = math.asin(t2)
    
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = math.atan2(t3, t4)
    
    return roll_x, pitch_y, yaw_z  # in radians

class ArucoMarkerVisualizer(Node):
    def __init__(self):
        super().__init__('aruco_marker_visualizer')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Camera intrinsics
        self.camera_matrix = np.array([
            [1240.81872558594, 0.0, 604.823913574219],
            [0.0, 1240.64428710938, 509.505249023438],
            [0.0, 0.0, 1.0]
        ], dtype=np.float32)
        
        # Distortion coefficients
        self.dist_coeffs = np.array([
            0.0494324862957001,
            -0.0316880233585835,
            -0.000165768535225652,
            -1.67045436683111e-05,
            -0.19348831474781
        ], dtype=np.float32)
        
        # Marker size in meters (35mm = 0.035m)
        self.marker_size = 0.035  # 35mm
        
        # ArUco dictionary and parameters
        self.aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_6X6_250)
        self.parameters = aruco.DetectorParameters()
        
        # Use a latched publisher to ensure RViz gets the marker even if it starts after this node
        self.marker_pub = self.create_publisher(Marker, '/visualization_marker', 10)
        self.timer = self.create_timer(0.05, self.timer_callback)  # 20 Hz for smoother updates
        self.marker_pose = None
        self.marker_id = None
        self.get_logger().info('ArucoMarkerVisualizer node started')
        self.get_logger().info(f'Using marker size: {self.marker_size*1000:.1f}mm')
    
    def update_marker_pose(self, position, quaternion, marker_id):
        """Update the marker pose that will be published."""
        self.marker_pose = (position, quaternion)
        self.marker_id = marker_id
        
        # Log the detected marker position
        self.get_logger().info(
            f'Marker {marker_id} detected at position: '
            f'x={position[0]:.3f}, y={position[1]:.3f}, z={position[2]:.3f}'
        )
    
    def timer_callback(self):
        """Publish marker at the current pose."""
        if self.marker_pose is None or self.marker_id is None:
            print("No marker pose or ID available")
            return
            
        position, quaternion = self.marker_pose
        
        # Create main marker (simple cube)
        marker = Marker()
        marker.header.frame_id = "link_camera_frame"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "aruco_marker"
        marker.id = self.marker_id
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        
        # Set marker size (35mm x 35mm x 5mm)
        marker.scale.x = self.marker_size  # 35mm
        marker.scale.y = self.marker_size  # 35mm
        marker.scale.z = 0.005  # 5mm
        
        # Set marker color (green)
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 0.8  # Slightly transparent
        
        # Ensure position values are Python floats
        pos_x = float(position[0])
        pos_y = float(position[1])
        pos_z = float(position[2])
        
        # Set the pose of the marker
        marker.pose.position = Point(x=pos_x, y=pos_y, z=pos_z)
        marker.pose.orientation = Quaternion(
            x=quaternion[0], y=quaternion[1], 
            z=quaternion[2], w=quaternion[3])
            
        # Set the scale of the marker (in meters)
        marker.scale.x = self.marker_size  # 35mm
        marker.scale.y = self.marker_size  # 35mm
        marker.scale.z = 0.005  # 5mm
        
        # Set the color to semi-transparent green
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 0.8  # Semi-transparent
        
        # Publish the main marker (green square)
        self.marker_pub.publish(marker)
        
        # Create axis markers for better orientation visualization
        axis_length = 0.1  # 10cm
        origin = Point(x=pos_x, y=pos_y, z=pos_z)
        
        # Create axis arrows (X=red, Y=green, Z=blue)
        def create_axis_marker(axis_id, start, end, color):
            m = Marker()
            m.header.frame_id = "link_camera_frame"
            m.header.stamp = self.get_clock().now().to_msg()
            m.ns = "axis"
            m.id = axis_id + self.marker_id * 10  # Unique ID for each marker's axes
            m.type = Marker.ARROW
            m.action = Marker.ADD
            m.scale.x = 0.01  # Shaft diameter
            m.scale.y = 0.02  # Head diameter
            m.scale.z = 0.0    # Not used for ARROW with points
            m.color.r, m.color.g, m.color.b = color
            m.color.a = 1.0
            m.points = [start, end]
            return m
        
        # Function to rotate a vector by a quaternion
        def rotate_vector(v, q):
            # q = [x, y, z, w]
            x, y, z = v
            qx, qy, qz, qw = q
            
            # Convert quaternion to rotation matrix
            xx = qx * qx
            yy = qy * qy
            zz = qz * qz
            xy = qx * qy
            xz = qx * qz
            yz = qy * qz
            wx = qw * qx
            wy = qw * qy
            wz = qw * qz
            
            # Apply rotation
            rx = (1 - 2 * (yy + zz)) * x + 2 * (xy - wz) * y + 2 * (xz + wy) * z
            ry = 2 * (xy + wz) * x + (1 - 2 * (xx + zz)) * y + 2 * (yz - wx) * z
            rz = 2 * (xz - wy) * x + 2 * (yz + wx) * y + (1 - 2 * (xx + yy)) * z
            
            return rx, ry, rz
        
        # Use the marker's quaternion for rotation
        marker_quat = [quaternion[0], quaternion[1], quaternion[2], quaternion[3]]  # x, y, z, w
        
        # Debug output
        self.get_logger().info(f"Marker quaternion: {marker_quat}")
        
        # Create axis vectors and rotate them with the marker's quaternion
        x_axis = rotate_vector((axis_length, 0, 0), marker_quat)
        y_axis = rotate_vector((0, axis_length, 0), marker_quat)
        z_axis = rotate_vector((0, 0, axis_length), marker_quat)
        
        # Create endpoints for the arrows
        x_end = Point(x=origin.x + x_axis[0], y=origin.y + x_axis[1], z=origin.z + x_axis[2])
        y_end = Point(x=origin.x + y_axis[0], y=origin.y + y_axis[1], z=origin.z + y_axis[2])
        z_end = Point(x=origin.x + z_axis[0], y=origin.y + z_axis[1], z=origin.z + z_axis[2])
        
        # Create and publish axis markers (aligned with marker's orientation)
        arrows = [
            (1, origin, x_end, (1.0, 0.0, 0.0)),  # Red X (aligned with marker's X)
            (2, origin, y_end, (0.0, 1.0, 0.0)),  # Green Y (aligned with marker's Y)
            (3, origin, z_end, (0.0, 0.0, 1.0))   # Blue Z (aligned with marker's Z)
        ]
        
        for arrow in arrows:
            self.marker_pub.publish(create_axis_marker(*arrow))
        
        self.get_logger().info(f"Published marker at: ({pos_x:.3f}, {pos_y:.3f}, {pos_z:.3f})")

    @staticmethod
    def rotate_vector_by_quaternion(vector, q):
        """Rotate a 3D vector by a quaternion q (qx, qy, qz, qw)"""
        x, y, z = vector
        qx, qy, qz, qw = q
        # Compute rotation matrix elements from quaternion:
        r00 = 1 - 2*(qy*qy + qz*qz)
        r01 = 2*(qx*qy - qz*qw)
        r02 = 2*(qx*qz + qy*qw)
        r10 = 2*(qx*qy + qz*qw)
        r11 = 1 - 2*(qx*qx + qz*qz)
        r12 = 2*(qy*qz - qx*qw)
        r20 = 2*(qx*qz - qy*qw)
        r21 = 2*(qy*qz + qx*qw)
        r22 = 1 - 2*(qx*qx + qy*qy)
        rx = r00 * x + r01 * y + r02 * z
        ry = r10 * x + r11 * y + r12 * z
        rz = r20 * x + r21 * y + r22 * z
        return rx, ry, rz

def get_marker_pose_from_live_camera(visualizer_node) -> None:
    """
    Connects to a live camera, continuously captures frames, detects ArUco markers,
    and visualizes them in RViz.
    """
    app = zivid.Application()
    camera = None  # Initialize camera to None for the finally block

    try:
        print("Connecting to the first available Zivid camera...")
        camera = app.connect_camera()

        print("\nIMPORTANT: Place the Zivid Calibration Board in the camera's view.")
        print("Press Ctrl+C to exit...")

        print("Configuring capture settings...")
        settings = zivid.Settings(
            acquisitions=[
                zivid.Settings.Acquisition(
                    aperture=5.6,
                    exposure_time=datetime.timedelta(microseconds=20000),  # 20 ms
                )
            ]
        )

        # Configure marker detection parameters once
        marker_dictionary = zivid.calibration.MarkerDictionary.aruco4x4_50
        marker_ids_to_detect = [1]  # The function expects a list of integer IDs.
        print(f"Looking for ArUco marker(s) with IDs: {marker_ids_to_detect}")

        while rclpy.ok():
            try:
                # Capture a new frame
                frame = camera.capture(settings)
                
                # Detect markers in the frame
                detection_result = zivid.calibration.detect_markers(frame, marker_ids_to_detect, marker_dictionary)
                
                if not detection_result.valid():
                    print("No markers detected in this frame")
                    rclpy.spin_once(visualizer_node, timeout_sec=0.1)
                    continue
                
                # Get detected markers
                detected_markers = detection_result.detected_markers()
                print(f"\nFound {len(detected_markers)} marker(s)")
                
                # Process each detected marker
                for i, marker in enumerate(detected_markers, 1):
                    try:
                        # Get marker ID and pose
                        marker_id = getattr(marker, 'id', 0)  # Default to 0 if ID is not available
                        print(f"Marker {i} - ID: {marker_id} (Type: {type(marker_id)})")
                        
                        # Debug print all attributes of the marker
                        print("Marker attributes:", dir(marker))
                        if hasattr(marker, 'pose'):
                            print("Pose attributes:", dir(marker.pose))
                        
                        if hasattr(marker, 'pose'):
                            marker_pose = marker.pose
                            position = None
                            quaternion = None
                            
                            # Extract pose from the transformation matrix
                            try:
                                # Get the 4x4 transformation matrix
                                transform_matrix = marker_pose.to_matrix()
                                
                                # Extract position (translation) from the last column
                                position = transform_matrix[0:3, 3]  # First 3 elements of last column
                                
                                # Extract rotation matrix (3x3 top-left)
                                rotation_matrix = transform_matrix[0:3, 0:3]
                                
                                # Convert rotation matrix to quaternion
                                # Using the method from: https://www.euclideanspace.com/maths/geometry/rotations/conversions/matrixToQuaternion/
                                trace = np.trace(rotation_matrix)
                                if trace > 0:
                                    S = np.sqrt(trace + 1.0) * 2  # S=4*qw 
                                    w = 0.25 * S
                                    x = (rotation_matrix[2,1] - rotation_matrix[1,2]) / S
                                    y = (rotation_matrix[0,2] - rotation_matrix[2,0]) / S 
                                    z = (rotation_matrix[1,0] - rotation_matrix[0,1]) / S 
                                elif (rotation_matrix[0,0] > rotation_matrix[1,1]) and (rotation_matrix[0,0] > rotation_matrix[2,2]):
                                    S = np.sqrt(1.0 + rotation_matrix[0,0] - rotation_matrix[1,1] - rotation_matrix[2,2]) * 2
                                    w = (rotation_matrix[2,1] - rotation_matrix[1,2]) / S
                                    x = 0.25 * S
                                    y = (rotation_matrix[0,1] + rotation_matrix[1,0]) / S 
                                    z = (rotation_matrix[0,2] + rotation_matrix[2,0]) / S 
                                elif rotation_matrix[1,1] > rotation_matrix[2,2]:
                                    S = np.sqrt(1.0 + rotation_matrix[1,1] - rotation_matrix[0,0] - rotation_matrix[2,2]) * 2
                                    w = (rotation_matrix[0,2] - rotation_matrix[2,0]) / S
                                    x = (rotation_matrix[0,1] + rotation_matrix[1,0]) / S 
                                    y = 0.25 * S
                                    z = (rotation_matrix[1,2] + rotation_matrix[2,1]) / S 
                                else:
                                    S = np.sqrt(1.0 + rotation_matrix[2,2] - rotation_matrix[0,0] - rotation_matrix[1,1]) * 2
                                    w = (rotation_matrix[1,0] - rotation_matrix[0,1]) / S
                                    x = (rotation_matrix[0,2] + rotation_matrix[2,0]) / S
                                    y = (rotation_matrix[1,2] + rotation_matrix[2,1]) / S
                                    z = 0.25 * S
                                
                                # Use the original quaternion without any transformations
                                quaternion = np.array([x, y, z, w])
                                # Position is already correctly extracted from the transform matrix
                                

                                
                            except Exception as e:
                                print(f"Error processing pose matrix: {e}")
                                position = None
                                quaternion = None
                            
                            # Update visualizer if we have both position and orientation
                            if position is not None and quaternion is not None:
                                try:
                                    # Convert from mm to meters and ensure Z is positive (in front of camera)
                                    position_m = position / 1000.0  # Convert to meters
                                    position_m[2] = abs(position_m[2])  # Ensure Z is positive (in front of camera)
                                    print(f"Updating marker {marker_id} at position: {position_m}")
                                    
                                    # Use marker ID or a default ID if not available
                                    marker_id_int = int(float(marker_id)) if marker_id != 'unknown' else 0
                                    
                                    
                                    # Use the detected quaternion as-is
                                    quaternion = quaternion
                                    
                                    # Publish the marker
                                    visualizer_node.update_marker_pose(position_m, quaternion, marker_id_int)
                                    
                                    # Get transform from camera to base
                                    try:
                                        # Lookup transform from camera to base
                                        transform = visualizer_node.tf_buffer.lookup_transform(
                                            'base',  # target frame
                                            'link_camera_frame',  # source frame
                                            rclpy.time.Time()
                                        )
                                        
                                        # Extract translation and rotation
                                        t = transform.transform.translation
                                        q = transform.transform.rotation
                                        
                                        # Convert marker position to base frame
                                        # First, rotate the marker position by the camera's rotation
                                        rx = q.w*q.w*t.x + 2*q.y*q.w*t.z - 2*q.z*q.w*t.y + q.x*q.x*t.x + 2*q.y*q.x*t.y + 2*q.z*q.x*t.z - q.z*q.z*t.x - q.y*q.y*t.x
                                        ry = 2*q.x*q.y*t.x + q.y*q.y*t.y + 2*q.z*q.y*t.z + 2*q.w*q.z*t.x - q.z*q.z*t.y + q.w*q.w*t.y - 2*q.x*q.w*t.z - q.x*q.x*t.y
                                        rz = 2*q.x*q.z*t.x + 2*q.y*q.z*t.y + q.z*q.z*t.z - 2*q.w*q.y*t.x - q.y*q.y*t.z + 2*q.w*q.x*t.y - q.x*q.x*t.z + q.w*q.w*t.z
                                        
                                        # Then add the camera's translation
                                        marker_pos_base = [
                                            position_m[0] + rx,
                                            position_m[1] + ry,
                                            position_m[2] + rz
                                        ]
                                        
                                        # Convert quaternion to Euler angles (roll, pitch, yaw)
                                        roll, pitch, yaw = quaternion_to_euler(
                                            quaternion[0], quaternion[1], quaternion[2], quaternion[3]
                                        )
                                        
                                        # Print only position and Euler angles in radians
                                        print(f"Marker {marker_id} - Position (x, y, z): [{marker_pos_base[0]:.6f}, {marker_pos_base[1]:.6f}, {marker_pos_base[2]:.6f}] m")
                                        print(f"Marker {marker_id} - Orientation (roll, pitch, yaw): [{roll:.6f}, {pitch:.6f}, {yaw:.6f}] rad")
                                        
                                    except Exception as e:
                                        print(f"Could not get transform from camera to base: {e}")
                                        
                                except Exception as e:
                                    print(f"Error updating marker pose: {e}")
                    
                    except Exception as e:
                        print(f"Error processing marker: {str(e)}")
                
                # Process ROS callbacks
                rclpy.spin_once(visualizer_node, timeout_sec=0.1)
                
            except KeyboardInterrupt:
                print("\nExiting...")
                break
            except Exception as e:
                print(f"Error in capture loop: {str(e)}")
                rclpy.spin_once(visualizer_node, timeout_sec=0.1)
                continue

    except RuntimeError as e:
        print(f"An error occurred: {e}")

    finally:
        if camera:
            print("\nDisconnecting camera.")
            camera.disconnect()


def main(args=None):
    rclpy.init(args=args)
    visualizer = ArucoMarkerVisualizer()
    
    # Print RViz setup instructions
    print("\nRViz Setup Instructions:")
    print("1. Open RViz: rviz2")
    print("2. Set Fixed Frame to 'link_camera_frame'")
    print("3. Add a Marker display and set the topic to '/visualization_marker'")
    print("4. Add a TF display to see coordinate frames")
    print("5. Look for a green square at the marker's position with RGB axes")
    print("\nLooking for ArUco markers...")
    
    try:
        get_marker_pose_from_live_camera(visualizer)
    except KeyboardInterrupt:
        print("\nShutting down...")
    finally:
        visualizer.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()