#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
import math

def euler_to_quaternion(roll, pitch, yaw):
    """
    Convert Euler angles (roll, pitch, yaw) to quaternion (x, y, z, w)
    """
    qx = math.sin(roll / 2) * math.cos(pitch / 2) * math.cos(yaw / 2) - math.cos(roll / 2) * math.sin(pitch / 2) * math.sin(yaw / 2)
    qy = math.cos(roll / 2) * math.sin(pitch / 2) * math.cos(yaw / 2) + math.sin(roll / 2) * math.cos(pitch / 2) * math.sin(yaw / 2)
    qz = math.cos(roll / 2) * math.cos(pitch / 2) * math.sin(yaw / 2) - math.sin(roll / 2) * math.sin(pitch / 2) * math.cos(yaw / 2)
    qw = math.cos(roll / 2) * math.cos(pitch / 2) * math.cos(yaw / 2) + math.sin(roll / 2) * math.sin(pitch / 2) * math.sin(yaw / 2)
    return qx, qy, qz, qw

def rotate_vector_by_quaternion(vector, q):
    """
    Rotate a 3D vector by a quaternion q (qx, qy, qz, qw)
    """
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

class MarkerPublisher(Node):
    def __init__(self):
        super().__init__('marker_publisher')

        # Declare parameters for the marker's frame and 6D pose.
        self.declare_parameter('frame_id', 'link_torso_5')
        self.declare_parameter('marker_x', 0.5)
        self.declare_parameter('marker_y', 0.14)
        self.declare_parameter('marker_z', -0.22)
        self.declare_parameter('marker_roll', 0.0)
        self.declare_parameter('marker_pitch', 0.0)
        self.declare_parameter('marker_yaw', 0)

        # Create a publisher for visualization markers.
        self.publisher_ = self.create_publisher(Marker, 'visualization_marker', 10)

        # Timer to update markers.
        self.timer = self.create_timer(1.0, self.timer_callback)

    def timer_callback(self):
        # Read parameters.
        frame_id = self.get_parameter('frame_id').get_parameter_value().string_value
        x = self.get_parameter('marker_x').get_parameter_value().double_value
        y = self.get_parameter('marker_y').get_parameter_value().double_value
        z = self.get_parameter('marker_z').get_parameter_value().double_value
        roll = self.get_parameter('marker_roll').get_parameter_value().double_value
        pitch = self.get_parameter('marker_pitch').get_parameter_value().double_value
        yaw = self.get_parameter('marker_yaw').get_parameter_value().double_value

        # Convert Euler angles to a quaternion.
        qx, qy, qz, qw = euler_to_quaternion(roll, pitch, yaw)

        # Publish the main marker (cube).
        marker = Marker()
        marker.header.frame_id = frame_id
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "basic_shapes"
        marker.id = 0
        marker.type = Marker.CUBE
        marker.action = Marker.ADD

        # Marker pose.
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = z
        marker.pose.orientation.x = qx
        marker.pose.orientation.y = qy
        marker.pose.orientation.z = qz
        marker.pose.orientation.w = qw

        # Marker scale and color.
        marker.scale.x = 0.02
        marker.scale.y = 0.02
        marker.scale.z = 0.02
        marker.color.a = 1.0
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0

        self.publisher_.publish(marker)

        # Define axis arrow length.
        axis_length = 0.1

        # Origin for the axes.
        origin = Point(x=x, y=y, z=z)

        # Compute axis endpoints using the marker's orientation.
        x_axis = rotate_vector_by_quaternion((axis_length, 0, 0), (qx, qy, qz, qw))
        y_axis = rotate_vector_by_quaternion((0, axis_length, 0), (qx, qy, qz, qw))
        z_axis = rotate_vector_by_quaternion((0, 0, axis_length), (qx, qy, qz, qw))

        x_end = Point(x=origin.x + x_axis[0], y=origin.y + x_axis[1], z=origin.z + x_axis[2])
        y_end = Point(x=origin.x + y_axis[0], y=origin.y + y_axis[1], z=origin.z + y_axis[2])
        z_end = Point(x=origin.x + z_axis[0], y=origin.y + z_axis[1], z=origin.z + z_axis[2])

        # Helper function to create an arrow marker.
        def create_axis_marker(axis_id, start, end, color):
            m = Marker()
            m.header.frame_id = frame_id
            m.header.stamp = self.get_clock().now().to_msg()
            m.ns = "axis"
            m.id = axis_id
            m.type = Marker.ARROW
            m.action = Marker.ADD
            # Set arrow dimensions (shaft diameter, head diameter).
            m.scale.x = 0.02
            m.scale.y = 0.04
            m.scale.z = 0.0  # not used for ARROW with points
            m.color.a = 1.0
            m.color.r, m.color.g, m.color.b = color
            m.points.append(start)
            m.points.append(end)
            return m

        # X-axis marker (red).
        x_marker = create_axis_marker(1, origin, x_end, (1.0, 0.0, 0.0))
        self.publisher_.publish(x_marker)

        # Y-axis marker (green).
        y_marker = create_axis_marker(2, origin, y_end, (0.0, 1.0, 0.0))
        self.publisher_.publish(y_marker)

        # Z-axis marker (blue).
        z_marker = create_axis_marker(3, origin, z_end, (0.0, 0.0, 1.0))
        self.publisher_.publish(z_marker)

        # Log the marker's pose and the endpoints of the axes.
        self.get_logger().info(
            f"Cube marker pose in {frame_id}: position=({x}, {y}, {z}), orientation(quat)=({qx}, {qy}, {qz}, {qw})"
        )
        self.get_logger().info(
            f"X-axis end: ({x_end.x:.2f}, {x_end.y:.2f}, {x_end.z:.2f})"
        )
        self.get_logger().info(
            f"Y-axis end: ({y_end.x:.2f}, {y_end.y:.2f}, {y_end.z:.2f})"
        )
        self.get_logger().info(
            f"Z-axis end: ({z_end.x:.2f}, {z_end.y:.2f}, {z_end.z:.2f})"
        )

def main(args=None):
    rclpy.init(args=args)
    node = MarkerPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
