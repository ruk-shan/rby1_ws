#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Pose, Point, Quaternion, Vector3
from std_msgs.msg import ColorRGBA, Header
import yaml
import os
import math
from ament_index_python.packages import get_package_share_directory

def euler_to_quaternion(roll, pitch, yaw):
    """
    Convert Euler angles to quaternion (x, y, z, w)
    
    Args:
        roll: Rotation around x-axis in radians
        pitch: Rotation around y-axis in radians
        yaw: Rotation around z-axis in radians
        
    Returns:
        Quaternion as (x, y, z, w)
    """
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    
    qx = sr * cp * cy - cr * sp * sy
    qy = cr * sp * cy + sr * cp * sy
    qz = cr * cp * sy - sr * sp * cy
    qw = cr * cp * cy + sr * sp * sy
    
    return (qx, qy, qz, qw)

class ModelVisualizer(Node):
    def __init__(self):
        super().__init__('model_visualizer')
        
        # Load configuration
        config_path = os.path.join(
            get_package_share_directory('point_cloud_pub'),
            'config',
            'models_config.yaml'
        )
        
        with open(config_path, 'r') as f:
            self.config = yaml.safe_load(f)
        
        # Create publisher for markers
        self.marker_pub = self.create_publisher(MarkerArray, 'visualization_marker_array', 10)
        
        # Timer for publishing markers
        self.timer = self.create_timer(0.1, self.publish_markers)
        self.marker_id = 0
        
        self.get_logger().info('Model Visualizer started')
    
    def create_marker(self, model_config, marker_id):
        marker = Marker()
        
        # Set the frame ID and timestamp
        marker.header = Header()
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.header.frame_id = model_config['frame_id']
        
        # Set the namespace and id for this marker
        marker.ns = model_config['name']
        marker.id = marker_id
        
        # Set the marker type
        if model_config['path'].endswith('.stl'):
            marker.type = Marker.MESH_RESOURCE
            marker.mesh_resource = model_config['path']
            marker.mesh_use_embedded_materials = False
        else:
            # Fallback to a cube if not STL
            marker.type = Marker.CUBE
        
        # Set the marker action
        marker.action = Marker.ADD if model_config['enabled'] else Marker.DELETE
        
        # Set the position of the marker
        marker.pose = Pose()
        marker.pose.position = Point(
            x=float(model_config['position'][0]),
            y=float(model_config['position'][1]),
            z=float(model_config['position'][2])
        )
        
        # Convert Euler angles to quaternion
        roll = float(model_config['orientation_rpy'][0])
        pitch = float(model_config['orientation_rpy'][1])
        yaw = float(model_config['orientation_rpy'][2])
        qx, qy, qz, qw = euler_to_quaternion(roll, pitch, yaw)
        
        # Set the orientation using the converted quaternion
        marker.pose.orientation = Quaternion(
            x=qx,
            y=qy,
            z=qz,
            w=qw
        )
        
        # Set the scale of the marker
        marker.scale = Vector3(
            x=model_config['scale'][0],
            y=model_config['scale'][1],
            z=model_config['scale'][2]
        )
        
        # Set the color if provided, otherwise use default (white)
        if 'color' in model_config:
            marker.color = ColorRGBA(
                r=float(model_config['color'][0]),
                g=float(model_config['color'][1]),
                b=float(model_config['color'][2]),
                a=float(model_config['color'][3]) if len(model_config['color']) > 3 else 1.0
            )
        else:
            # Default to white if no color specified
            marker.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)
        
        # Marker will last until it's explicitly deleted or the node is shut down
        marker.lifetime.sec = 0
        
        return marker
    
    def publish_markers(self):
        marker_array = MarkerArray()
        
        for i, model in enumerate(self.config['models']):
            if model['enabled']:
                marker = self.create_marker(model, i)
                marker_array.markers.append(marker)
        
        self.marker_pub.publish(marker_array)

def main(args=None):
    rclpy.init(args=args)
    visualizer = ModelVisualizer()
    
    try:
        rclpy.spin(visualizer)
    except KeyboardInterrupt:
        pass
    finally:
        visualizer.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
