#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Header
from sensor_msgs.msg import Image, PointCloud2, PointField
import sensor_msgs_py.point_cloud2 as pc2
from cv_bridge import CvBridge
import zivid
import numpy as np
import os

class Zivid2D3DMapPublisher(Node):
    def __init__(self):
        super().__init__('zivid_2d_3d_map_publisher')
        # Publishers
        self.img_pub = self.create_publisher(Image, 'camera/color/image_raw', 10)
        self.pc_pub  = self.create_publisher(PointCloud2, 'camera/depth/points', 10)

        self.bridge = CvBridge()

        # Zivid setup
        self.app = zivid.Application()
        self.get_logger().info('Connecting to Zivid camera…')
        self.camera = self.app.connect_camera()

        # Default 2D + 3D settings
        self.settings = zivid.Settings(
            acquisitions=[zivid.Settings.Acquisition()],
            color=zivid.Settings2D(acquisitions=[zivid.Settings2D.Acquisition()]),
        )
        self.get_logger().info('Zivid camera ready.')

        # 5 Hz capture loop
        # self.timer = self.create_timer(1.0 / 5.0, self.capture_and_publish)
        self.timer = self.create_timer(1.0 / 30.0, self.capture_and_publish)

    def capture_and_publish(self):
        try:
            with self.camera.capture_2d_3d(self.settings) as frame:
                # — Save to disk —
                image_rgba = frame.frame_2d().image_rgba_srgb()
                image_rgba.save("ImageRGBA.png")
                # frame.save("Frame.zdf")
                frame.save("PointCloud.ply")
                self.get_logger().info("Saved ImageRGBA.png, Frame.zdf, PointCloud.ply")

                # — Publish 2D image —
                arr = image_rgba.copy_data()  # H×W×4 uint8
                img_msg = self.bridge.cv2_to_imgmsg(arr, encoding="rgba8")
                img_msg.header.stamp    = self.get_clock().now().to_msg()
                img_msg.header.frame_id = 'zivid_camera_frame'
                self.img_pub.publish(img_msg)

                # — Publish 3D point cloud (XYZ only) in map frame —
                xyz = frame.point_cloud().copy_data("xyz")  # H×W×3 float32
                points = xyz.reshape(-1, 3)
                mask   = np.isfinite(points).all(axis=1)
                points = points[mask]

                pc_header = Header()
                pc_header.stamp    = img_msg.header.stamp
                pc_header.frame_id = 'map'

                fields = [
                    PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
                    PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
                    PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
                ]
                pc2_msg = pc2.create_cloud(pc_header, fields, points.tolist())
                self.pc_pub.publish(pc2_msg)

                self.get_logger().debug(f'Published image and cloud ({points.shape[0]} pts) in map frame')
        except Exception as e:
            self.get_logger().error(f'Capture/publish error: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = Zivid2D3DMapPublisher()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
