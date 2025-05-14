#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Header # Kept for consistency, though not strictly needed for image only
from sensor_msgs.msg import Image
# PointCloud2 related imports are removed as we are not publishing point clouds
# import sensor_msgs_py.point_cloud2 as pc2
# from sensor_msgs.msg import PointCloud2, PointField

from cv_bridge import CvBridge
import zivid
import numpy as np
import os # For file saving path, if needed, though example saves to CWD

class Zivid2DPublisher(Node): # Renamed class
    def __init__(self):
        super().__init__('zivid_2d_publisher') # Renamed node
        
        # Publisher for 2D Image
        self.img_pub = self.create_publisher(Image, 'camera/color/image_raw', 10)
        # Point cloud publisher is removed
        # self.pc_pub  = self.create_publisher(PointCloud2, 'camera/depth/points', 10)

        self.bridge = CvBridge()

        # Zivid setup
        self.app = zivid.Application()
        self.get_logger().info('Connecting to Zivid camera…')
        self.camera = self.app.connect_camera()

        # Default 2D settings
        # We need a zivid.Settings2D object for camera.capture_2d()
        self.settings_2d = zivid.Settings2D(
            acquisitions=[
                zivid.Settings2D.Acquisition(
                    # You might want to add specific 2D settings here, e.g.:
                    # exposure_time=datetime.timedelta(microseconds=20000),
                    # aperture=5.66,
                    # brightness=0.0, # Projector brightness (0.0 for passive 2D)
                    # gain=1.0,
                )
            ]
        )
        self.get_logger().info(f'Zivid camera ready. Using 2D settings:\n{self.settings_2d}')

        # Capture loop frequency (e.g., 30 Hz from the example)
        self.timer = self.create_timer(1.0 / 30.0, self.capture_and_publish)

    def capture_and_publish(self):
        try:
            # Use camera.capture_2d() with self.settings_2d
            with self.camera.capture_2d(self.settings_2d) as frame_2d:
                # frame_2d is a zivid.Frame2D object

                # — Save 2D image to disk —
                # The original code used frame.frame_2d().image_rgba_srgb()
                # Since frame_2d IS the Frame2D object, we call image_rgba_srgb() on it.
                # This method returns an image object (e.g., ImageSRGB or similar)
                image_srgb_obj = frame_2d.image_rgba_srgb()
                
                # Save the sRGB image
                # save_filename = "ImageSRGB.png"
                # image_srgb_obj.save(save_filename)
                # self.get_logger().info(f"Saved 2D image to {save_filename}")
                self.get_logger().info(f"image published")

                # ZDF and PLY saving are removed as they are for 3D data or combined frames
                # frame.save("Frame.zdf") # Frame2D can be saved as ZDF, but it's 2D only
                # frame.save("PointCloud.ply") # Not applicable for Frame2D

                # — Publish 2D image —
                arr = image_srgb_obj.copy_data()  # H×W×4 uint8 (assuming sRGB data is RGBA)
                img_msg = self.bridge.cv2_to_imgmsg(arr, encoding="rgba8")
                img_msg.header.stamp    = self.get_clock().now().to_msg()
                img_msg.header.frame_id = 'zivid_camera_frame' # Or your desired camera frame ID
                self.img_pub.publish(img_msg)

                # Point cloud publishing code is removed
                # ...

                self.get_logger().debug(f'Published 2D image')
        except Exception as e:
            self.get_logger().error(f'Capture/publish error: {e}')
            # Consider adding more specific error handling for Zivid API calls if needed

    def destroy_node(self):
        self.get_logger().info("Shutting down Zivid 2D publisher node.")
        if self.timer:
            self.timer.cancel()
        if self.camera and hasattr(self.camera, 'state') and self.camera.state.status == zivid.CameraState.Status.connected:
            self.get_logger().info("Disconnecting Zivid camera...")
            try:
                self.camera.disconnect()
                self.get_logger().info("Camera disconnected.")
            except RuntimeError as e:
                self.get_logger().error(f"Error during camera disconnect: {e}")
        self.camera = None
        # self.app = None # Optional
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = Zivid2DPublisher()
        rclpy.spin(node)
    except KeyboardInterrupt:
        if node:
            node.get_logger().info("Keyboard interrupt, shutting down...")
    except RuntimeError as e: 
        logger = rclpy.logging.get_logger("zivid_2d_publisher_main")
        if node and hasattr(node, 'get_logger'):
            logger = node.get_logger()
        logger.fatal(f"Critical RuntimeError: {e}")
    except Exception as e:
        logger = rclpy.logging.get_logger("zivid_2d_publisher_main")
        if node and hasattr(node, 'get_logger'):
            logger = node.get_logger()
        logger.fatal(f"An unexpected exception occurred in main: {e}")
    finally:
        if node:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()