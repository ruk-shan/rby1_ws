#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2 # OpenCV
import numpy as np
import zivid
import datetime

class Zivid2DPublisher(Node):
    def __init__(self):
        super().__init__('zivid_2d_publisher')

        # --- Hardcoded Default Values ---
        self.publish_topic = '/zivid/color/image_raw'
        self.camera_frame_id = 'zivid_camera_color_optical_frame'
        self.capture_interval_sec = 1.0  # Time between captures

        # 2D Capture Settings Defaults
        self.exposure_time_ms = 20.0  # milliseconds
        self.aperture = 5.66
        self.gain = 1.0
        self.projector_brightness = 0.0 # Default to projector OFF for passive 2D
        self.gamma = 1.0
        # --- End Hardcoded Default Values ---

        self.get_logger().info(f"Using hardcoded default settings.")
        self.get_logger().info(f"  Publish topic: {self.publish_topic}")
        self.get_logger().info(f"  Capture interval: {self.capture_interval_sec}s")
        self.get_logger().info(f"  Exposure: {self.exposure_time_ms}ms, Aperture: f/{self.aperture}, Gain: {self.gain}")
        self.get_logger().info(f"  Projector Brightness: {self.projector_brightness}, Gamma: {self.gamma}")


        # Initialize Zivid
        self.app = zivid.Application()
        self.camera = None
        try:
            self.get_logger().info("Connecting to Zivid camera...")
            self.camera = self.app.connect_camera()
            if self.camera.state.status != zivid.CameraState.Status.connected:
                 self.get_logger().error(f"Camera connected but not in 'connected' state. Current state: {self.camera.state.status}")
                 raise RuntimeError(f"Camera not in expected 'connected' state. State: {self.camera.state.status}")
            self.get_logger().info("Successfully connected to Zivid camera.")
        except RuntimeError as e:
            self.get_logger().error(f"Failed to connect to Zivid camera: {e}")
            raise

        # Initialize CvBridge
        self.bridge = CvBridge()

        # Create publisher
        self.image_publisher = self.create_publisher(Image, self.publish_topic, 10)
        self.get_logger().info(f"Publishing 2D images on topic: {self.publish_topic}")

        # Configure 2D settings
        self.settings_2d = self._configure_2d_settings()
        self.get_logger().info(f"Using 2D settings object:\n{self.settings_2d}")

        # Create timer for capturing and publishing
        self.timer = self.create_timer(self.capture_interval_sec, self.capture_and_publish)


    def _configure_2d_settings(self) -> zivid.Settings2D:
        settings_2d = zivid.Settings2D()
        acq_settings = zivid.Settings2D.Acquisition()
        acq_settings.exposure_time = datetime.timedelta(microseconds=self.exposure_time_ms * 1000)
        acq_settings.aperture = self.aperture
        acq_settings.gain = self.gain
        acq_settings.brightness = self.projector_brightness
        settings_2d.acquisitions.append(acq_settings)

        if self.gamma != 1.0: # Only set gamma if it's not the default
             settings_2d.processing.color.gamma = self.gamma
        return settings_2d

    def capture_and_publish(self):
        if not self.camera or self.camera.state.status != zivid.CameraState.Status.connected:
            self.get_logger().warn("Camera not connected or available. Attempting to reconnect...")
            try:
                if self.app is None:
                    self.get_logger().info("Re-initializing Zivid application.")
                    self.app = zivid.Application()
                self.get_logger().info("Attempting to connect camera...")
                self.camera = self.app.connect_camera()
                if self.camera.state.status == zivid.CameraState.Status.connected:
                    self.get_logger().info("Reconnected to Zivid camera.")
                    self.settings_2d = self._configure_2d_settings() # Re-apply settings
                    self.get_logger().info(f"Re-applied 2D settings:\n{self.settings_2d}")
                else:
                    self.get_logger().error(f"Reconnect attempt: Camera state is {self.camera.state.status}, not 'connected'.")
                    if self.camera: # If camera object exists but not connected
                        self.camera.disconnect()
                    self.camera = None
                    return
            except RuntimeError as e:
                self.get_logger().error(f"Failed to reconnect to Zivid camera: {e}")
                self.camera = None
                return
            except Exception as e:
                self.get_logger().error(f"Unexpected error during reconnect attempt: {e}")
                self.camera = None
                return

        try:
            self.get_logger().debug("Capturing 2D frame...")
            with self.camera.capture_2d(self.settings_2d) as frame_2d:
                self.get_logger().debug("Getting RGBA image data...")
                image_rgba_numpy = frame_2d.image_rgba().copy_data()

                ros_image_msg = self.bridge.cv2_to_imgmsg(image_rgba_numpy, encoding="rgba8")
                ros_image_msg.header.stamp = self.get_clock().now().to_msg()
                ros_image_msg.header.frame_id = self.camera_frame_id

                self.image_publisher.publish(ros_image_msg)
                self.get_logger().debug(f"Published 2D image with timestamp: {ros_image_msg.header.stamp.sec}.{ros_image_msg.header.stamp.nanosec}")

        except RuntimeError as e:
            self.get_logger().error(f"Zivid capture error: {e}")
        except Exception as e:
            self.get_logger().error(f"An unexpected error occurred during capture/publish: {e}")

    def destroy_node(self):
        self.get_logger().info("Shutting down Zivid 2D publisher node.")
        if self.timer:
            self.timer.cancel()
            self.timer = None
        if self.camera and hasattr(self.camera, 'state') and self.camera.state.status == zivid.CameraState.Status.connected:
            self.get_logger().info("Disconnecting Zivid camera...")
            try:
                self.camera.disconnect()
                self.get_logger().info("Camera disconnected.")
            except RuntimeError as e:
                self.get_logger().error(f"Error during camera disconnect: {e}")
        self.camera = None
        # self.app = None # Optional: del self.app or self.app = None
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