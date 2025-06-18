#!/usr/bin/env python3

"""
ArUco marker detector for Zivid 2+ M60 camera (reads image directly from Zivid camera in 2D mode).
This script detects ArUco markers in real-time and displays their orientation.
Also publishes visualization markers, marker IDs, and TF frames to RViz2 using ROS2.
Provides REST API (via FastAPI) to access latest marker IDs and positions.
Also allows 3D point cloud capture via REST endpoint `/capture_pointcloud`.
"""

import cv2
import numpy as np
import sys
import time
import math
import json
from scipy.spatial.transform import Rotation
import zivid

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, TransformStamped
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA
from tf2_ros import TransformBroadcaster

from fastapi import FastAPI
from fastapi.responses import JSONResponse
from threading import Thread
import uvicorn

# Global state
ARUCO_DICT = {
    "DICT_4X4_50": cv2.aruco.DICT_4X4_50,
    "DICT_ARUCO_ORIGINAL": cv2.aruco.DICT_ARUCO_ORIGINAL,
}

latest_marker_data = {}
app = FastAPI()
global_camera = None  # Reuse camera instance

# === REST API endpoints ===

@app.get("/markers")
async def get_markers():
    return JSONResponse(content=latest_marker_data)


@app.get("/capture_pointcloud")
async def capture_pointcloud():
    global global_camera
    try:
        frame_3d = global_camera.capture()
        point_cloud = frame_3d.point_cloud().copy_data("xyz")
        point_cloud_np = np.squeeze(point_cloud)  # Shape: Nx3

        point_list = [
            {"x": float(pt[0]), "y": float(pt[1]), "z": float(pt[2])}
            for pt in point_cloud_np if not np.isnan(pt).any()
        ]

        return JSONResponse(content={"point_count": len(point_list), "points": point_list})
    except Exception as e:
        return JSONResponse(status_code=500, content={"error": str(e)})

# === API Thread ===
def start_fastapi():
    uvicorn.run(app, host="0.0.0.0", port=5000)

# === Utility ===
def rotation_matrix_to_euler_angles(R):
    sy = math.sqrt(R[0, 0] ** 2 + R[1, 0] ** 2)
    if sy > 1e-6:
        roll = math.atan2(R[2, 1], R[2, 2])
        pitch = math.atan2(-R[2, 0], sy)
        yaw = math.atan2(R[1, 0], R[0, 0])
    else:
        roll = math.atan2(-R[1, 2], R[1, 1])
        pitch = math.atan2(-R[2, 0], sy)
        yaw = 0
    return np.array([roll, pitch, yaw])

# === Main Processing Loop ===
def main():
    api_thread = Thread(target=start_fastapi, daemon=True)
    api_thread.start()

    rclpy.init()
    node = Node("aruco_marker_visualizer")
    marker_pub = node.create_publisher(MarkerArray, "aruco_markers_viz", 10)
    tf_broadcaster = TransformBroadcaster(node)

    aruco_dict = cv2.aruco.getPredefinedDictionary(ARUCO_DICT["DICT_4X4_50"])
    aruco_params = cv2.aruco.DetectorParameters_create()

    print("Connecting to Zivid camera...")
    app_zivid = zivid.Application()
    global global_camera
    global_camera = app_zivid.connect_camera()

    settings_2d = zivid.Settings2D(acquisitions=[zivid.Settings2D.Acquisition()])

    image_width = 2448
    image_height = 2048
    fx = (image_width * 600) / 570
    fy = (image_height * 600) / 460
    cx = image_width / 2.0
    cy = image_height / 2.0
    camera_matrix = np.array([
        [fx, 0, cx],
        [0, fy, cy],
        [0, 0, 1]
    ], dtype=np.float32)
    dist_coeffs = np.zeros((5, 1), dtype=np.float32)

    marker_size = 0.05
    axis_length = marker_size * 2

    print("Starting ArUco marker detection... Press 'q' to quit.")

    while rclpy.ok():
        frame = global_camera.capture(settings_2d)
        image = frame.image_rgba().copy_data()
        image_bgr = cv2.cvtColor(image, cv2.COLOR_RGBA2BGR)
        gray = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2GRAY)

        corners, ids, _ = cv2.aruco.detectMarkers(gray, aruco_dict, parameters=aruco_params)

        marker_array = MarkerArray()
        timestamp = node.get_clock().now().to_msg()
        global latest_marker_data
        latest_marker_data = {}

        if ids is not None:
            cv2.aruco.drawDetectedMarkers(image_bgr, corners, ids)
            for i, marker_corners in enumerate(corners):
                marker_id = int(ids[i][0])
                obj_points = np.array([
                    [-marker_size / 2, marker_size / 2, 0],
                    [marker_size / 2, marker_size / 2, 0],
                    [marker_size / 2, -marker_size / 2, 0],
                    [-marker_size / 2, -marker_size / 2, 0]
                ], dtype=np.float32)

                ret, rvec, tvec = cv2.solvePnP(
                    obj_points,
                    marker_corners.astype(np.float32).reshape(4, 2),
                    camera_matrix,
                    dist_coeffs,
                    flags=cv2.SOLVEPNP_IPPE_SQUARE
                )

                if not ret:
                    continue

                rvec = np.array(rvec).reshape((3,))
                tvec = np.array(tvec).reshape((3,))
                rot_matrix, _ = cv2.Rodrigues(rvec)
                q = Rotation.from_matrix(rot_matrix).as_quat()
                euler = rotation_matrix_to_euler_angles(rot_matrix) * 180 / np.pi

                latest_marker_data[marker_id] = {
                    "position": {
                        "x": float(tvec[0]),
                        "y": float(tvec[1]),
                        "z": float(tvec[2])
                    },
                    "rotation_euler_deg": {
                        "roll": float(euler[0]),
                        "pitch": float(euler[1]),
                        "yaw": float(euler[2])
                    }
                }

                marker = Marker()
                marker.header.frame_id = "link_zivid"
                marker.header.stamp = timestamp
                marker.ns = "aruco"
                marker.id = marker_id
                marker.type = Marker.CUBE
                marker.action = Marker.ADD
                marker.pose.position.x = float(tvec[0])
                marker.pose.position.y = float(tvec[1])
                marker.pose.position.z = float(tvec[2])
                marker.pose.orientation.x = float(q[0])
                marker.pose.orientation.y = float(q[1])
                marker.pose.orientation.z = float(q[2])
                marker.pose.orientation.w = float(q[3])
                marker.scale.x = marker_size * 0.8
                marker.scale.y = marker_size * 0.8
                marker.scale.z = marker_size * 0.1
                marker.color = ColorRGBA(r=0.0, g=1.0, b=0.0, a=0.8)
                marker_array.markers.append(marker)

                text_marker = Marker()
                text_marker.header.frame_id = "link_zivid"
                text_marker.header.stamp = timestamp
                text_marker.ns = "aruco_text"
                text_marker.id = 1000 + marker_id
                text_marker.type = Marker.TEXT_VIEW_FACING
                text_marker.action = Marker.ADD
                text_marker.pose.position.x = float(tvec[0])
                text_marker.pose.position.y = float(tvec[1])
                text_marker.pose.position.z = float(tvec[2]) + marker_size * 0.6
                text_marker.scale.z = marker_size * 0.4
                text_marker.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)
                text_marker.text = f"ID:{marker_id}"
                marker_array.markers.append(text_marker)

                transform = TransformStamped()
                transform.header.stamp = timestamp
                transform.header.frame_id = "link_zivid"
                transform.child_frame_id = f"aruco_marker_{marker_id}"
                transform.transform.translation.x = float(tvec[0])
                transform.transform.translation.y = float(tvec[1])
                transform.transform.translation.z = float(tvec[2])
                transform.transform.rotation.x = float(q[0])
                transform.transform.rotation.y = float(q[1])
                transform.transform.rotation.z = float(q[2])
                transform.transform.rotation.w = float(q[3])
                tf_broadcaster.sendTransform(transform)

        marker_pub.publish(marker_array)
        rclpy.spin_once(node, timeout_sec=0)

        cv2.imshow("Zivid ArUco Marker Detection", image_bgr)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cv2.destroyAllWindows()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
