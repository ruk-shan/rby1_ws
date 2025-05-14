#!/usr/bin/env python3

"""
Launch script for the complete RBY1 system with ArUco marker detection.
This script starts:
1. Camera stream
2. ArUco marker detector
3. URDF visualization
4. Joint state publisher
"""

import os
import sys
import time
import signal
import subprocess
import argparse
from pathlib import Path

# Get the base directory
BASE_DIR = Path(os.path.dirname(os.path.abspath(__file__)))

# Define paths to the components
CAMERA_STREAM_PATH = BASE_DIR / "src/point_cloud_pub/point_cloud_pub/camera_stream_02.py"
ARUCO_DETECTOR_PATH = BASE_DIR / "src/ros2_aruco/ros2_aruco/ros2_aruco/aruco_detector_zivid.py"
JOINT_STATE_PUB_PATH = BASE_DIR / "src/rby1_joint_state_publisher/rby1_joint_state_publisher/rby1_state_pub.py"

# List to store all processes
processes = []

def signal_handler(sig, frame):
    """Handle Ctrl+C to gracefully terminate all processes."""
    print("\nShutting down all processes...")
    for proc in processes:
        if proc.poll() is None:  # If process is still running
            proc.terminate()
            try:
                proc.wait(timeout=5)  # Wait for process to terminate
            except subprocess.TimeoutExpired:
                proc.kill()  # Force kill if it doesn't terminate
    print("All processes terminated. Exiting.")
    sys.exit(0)

def start_process(cmd, name, shell=False):
    """Start a process and add it to the processes list."""
    print(f"Starting {name}...")
    try:
        if shell:
            proc = subprocess.Popen(cmd, shell=True)
        else:
            proc = subprocess.Popen(cmd)
        processes.append(proc)
        print(f"{name} started with PID {proc.pid}")
        return proc
    except Exception as e:
        print(f"Error starting {name}: {e}")
        return None

def check_connectivity(ip, name, max_retries=3):
    """Check if a device is reachable via ping"""
    print(f"Checking connectivity to {name} ({ip})...")
    for i in range(max_retries):
        try:
            # Use ping with a timeout of 1 second and 2 packets
            result = subprocess.run(
                ["ping", "-c", "2", "-W", "1", ip],
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True,
                check=False
            )
            if result.returncode == 0:
                print(f"✅ {name} is connected!")
                return True
            else:
                print(f"⚠️ Attempt {i+1}/{max_retries}: {name} not responding...")
                time.sleep(1)
        except Exception as e:
            print(f"⚠️ Error checking {name}: {e}")
    
    print(f"❌ {name} is not reachable. Please check the connection.")
    return False

def main():
    # Register signal handler for Ctrl+C
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)

    print("=== RBY1 System Launcher ===")
    print("Checking device connectivity...")
    
    # Check connectivity to Zivid camera and robot
    camera_connected = check_connectivity("192.168.1.101", "Zivid camera")
    robot_connected = check_connectivity("192.168.12.1", "Robot")
    
    if not camera_connected:
        print("⚠️ Warning: Zivid camera not connected. Camera stream and ArUco detection may not work.")
        proceed = input("Do you want to proceed anyway? (y/n): ").lower()
        if proceed != 'y':
            print("Exiting.")
            return
    
    if not robot_connected:
        print("⚠️ Warning: Robot not connected. Joint state publisher may not work.")
        proceed = input("Do you want to proceed anyway? (y/n): ").lower()
        if proceed != 'y':
            print("Exiting.")
            return
    
    print("\nStarting all components...")
    
    # Start camera stream
    camera_cmd = [sys.executable, str(CAMERA_STREAM_PATH)]
    camera_proc = start_process(camera_cmd, "Camera Stream")
    # Give camera time to initialize
    time.sleep(2)
    
    # Start ArUco detector with display
    aruco_cmd = [sys.executable, str(ARUCO_DETECTOR_PATH), "--display"]
    aruco_proc = start_process(aruco_cmd, "ArUco Detector")
    # Give ArUco detector time to initialize
    time.sleep(1)
    
    # Start URDF visualization
    urdf_cmd = "ros2 launch rby1_urdf_visualization display.launch.py model:=urdf/rby1_urdf/model.urdf"
    urdf_proc = start_process(urdf_cmd, "URDF Visualization", shell=True)
    # Give URDF visualization time to initialize
    time.sleep(2)
    
    # Start joint state publisher
    joint_pub_cmd = [sys.executable, str(JOINT_STATE_PUB_PATH)]
    joint_pub_proc = start_process(joint_pub_cmd, "Joint State Publisher")
    
    print("\nAll components started. Press Ctrl+C to stop all processes.")
    
    # Wait for all processes to complete (which they won't unless terminated)
    try:
        for proc in processes:
            proc.wait()
    except KeyboardInterrupt:
        # This should be caught by the signal handler
        pass

if __name__ == "__main__":
    main()
