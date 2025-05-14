#!/usr/bin/env python3

"""
Launch script for the complete RBY1 system with ArUco marker detection.
This script starts each component in a separate Terminator terminal.
"""

import os
import sys
import time
import subprocess
import argparse
from pathlib import Path

# Get the base directory
BASE_DIR = Path(os.path.dirname(os.path.abspath(__file__)))

# Define paths to the components
CAMERA_STREAM_PATH = BASE_DIR / "src/point_cloud_pub/point_cloud_pub/camera_stream_02.py"
ARUCO_DETECTOR_PATH = BASE_DIR / "src/ros2_aruco/ros2_aruco/ros2_aruco/aruco_detector_zivid.py"
JOINT_STATE_PUB_PATH = BASE_DIR / "src/rby1_joint_state_publisher/rby1_joint_state_publisher/rby1_state_pub.py"

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

def launch_in_terminator(commands, titles):
    """Launch commands in Terminator terminals with specified titles."""
    # Check if Terminator is installed
    try:
        subprocess.run(["which", "terminator"], check=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    except subprocess.CalledProcessError:
        print("❌ Terminator is not installed. Please install it with: sudo apt install terminator")
        return False
    
    # Create a script to launch all commands
    script_path = BASE_DIR / "terminator_launch_script.sh"
    with open(script_path, "w") as f:
        f.write("#!/bin/bash\n\n")
        f.write("# Script to launch all components in Terminator\n\n")
        
        # Write commands to create terminals
        f.write("# Launch first terminal\n")
        f.write(f"terminator \
")
        
        # First terminal with first command
        f.write(f"  --title=\"{titles[0]}\" \
")
        f.write(f"  -e \"bash -c 'echo \\\"=== {titles[0]} ===\\\" && {commands[0]}; exec bash'\" \
")
        
        # Add additional terminals
        for i in range(1, len(commands)):
            f.write(f"  --new-tab \
")
            f.write(f"  --title=\"{titles[i]}\" \
")
            f.write(f"  -e \"bash -c \'echo \\\"=== {titles[i]} ===\\\" && {commands[i]}; exec bash\'\" \
")
        
        # End the command
        f.write("  &\n")
    
    # Make script executable
    os.chmod(script_path, 0o755)
    
    # Run the script
    try:
        print(f"Launching Terminator with script: {script_path}")
        subprocess.Popen([str(script_path)], shell=True)
        return True
    except Exception as e:
        print(f"❌ Error launching Terminator: {e}")
        return False

def main():
    print("=== RBY1 System Launcher (Terminator) ===")
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
    
    print("\nStarting all components in Terminator...")
    
    # Define commands and titles
    # Source ROS 2 environment before each command
    source_cmd = "source /opt/ros/humble/setup.bash && source ~/rby1/rby1_ws/install/setup.bash && "
    
    commands = [
        f"{source_cmd} python3 {CAMERA_STREAM_PATH}",
        f"{source_cmd} python3 {ARUCO_DETECTOR_PATH} --display",
        f"{source_cmd} ros2 launch rby1_urdf_visualization display.launch.py model:=urdf/rby1_urdf/model.urdf",
        f"{source_cmd} python3 {JOINT_STATE_PUB_PATH}"
    ]
    
    titles = [
        "Camera Stream",
        "ArUco Detector",
        "URDF Visualization",
        "Joint State Publisher"
    ]
    
    # Launch in Terminator
    success = launch_in_terminator(commands, titles)
    
    if success:
        print("\nAll components launched in Terminator.")
        print("Close the Terminator window to stop all processes.")
    else:
        print("\nFailed to launch components in Terminator.")
        print("Please check if Terminator is installed.")

if __name__ == "__main__":
    main()
