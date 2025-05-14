#!/bin/bash

# Script to launch all RBY1 components in separate gnome-terminal windows

# Base directory
BASE_DIR="$HOME/rby1/rby1_ws"

# Define paths to components
CAMERA_STREAM_PATH="$BASE_DIR/src/point_cloud_pub/point_cloud_pub/camera_stream_02.py"
ARUCO_DETECTOR_PATH="$BASE_DIR/src/ros2_aruco/ros2_aruco/ros2_aruco/aruco_detector_zivid.py"
JOINT_STATE_PUB_PATH="$BASE_DIR/src/rby1_joint_state_publisher/rby1_joint_state_publisher/rby1_state_pub.py"

# Source ROS 2 environment
source_cmd="source /opt/ros/humble/setup.bash && source $BASE_DIR/install/setup.bash"

# Check connectivity
echo "=== RBY1 System Launcher ==="
echo "Checking device connectivity..."

# Check Zivid camera
echo "Checking connectivity to Zivid camera (192.168.1.101)..."
if ping -c 2 -W 1 192.168.1.101 > /dev/null; then
    echo "✅ Zivid camera is connected!"
    camera_connected=true
else
    echo "⚠️ Warning: Zivid camera not responding. Camera stream and ArUco detection may not work."
    read -p "Do you want to proceed anyway? (y/n): " proceed
    if [[ $proceed != "y" ]]; then
        echo "Exiting."
        exit 1
    fi
    camera_connected=false
fi

# Check robot
echo "Checking connectivity to Robot (192.168.12.1)..."
if ping -c 2 -W 1 192.168.12.1 > /dev/null; then
    echo "✅ Robot is connected!"
    robot_connected=true
else
    echo "⚠️ Warning: Robot not responding. Joint state publisher may not work."
    read -p "Do you want to proceed anyway? (y/n): " proceed
    if [[ $proceed != "y" ]]; then
        echo "Exiting."
        exit 1
    fi
    robot_connected=false
fi

echo -e "\nStarting all components in separate terminals..."

# Launch camera stream
gnome-terminal --title="Camera Stream" -- bash -c "$source_cmd && echo '=== Camera Stream ===' && python3 $CAMERA_STREAM_PATH; exec bash"
echo "✅ Camera Stream launched"
sleep 2

# Launch ArUco detector
gnome-terminal --title="ArUco Detector" -- bash -c "$source_cmd && echo '=== ArUco Detector ===' && python3 $ARUCO_DETECTOR_PATH --display; exec bash"
echo "✅ ArUco Detector launched"
sleep 1

# Launch URDF visualization
gnome-terminal --title="URDF Visualization" -- bash -c "$source_cmd && echo '=== URDF Visualization ===' && ros2 launch rby1_urdf_visualization display.launch.py model:=urdf/rby1_urdf/model.urdf; exec bash"
echo "✅ URDF Visualization launched"
sleep 2

# Launch joint state publisher
gnome-terminal --title="Joint State Publisher" -- bash -c "$source_cmd && echo '=== Joint State Publisher ===' && python3 $JOINT_STATE_PUB_PATH; exec bash"
echo "✅ Joint State Publisher launched"

echo -e "\nAll components launched in separate terminals."
echo "Close each terminal window to stop the respective component."
