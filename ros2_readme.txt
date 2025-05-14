cd ~/rby1/rby1_ws
./launch_rby1_gnome.sh


python3 /home/rby1/rby1/rby1_ws/src/rby1_joint_state_publisher/rby1_joint_state_publisher/rby1_state_pub.py

packages 

sudo apt install ros-humble-xacro
sudo apt install ros-humble-joint-state-publisher-gui
pip install rby1-sdk

#launch  test_urdf
ros2 launch rby1_urdf_visualization display.launch.py model:=urdf/test_urdf/01-myfirst.urdf

#launch rby1 with out joint-state-publisher-gui
ros2 launch rby1_urdf_visualization display.launch.py model:=urdf/rby1_urdf/model.urdf

python3 ~/home/rby1/rby1/rby1_ws/src/point_cloud_pub/point_cloud_pub/camera_stream_02.py


#launch rby1 with joint-state-publisher-gui
ros2 launch rby1_urdf_visualization display_with_joint_gui.launch.py  model:=urdf/rby1_urdf/model.urdf


run point_cloud_pub
ros2 run point_cloud_pub point_cloud_pub 

run joint state pub 
ip is hardcoded
python3 src/rby1_joint_state_publisher/rby1_joint_state_publisher/rby1_state_pub.py 




pkg rby1_joint_state_pub 
    rby1_state_pub.py >  config file (to get ip address, ect.) is located at src folder and path is hardcoded. Pls update manually. 


### installing "ros2_ros_bt_py" ##

https://github.com/fzi-forschungszentrum-informatik/ros2_ros_bt_py
https://fzi-forschungszentrum-informatik.github.io/ros2_ros_bt_py/index.html


run as python file 
python3 cartesian_move_left.py --address localhost:50051 --target 0.5 02 -0.1

run as a python file 
shan@shan-Precision-7670:~/mywork/RBY1/rby1_ws/src/rby1_joint_state_publisher/rby1_joint_state_publisher$ python3 marker_pub.py 

### Full System Launch Instructions

## Option 1: Launch All Components in Separate GNOME Terminals (Recommended)

```bash
# Make the script executable (first time only)
chmod +x ~/rby1/rby1_ws/launch_rby1_gnome.sh

# Run the launcher script
cd ~/rby1/rby1_ws
./launch_rby1_gnome.sh
```

This script will:
1. Check connectivity to the Zivid camera (192.168.1.101) and robot (192.168.12.1)
2. Launch each component in a separate labeled terminal window
3. Properly source the ROS 2 environment for each component

## Option 2: Launch All Components in a Single Terminal

```bash
# Make the script executable (first time only)
chmod +x ~/rby1/rby1_ws/launch_rby1_system.py

# Run the launcher script
cd ~/rby1/rby1_ws
python3 launch_rby1_system.py
```

This script will launch all components in a single terminal. Press Ctrl+C to stop all processes.

## Option 3: Launch Components Individually

# Start the camera stream
python3 /home/rby1/rby1/rby1_ws/src/point_cloud_pub/point_cloud_pub/camera_stream_02.py

# Run the ArUco marker detector
python3 /home/rby1/rby1/rby1_ws/src/ros2_aruco/ros2_aruco/ros2_aruco/aruco_detector_zivid.py --display

# Launch the RBY1 URDF visualization
ros2 launch rby1_urdf_visualization display.launch.py model:=urdf/rby1_urdf/model.urdf

# Start the joint state publisher
python3 src/rby1_joint_state_publisher/rby1_joint_state_publisher/rby1_state_pub.py

## Monitoring and Debugging

# To check camera topics
ros2 topic list | grep camera
ros2 topic echo /camera/color/image_raw --no-arr | head -n 5

# To view ArUco marker topics
ros2 topic list | grep aruco
ros2 topic echo /aruco_markers_json --no-arr | head -n 5

# To view TF transforms for markers
ros2 run tf2_ros tf2_echo zivid_camera_frame aruco_marker_0

## Using RViz2 for Visualization

# Start RViz2
ros2 run rviz2 rviz2

# In RViz2 GUI:
# 1. Set Fixed Frame to "zivid_camera_frame"
# 2. Add MarkerArray display for "/aruco_markers_viz"
# 3. Add Image display for "/camera/color/image_raw"
# 4. Add TF display to see marker transforms

### Visualizing camera_stream_02.py in RViz2

1. Run the camera node:
   ```bash
   python3 src/point_cloud_pub/point_cloud_pub/camera_stream_02.py
   ```
   or
   ```bash
   ros2 run point_cloud_pub camera_stream_02.py
   ```

2. In a new terminal, launch RViz2:
   ```bash
   rviz2
   ```

3. In RViz2:
   - Click "Add" in the bottom-left corner
   - Select "Image" from the list
   - In the Image display properties:
     - Set "Image Topic" to `/camera/color/image_raw`
     - Set "Color Scheme" to "RGB8" or "RGBA8"
     - Set "Fixed Frame" to `zivid_camera_frame`

4. Verify the image is being published:
   ```bash
   ros2 topic list | grep image
   ros2 topic hz /camera/color/image_raw


   
   ```
