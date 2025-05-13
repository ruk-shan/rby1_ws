# ROS2 ArUco Marker Detector

A ROS2 package for detecting ArUco markers from camera feeds and publishing their poses, transformations, and visualizations.

## Overview

This package provides tools for:
1. Detecting ArUco markers from ROS2 camera topics
2. Publishing marker poses and transformations
3. Visualizing markers in RViz2 with 3D axes
4. Generating custom ArUco markers

## Requirements

- ROS2 (tested on Humble)
- OpenCV with ArUco module
- Python 3.8+
- numpy
- scipy

## Input Topics

The detector subscribes to the following topics:

1. **Camera Image** (default: `/image_raw`)
   - Type: `sensor_msgs/Image`
   - Description: RGB camera feed containing ArUco markers

2. **Camera Info** (automatically derived from camera topic)
   - Type: `sensor_msgs/CameraInfo`
   - Description: Camera calibration parameters
   - Note: Automatically determined by appending `/camera_info` to the base camera topic path

## Output Topics

The detector publishes to the following topics:

1. **ArUco Marker JSON** (`/aruco_markers_json`)
   - Type: `std_msgs/String`
   - Description: JSON-formatted data containing all detected markers with their IDs, positions, and rotations

2. **ArUco Marker Poses** (`/aruco_markers_poses`)
   - Type: `geometry_msgs/PoseArray`
   - Description: Array of poses (position and orientation) for all detected markers

3. **ArUco Marker Visualization** (`/aruco_markers_visual`)
   - Type: `visualization_msgs/MarkerArray`
   - Description: Visualization markers for RViz2, including:
     - Cubes at marker positions
     - Text showing marker IDs
     - Colored arrows showing X (red), Y (green), and Z (blue) axes

4. **TF Transforms** (`/tf`)
   - Type: `tf2_msgs/TFMessage`
   - Description: Coordinate transformations from camera frame to each marker

## Usage

### Running the ArUco Detector

```bash
# Source ROS2
source /opt/ros/humble/setup.bash

# Run the detector with default settings
python3 /path/to/aruco_detector_simple.py

# Run with custom camera topic
python3 /path/to/aruco_detector_simple.py --camera-topic /my_camera/image_raw

# Run with display window showing detections
python3 /path/to/aruco_detector_simple.py --display

# Run with custom ArUco dictionary
python3 /path/to/aruco_detector_simple.py --dict DICT_5X5_100

# Run with custom marker size (in meters)
python3 /path/to/aruco_detector_simple.py --marker-size 0.08
```

### Command-line Arguments

- `--camera-topic`: ROS2 camera topic to subscribe to (default: `/image_raw`)
- `--dict`: ArUco dictionary to use (default: `DICT_4X4_50`)
- `--display`: Display detection results in a window
- `--marker-size`: Size of the ArUco marker in meters (default: 0.05)

### Generating ArUco Markers

```bash
# Generate a single marker
python3 /path/to/aruco_generate_markers.py --id 1 --size 200 --output marker1.png

# Generate a marker with white border
python3 /path/to/aruco_generate_markers.py --id 2 --white-border 20

# Generate a grid of markers
python3 /path/to/aruco_generate_markers.py --grid 2x2 --first-id 10
```

## Visualization in RViz2

To visualize the markers in RViz2:

1. Run RViz2:
   ```bash
   rviz2
   ```

2. Add the following displays:
   - Add a `MarkerArray` display and set the topic to `/aruco_markers_visual`
   - Add a `TF` display to see the marker coordinate frames
   - Add an `Image` display to see the camera feed (topic: your camera topic)

3. Set the fixed frame to `camera` in the Global Options

## Troubleshooting

- If markers are not being detected, check that:
  - The camera is publishing to the expected topic
  - The marker size is set correctly
  - The correct ArUco dictionary is selected
  - The markers are clearly visible in the camera view

- If visualization is not appearing in RViz2, check that:
  - The correct topics are selected in RViz2
  - The fixed frame is set to `camera`
  - The markers are being detected (check console output)

## License

This package is provided under the MIT License.
