Generete marker 
ros2 run ros2_aruco aruco_generate_marker 

# Generate marker with ID 2
ros2 run ros2_aruco aruco_generate_marker --id 2

# Generate larger marker (400x400 pixels)
ros2 run ros2_aruco aruco_generate_marker --size 400

# Generate marker with different dictionary
ros2 run ros2_aruco aruco_generate_marker --dictionary DICT_4X4_50

run usb cam node 
ros2 run usb_cam usb_cam_node_exe

run node aruco_node 
ros2 run ros2_aruco aruco_node