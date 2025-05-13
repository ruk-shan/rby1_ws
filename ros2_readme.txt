packages 

sudo apt install ros-humble-xacro
sudo apt install ros-humble-joint-state-publisher-gui
pip install rby1-sdk

launch  test_urdf
ros2 launch rby1_urdf_visualization display.launch.py model:=urdf/test_urdf/01-myfirst.urdf

launch rby1 without joint-state-publisher-gui
ros2 launch rby1_urdf_visualization display.launch.py model:=urdf/rby1_urdf/model.urdf

run joint state pub 
ip is hardcoded
python3 src/rby1_joint_state_publisher/rby1_joint_state_publisher/rby1_state_pub.py


launch rby1 with joint-state-publisher-gui
ros2 launch rby1_urdf_visualization display_with_joint_gui.launch.py  model:=urdf/rby1_urdf/model.urdf


run point_cloud_pub
ros2 run point_cloud_pub point_cloud_pub 

 
pkg rby1_joint_state_pub 
    rby1_state_pub.py >  config file (to get ip address, ect.) is located at src folder and path is hardcoded. Pls update manually. 


### installing "ros2_ros_bt_py" ##

https://github.com/fzi-forschungszentrum-informatik/ros2_ros_bt_py
https://fzi-forschungszentrum-informatik.github.io/ros2_ros_bt_py/index.html


run as python file 
python3 cartesian_move_left.py --address localhost:50051 --target 0.5 02 -0.1

run as a python file 
shan@shan-Precision-7670:~/mywork/RBY1/rby1_ws/src/rby1_joint_state_publisher/rby1_joint_state_publisher$ python3 marker_pub.py 
