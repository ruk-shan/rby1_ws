packages 

sudo apt install ros-humble-xacro
sudo apt install ros-humble-joint-state-publisher-gui
pip install rby1-sdk

launch  test_urdf
ros2 launch rby1_urdf_visualization display.launch.py model:=urdf/test_urdf/01-myfirst.urdf

launch rby1
ros2 launch rby1_urdf_visualization display.launch.py model:=urdf/rby1_urdf/model.urdf


pkg rby1_joint_state_pub 
    rby1_state_pub.py >  config file (to get ip address, ect.) is located at src folder and path is hardcoded. Pls update manually. 


### installing "ros2_ros_bt_py"

https://github.com/fzi-forschungszentrum-informatik/ros2_ros_bt_py
https://fzi-forschungszentrum-informatik.github.io/ros2_ros_bt_py/index.html



