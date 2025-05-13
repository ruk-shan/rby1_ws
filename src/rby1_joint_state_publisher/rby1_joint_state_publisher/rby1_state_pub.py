import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import time
import math
import socket  # Import the socket module for TCP/IP
import rby1_sdk

class SimpleJointStatePublisher(Node):

    def __init__(self):
        super().__init__('simple_joint_state_publisher')
        self.publisher_ = self.create_publisher(JointState, 'joint_states', 10)
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0.0
        self.robot_port = 50051        # Hardcoded robot port
        self.robot_socket = None       # Socket object, initialized to None

        self.connect_to_robot() # Call the connection method during initialization

        self.joint_names = [
            "right_wheel",
            "left_wheel",
            "torso_0",
            "torso_1",
            "torso_2",
            "torso_3",
            "torso_4",
            "torso_5",
            "right_arm_0",
            "right_arm_1",
            "right_arm_2",
            "right_arm_3",
            "right_arm_4",
            "right_arm_5",
            "right_arm_6",
            "left_arm_0",
            "left_arm_1",
            "left_arm_2",
            "left_arm_3",
            "left_arm_4",
            "left_arm_5",
            "left_arm_6",
            "head_0",
            "head_1",
            "gripper_finger_r1",
            "gripper_finger_r2",
            "gripper_finger_l1",
            "gripper_finger_l2",
            
        ]

    def connect_to_robot(self):
        address = "localhost:50051"
        # address = "192.168.12.1:50051"
        power_device = ".*"
        self.robot = rby1_sdk.create_robot_a(address)
        self.robot.connect()
        if not self.robot.is_connected():
            print("Robot is not connected")
            exit(1)
        if not self.robot.is_power_on(power_device):
            rv = self.robot.power_on(power_device)
            if not rv:
                print("Failed to power on")
                exit(1)

    # def receive_joint_states(self):
    def get_states(self):
        if self.robot_socket: 
            self.np.set_printoptions(precision=3, suppress=True, floatmode='fixed')
            print(self.robot_state)
            return self.robot_state


    def timer_callback(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.joint_names  

        # Get joint positions from the dummy robot communication
        # robot_state = self.robot.get_state()
        # print(robot_state)


        joint_positions = self.robot.get_state().position
        # print (len(joint_positions))
        
        new_elements = [0.0, 0.0, 0.0 , 0.0]
        all_joint_positions = list(joint_positions) + new_elements 
        print (len(all_joint_positions))
        print (all_joint_positions)


        msg.position = all_joint_positions # Set positions from robot data
        # print (len(all_joint_positions))

        # msg.velocity = []  # Optional, can be empty if not publishing velocity
        # msg.effort = []    # Optional, can be empty if not publishing effort

        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing Joint States for {len(self.joint_names)} joints') # Optional logging


def main(args=None):
    rclpy.init(args=args)
    simple_joint_state_publisher = SimpleJointStatePublisher()
    rclpy.spin(simple_joint_state_publisher)
    simple_joint_state_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()