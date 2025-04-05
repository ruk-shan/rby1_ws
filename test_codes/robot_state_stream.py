import time

import rby1_sdk
import argparse
import numpy as np


def callback(robot_state, control_manager_state: rby1_sdk.ControlManagerState):
    np.set_printoptions(precision=3, suppress=True, floatmode='fixed')
    joint_position = robot_state.position
    joint_position_dict = {
            "right_wheel":joint_position[0],
            "left_wheel":joint_position[1],
            "torso_0":joint_position[2],
            "torso_1":joint_position[3],
            "torso_2":joint_position[4],
            "torso_3":joint_position[5],
            "torso_4":joint_position[6],
            "torso_5":joint_position[7],
            "right_arm_0":joint_position[8],
            "right_arm_1":joint_position[9],
            "right_arm_2":joint_position[10],
            "right_arm_3":joint_position[11],
            "right_arm_4":joint_position[12],
            "right_arm_5":joint_position[13],
            "right_arm_6":joint_position[14],
            "left_arm_0":joint_position[15],
            "left_arm_1":joint_position[16],
            "left_arm_2":joint_position[17],
            "left_arm_3":joint_position[18],
            "left_arm_4":joint_position[19],
            "left_arm_5":joint_position[20],
            "left_arm_6":joint_position[21],
            "head_0":joint_position[22],
            "head_1":joint_position[23],
            "gripper_finger_r1":joint_position[24],
            "gripper_finger_r2":joint_position[25],
            "gripper_finger_l1":joint_position[26],
            "gripper_finger_l2":joint_position[27]
            }
    # print(control_manager_state)
    # print(f"{joint_position_dict}", end="\r", flush=True)
    print (joint_position)


# or
# def callback(robot_state):
#     print(robot_state)


# def main(address, power_device):
def main():
    address = "localhost:50051"
    power_device = ".*"
    robot = rby1_sdk.create_robot_a(address)
    robot.connect()
    if not robot.is_connected():
        print("Robot is not connected")
        exit(1)
    if not robot.is_power_on(power_device):
        rv = robot.power_on(power_device)
        if not rv:
            print("Failed to power on")
            exit(1)

    robot.start_state_update(callback,
                             rate=10  # Hz
                             )
    try:
        time.sleep(100)
    except KeyboardInterrupt:
        print("Stopping state update...")
    finally:
        robot.stop_state_update()


if __name__ == "__main__":
   main()
