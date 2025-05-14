import time

import rby1_sdk
import argparse


def callback(robot_state: rby1_sdk.RobotState_A, control_manager_state: rby1_sdk.ControlManagerState):
    # print(robot_state.position)
    # print (robot_state.position[0])
    joint_position = robot_state.position
    print (len(joint_position))
    joint_position_dict = {
    "right_wheel": float(joint_position[0]),
    "left_wheel": float(joint_position[1]),
    "torso_0": float(joint_position[2]),
    "torso_1": float(joint_position[3]),
    "torso_2": float(joint_position[4]),
    "torso_3": float(joint_position[5]),
    "torso_4": float(joint_position[6]),
    "torso_5": float(joint_position[7]),
    "right_arm_0": float(joint_position[8]),
    "right_arm_1": float(joint_position[9]),
    "right_arm_2": float(joint_position[10]),
    "right_arm_3": float(joint_position[11]),
    "right_arm_4": float(joint_position[12]),
    "right_arm_5": float(joint_position[13]),
    "right_arm_6": float(joint_position[14]),
    "left_arm_0": float(joint_position[15]),
    "left_arm_1": float(joint_position[16]),
    "left_arm_2": float(joint_position[17]),
    "left_arm_3": float(joint_position[18]),
    "left_arm_4": float(joint_position[19]),
    "left_arm_5": float(joint_position[20]),
    "left_arm_6": float(joint_position[21]),
    "head_0": float(joint_position[22]),
    "head_1": float(joint_position[23])}
    # "gripper_finger_r1": float(joint_position[24]),
    # "gripper_finger_r2": float(joint_position[25]),
    # "gripper_finger_l1": float(joint_position[26]),
    # "gripper_finger_l2": float(joint_position[27])
# }

    # print(control_manager_state)
    # print(f"{joint_position_dict}", end="\r", flush=True)
    print (joint_position_dict)

    # print(control_manager_state)


# or
# def callback(robot_state: rby1_sdk.RobotState_A):
#     print(robot_state)


def main(address, power_device):
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
    parser = argparse.ArgumentParser(description="04_robot_state_stream")
    parser.add_argument('--address', type=str, required=True, help="Robot address")
    parser.add_argument('--device', type=str, default=".*", help="Power device name regex pattern (default: '.*')")
    args = parser.parse_args()

    main(address=args.address,
         power_device=args.device)
