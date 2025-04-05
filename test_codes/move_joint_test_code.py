import rby1_sdk
import numpy as np
import sys
import time
import argparse
from rby1_sdk import *

# Constants
D2R = np.pi / 180          # Degrees to radians
MINIMUM_TIME = 2.5         # Time to execute each motion

def move_each_joint_individually(robot):
    # Define the list of joints you want to control.
    # In our examples, only the torso and arms (20 joints) are controlled.
    controlled_joints = [
        "torso_0", "torso_1", "torso_2", "torso_3", "torso_4", "torso_5",
        "right_arm_0", "right_arm_1", "right_arm_2", "right_arm_3", "right_arm_4", "right_arm_5", "right_arm_6",
        "left_arm_0", "left_arm_1", "left_arm_2", "left_arm_3", "left_arm_4", "left_arm_5", "left_arm_6"
    ]
    num_joints = len(controlled_joints)
    target_angle = 30 * D2R  # 10° in radians

    for i, joint in enumerate(controlled_joints):
        print(f"Moving joint '{joint}' to {target_angle:.3f} rad")
        # Create a command vector with zeros and set only the i-th joint to the target angle.
        q = np.zeros(num_joints)
        q[i] = target_angle

        # Build and send the joint position command.
        command = JointPositionCommandBuilder().set_position(q).set_minimum_time(MINIMUM_TIME)
        rc = RobotCommandBuilder().set_command(
            ComponentBasedCommandBuilder().set_body_command(command)
        )
        rv = robot.send_command(rc, 10).get()

        if rv.finish_code != RobotCommandFeedback.FinishCode.Ok:
            print(f"Error moving joint '{joint}'")
        else:
            print(f"Joint '{joint}' moved to {target_angle:.3f} rad")
        time.sleep(2)  # Wait 2 seconds for the motion to complete

        # Now return the joint back to 0.
        print(f"Returning joint '{joint}' to 0 rad")
        q[i] = 0.0
        command = JointPositionCommandBuilder().set_position(q).set_minimum_time(MINIMUM_TIME)
        rc = RobotCommandBuilder().set_command(
            ComponentBasedCommandBuilder().set_body_command(command)
        )
        rv = robot.send_command(rc, 10).get()

        if rv.finish_code != RobotCommandFeedback.FinishCode.Ok:
            print(f"Error returning joint '{joint}' to zero")
        else:
            print(f"Joint '{joint}' returned to 0 rad")
        time.sleep(2)

def main(address, power_device, servo):
    print("Connecting to the robot...")
    robot = rby1_sdk.create_robot_a(address)

    if not robot.connect():
        print("Error: Unable to connect to the robot.")
        sys.exit(1)
    print("Successfully connected.")

    # Power on the robot if it isn't already.
    if not robot.is_power_on(power_device):
        if not robot.power_on(power_device):
            print("Error: Failed to power on.")
            sys.exit(1)

    # Turn on the servo if it isn't on.
    if not robot.is_servo_on(servo):
        if not robot.servo_on(servo):
            print("Error: Failed to servo on.")
            sys.exit(1)

    # Enable the control manager.
    if not robot.enable_control_manager():
        print("Error: Failed to enable the Control Manager.")
        sys.exit(1)
    print("Control Manager enabled.")

    # Move each joint individually.
    move_each_joint_individually(robot)
    print("Finished moving joints individually.")

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Move each joint individually")
    parser.add_argument('--address', type=str, required=True, help="Robot address (e.g., tcp://192.168.1.10:50051)")
    parser.add_argument('--device', type=str, default=".*", help="Power device regex pattern (default: '.*')")
    parser.add_argument('--servo', type=str, default=".*", help="Servo regex pattern (default: '.*')")
    args = parser.parse_args()

    main(args.address, args.device, args.servo)
