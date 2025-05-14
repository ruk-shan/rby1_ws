import rby1_sdk
import numpy as np
import sys
import time
import argparse
from rby1_sdk import *

# Constants (adjust as needed)
LINEAR_VELOCITY_LIMIT = 1.5
ANGULAR_VELOCITY_LIMIT = np.pi * 1.5
ACCELERATION_LIMIT = 1.0
MINIMUM_TIME = 2.5
STOP_ORIENTATION_TRACKING_ERROR = 1e-5
STOP_POSITION_TRACKING_ERROR = 1e-5

def move_gripper_cartesian(robot):
    # Create a target transformation for the gripper.
    # This is a 4x4 homogeneous transformation matrix.
    T_gripper = np.eye(4)
    # For example, set the target position (in meters) relative to the "base" frame.
    # Here, we move the gripper to x=0.5, y=-0.3, z=1.0. Adjust these as needed.
    T_gripper[:3, 3] = [0.5, 0, 1.2]
    # The rotation is left as identity (no rotation). Modify T_gripper[:3, :3] if a specific orientation is desired.

    # Build the Cartesian command for the gripper (right arm end-effector)
    command = CartesianCommandBuilder() \
        .add_target("base", "ee_right", T_gripper,
                    LINEAR_VELOCITY_LIMIT, ANGULAR_VELOCITY_LIMIT, ACCELERATION_LIMIT) \
        .set_minimum_time(MINIMUM_TIME) \
        .set_stop_position_tracking_error(STOP_POSITION_TRACKING_ERROR) \
        .set_stop_orientation_tracking_error(STOP_ORIENTATION_TRACKING_ERROR)

    # Wrap the command in a RobotCommand and send it.
    rc = RobotCommandBuilder().set_command(
        ComponentBasedCommandBuilder().set_body_command(command)
    )
    
    rv = robot.send_command(rc, 10).get()
    if rv.finish_code != RobotCommandFeedback.FinishCode.Ok:
        print("Error: Failed to move gripper in Cartesian space.")
    else:
        print("Gripper moved successfully in Cartesian space.")

def main(address, power_device, servo):
    print("Connecting to the robot...")
    robot = rby1_sdk.create_robot_a(address)
    if not robot.connect():
        print("Error: Unable to connect to the robot.")
        sys.exit(1)
    print("Connected to the robot.")

    # Ensure the robot is powered on
    if not robot.is_power_on(power_device):
        if not robot.power_on(power_device):
            print("Error: Failed to power on.")
            sys.exit(1)

    # Ensure the servos are turned on
    if not robot.is_servo_on(servo):
        if not robot.servo_on(servo):
            print("Error: Failed to servo on.")
            sys.exit(1)

    # Enable the control manager
    if not robot.enable_control_manager():
        print("Error: Failed to enable the Control Manager.")
        sys.exit(1)
    print("Control Manager enabled.")

    # Move the gripper in Cartesian space
    move_gripper_cartesian(robot)
    time.sleep(MINIMUM_TIME + 1)  # wait a bit for the motion to complete

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Move the robot gripper in Cartesian space")
    parser.add_argument('--address', type=str, required=True, help="Robot address (e.g., tcp://192.168.1.10:50051)")
    parser.add_argument('--device', type=str, default=".*", help="Power device regex pattern (default: '.*')")
    parser.add_argument('--servo', type=str, default=".*", help="Servo regex pattern (default: '.*')")
    args = parser.parse_args()

    main(args.address, args.device, args.servo)
