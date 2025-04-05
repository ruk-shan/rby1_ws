import rby1_sdk
import numpy as np
import sys
import time
import argparse
from rby1_sdk import *

# Constants
D2R = np.pi / 180
MINIMUM_TIME = 2.5
LINEAR_VELOCITY_LIMIT = 1.5
ANGULAR_VELOCITY_LIMIT = np.pi * 1.5
ACCELERATION_LIMIT = 1.0
STOP_ORIENTATION_TRACKING_ERROR = 1e-5
STOP_POSITION_TRACKING_ERROR = 1e-5

def move_left_arm_cartesian(robot, target_xyz=None):
    """
    Send a Cartesian command to move the left arm (end-effector "ee_left") relative to "link_torso_5".
    If target_xyz is provided (as [x, y, z]), that becomes the desired target position in the torso frame.
    Otherwise, the target defaults to moving upward by 0.1 m along the torso’s z-axis.
    """
    # Since get_link_pose is unavailable, we construct a target transform from identity.
    T_target = np.eye(4)
    if target_xyz is not None:
        T_target[:3, 3] = np.array(target_xyz)
    else:
        T_target[2, 3] = 0.1  # default: raise by 0.1 m
    print("Target Left Arm Pose relative to link_torso_5:", T_target[:3, 3])
    
    # Build Cartesian command relative to "link_torso_5"
    command = CartesianCommandBuilder()\
        .add_target("link_torso_5", "ee_left", T_target,
                    LINEAR_VELOCITY_LIMIT, ANGULAR_VELOCITY_LIMIT, ACCELERATION_LIMIT)\
        .set_minimum_time(MINIMUM_TIME)\
        .set_stop_position_tracking_error(STOP_POSITION_TRACKING_ERROR)\
        .set_stop_orientation_tracking_error(STOP_ORIENTATION_TRACKING_ERROR)
    
    rc = RobotCommandBuilder().set_command(
        ComponentBasedCommandBuilder().set_body_command(command)
    )
    
    rv = robot.send_command(rc, 10).get()
    if rv.finish_code != RobotCommandFeedback.FinishCode.Ok:
        print("Error: Failed to move left arm in Cartesian space (relative to link_torso_5).")
    else:
        print("Left arm moved successfully in Cartesian space (relative to link_torso_5).")

def check_and_clear_faults(robot):
    """
    Checks the Control Manager state and, if a fault is detected, attempts to clear it.
    """
    control_manager_state = robot.get_control_manager_state()
    if (control_manager_state.state == rby1_sdk.ControlManagerState.State.MinorFault or 
        control_manager_state.state == rby1_sdk.ControlManagerState.State.MajorFault):
        if control_manager_state.state == rby1_sdk.ControlManagerState.State.MajorFault:
            print("Warning: Major Fault detected in Control Manager.")
        else:
            print("Warning: Minor Fault detected in Control Manager.")
        print("Attempting to reset the fault...")
        if not robot.reset_fault_control_manager():
            print("Error: Could not reset fault in Control Manager.")
            sys.exit(1)
        print("Fault reset successfully.")
    else:
        print("No faults detected in Control Manager.")

def main(address, power_device, servo, target_xyz):
    print("Connecting to the robot...")
    robot = rby1_sdk.create_robot_a(address)
    if not robot.connect():
        print("Error: Unable to connect to the robot.")
        sys.exit(1)
    print("Connected to the robot.")

    # Ensure the robot is powered on.
    if not robot.is_power_on(power_device):
        if not robot.power_on(power_device):
            print("Error: Failed to power on.")
            sys.exit(1)

    # Ensure servos are turned on.
    if not robot.is_servo_on(servo):
        if not robot.servo_on(servo):
            print("Error: Failed to servo on.")
            sys.exit(1)

    # Check and clear any faults before sending commands.
    check_and_clear_faults(robot)

    # Enable the Control Manager.
    if not robot.enable_control_manager():
        print("Error: Failed to enable the Control Manager.")
        sys.exit(1)
    print("Control Manager enabled successfully.")

    # Send Cartesian command to move the left arm relative to "link_torso_5"
    move_left_arm_cartesian(robot, target_xyz=target_xyz)
    
    # Wait for the motion to complete.
    time.sleep(MINIMUM_TIME + 1)

if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="Move left arm in Cartesian space relative to link_torso_5. "
                    "Specify the target position using --target (x y z in meters) in the link_torso_5 frame."
    )
    parser.add_argument('--address', type=str, required=True, help="Robot address (e.g., tcp://192.168.1.10:50051)")
    parser.add_argument('--device', type=str, default=".*", help="Power device regex pattern (default: '.*')")
    parser.add_argument('--servo', type=str, default=".*", help="Servo regex pattern (default: '.*')")
    parser.add_argument('--target', type=float, nargs=3, default=None,
                        help="Target Cartesian position (x y z in meters) for the left arm's end-effector in the link_torso_5 frame. "
                             "If not provided, the arm will move upward by 0.1 m.")
    args = parser.parse_args()

    main(args.address, args.device, args.servo, args.target)
