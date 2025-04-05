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

def create_transform(target_vec):
    """
    Convert a 6-element vector [x, y, z, rx, ry, rz] into a 4x4 homogeneous transform.
    The rotation angles (rx, ry, rz) are assumed to be in radians and follow the roll-pitch-yaw convention.
    """
    T = np.eye(4)
    # Set translation
    T[0:3, 3] = target_vec[0:3]
    # Extract Euler angles
    roll, pitch, yaw = target_vec[3], target_vec[4], target_vec[5]
    
    # Compute rotation matrices for each axis
    Rx = np.array([[1, 0, 0],
                   [0, np.cos(roll), -np.sin(roll)],
                   [0, np.sin(roll),  np.cos(roll)]])
    
    Ry = np.array([[ np.cos(pitch), 0, np.sin(pitch)],
                   [0,              1, 0],
                   [-np.sin(pitch), 0, np.cos(pitch)]])
    
    Rz = np.array([[np.cos(yaw), -np.sin(yaw), 0],
                   [np.sin(yaw),  np.cos(yaw), 0],
                   [0,            0,           1]])
    
    # Combined rotation: R = Rz * Ry * Rx
    R = Rz @ Ry @ Rx
    T[0:3, 0:3] = R
    return T

def move_both_arms_cartesian(robot, left_arm_target_pos, right_arm_target_pos):
    """
    Send a Cartesian command to move both arms (end-effectors "ee_left" and "ee_right")
    relative to "link_torso_5". Each target is defined as a 6-element vector: [x, y, z, rx, ry, rz].
    """
    # Create individual transformation matrices from the provided 6-element vectors
    left_target = create_transform(left_arm_target_pos)
    right_target = create_transform(right_arm_target_pos)
    
    print("Left Arm Target Pose relative to link_torso_5:")
    print("Translation:", left_target[:3, 3])
    print("Rotation matrix:\n", left_target[:3, :3])
    
    print("Right Arm Target Pose relative to link_torso_5:")
    print("Translation:", right_target[:3, 3])
    print("Rotation matrix:\n", right_target[:3, :3])
    
    # Build Cartesian command with two targets
    command = CartesianCommandBuilder()\
        .add_target("link_torso_5", "ee_left", left_target,
                    LINEAR_VELOCITY_LIMIT, ANGULAR_VELOCITY_LIMIT, ACCELERATION_LIMIT)\
        .add_target("link_torso_5", "ee_right", right_target,
                    LINEAR_VELOCITY_LIMIT, ANGULAR_VELOCITY_LIMIT, ACCELERATION_LIMIT)\
        .set_minimum_time(MINIMUM_TIME)\
        .set_stop_position_tracking_error(STOP_POSITION_TRACKING_ERROR)\
        .set_stop_orientation_tracking_error(STOP_ORIENTATION_TRACKING_ERROR)
    
    rc = RobotCommandBuilder().set_command(
        ComponentBasedCommandBuilder().set_body_command(command)
    )
    
    rv = robot.send_command(rc, 10).get()
    if rv.finish_code != RobotCommandFeedback.FinishCode.Ok:
        print("Error: Failed to move both arms in Cartesian space (relative to link_torso_5).")
    else:
        print("Both arms moved successfully in Cartesian space (relative to link_torso_5).")

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

def main(address, power_device, servo, left_arm_target_pos, right_arm_target_pos):
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

    # Send Cartesian command to move both arms relative to "link_torso_5"
    move_both_arms_cartesian(robot, left_arm_target_pos, right_arm_target_pos)
    
    # Wait for the motion to complete.
    time.sleep(MINIMUM_TIME + 1)

if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="Move both arms in Cartesian space relative to link_torso_5. "
                    "Specify target positions using --left_target and --right_target (x y z rx ry rz in meters and radians) for the left and right arms respectively. "
                    "If not provided, default positions are used."
    )
    parser.add_argument('--address', type=str, required=True, help="Robot address (e.g., tcp://192.168.1.10:50051)")
    parser.add_argument('--device', type=str, default=".*", help="Power device regex pattern (default: '.*')")
    parser.add_argument('--servo', type=str, default=".*", help="Servo regex pattern (default: '.*')")
    parser.add_argument('--left_target', type=float, nargs=6, default=[0.5, 0.2, -0.2, 0.0, 0.0, 0.0],
                        help="Target Cartesian pose for the left arm's end-effector in the link_torso_5 frame: x y z rx ry rz.")
    parser.add_argument('--right_target', type=float, nargs=6, default=[0.5, -0.2, -0.2, 0.0, 0.0, 0.0],
                        help="Target Cartesian pose for the right arm's end-effector in the link_torso_5 frame: x y z rx ry rz.")
    args = parser.parse_args()

    main(args.address, args.device, args.servo, args.left_target, args.right_target)
