# This code is example for mobile control
import rby1_sdk
import numpy as np
import sys
import time
import argparse
import re
from rby1_sdk import *

D2R = np.pi / 180  # Degree to Radian conversion factor
# MINIMUM_TIME = 2.5
MINIMUM_TIME = 1
LINEAR_VELOCITY_LIMIT = 1.5
ANGULAR_VELOCITY_LIMIT = np.pi * 1.5
ACCELERATION_LIMIT = 1.0
STOP_ORIENTATION_TRACKING_ERROR = 1e-5
STOP_POSITION_TRACKING_ERROR = 1e-5
WEIGHT = 0.0015
STOP_COST = 1e-2
VELOCITY_TRACKING_GAIN = 0.01
MIN_DELTA_COST = 1e-4
PATIENCE = 10


def cb(rs):
    print(f"Timestamp: {rs.timestamp - rs.ft_sensor_right.time_since_last_update}")
    position = rs.position * 180 / 3.141592

# move arm up 
def move_arms_up(robot):
    print("example_forward_command")

    # Initialize joint positions
    q_joint_waist = np.zeros(6)
    q_joint_right_arm = np.zeros(7)
    q_joint_left_arm = np.zeros(7)

    q_joint_waist = [0, 0 * D2R, 0 * D2R, 0 * D2R, 0, 0] 
    q_joint_right_arm = [0, -5 * D2R, 0, -120 * D2R, 0, 70 * D2R, 0]
    q_joint_left_arm = [0, 5 * D2R, 0, -120 * D2R, 0, 70 * D2R, 0]

    rc = RobotCommandBuilder().set_command(
        ComponentBasedCommandBuilder()
        .set_body_command(JointPositionCommandBuilder()
            .set_command_header(CommandHeaderBuilder()
            .set_control_hold_time(MINIMUM_TIME))
            .set_minimum_time(MINIMUM_TIME)
            .set_position(q_joint_waist + q_joint_right_arm + q_joint_left_arm)
            ))

    rv = robot.send_command(rc, 10).get()

    if rv.finish_code != RobotCommandFeedback.FinishCode.Ok:
        print("Error: Failed to conduct demo motion.")
        return 1

    return 0



##################### movebase forward ########################## 
def example_forward_command(robot):
    print("example_forward_command")

    # Initialize joint positions
    q_joint_waist = np.zeros(6)
    q_joint_right_arm = np.zeros(7)
    q_joint_left_arm = np.zeros(7)

    # Set specific joint positions
    q_joint_waist = [0, 0 * D2R, 0 * D2R, 0 * D2R, 0, 0] 
    q_joint_right_arm = [0, -5 * D2R, 0, -120 * D2R, 0, 70 * D2R, 0]
    q_joint_left_arm = [0, 5 * D2R, 0, -120 * D2R, 0, 70 * D2R, 0]

    rc = RobotCommandBuilder().set_command(
        ComponentBasedCommandBuilder()

        .set_mobility_command(JointVelocityCommandBuilder()
            .set_command_header(CommandHeaderBuilder()
            .set_control_hold_time(MINIMUM_TIME))
            .set_minimum_time(MINIMUM_TIME)
            .set_velocity([-np.pi] * 2) # joint velocity
            )
    )

    rv = robot.send_command(rc, 10).get()

    if rv.finish_code != RobotCommandFeedback.FinishCode.Ok:
        print("Error: Failed to conduct demo motion.")
        return 1

    return 0

##################################################################


######################## move_arms up ############################

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

##################################################################

######################## move_both_arms_cartesian ################

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

##################################################################

def example_impedance_control_command_1(robot, left_arm_target_pos, right_arm_target_pos, stf_px_l, stf_py_l, stf_pz_l, stf_rr_l, stf_rp_l, stf_ry_l, stf_px_r, stf_py_r, stf_pz_r, stf_rr_r, stf_rp_r, stf_ry_r):
    print("Impedance control command example 1")

    # Create individual transformation matrices from the provided 6-element vectors
    left_target = create_transform(left_arm_target_pos)
    right_target = create_transform(right_arm_target_pos)
    
    print("Left Arm Target Pose relative to link_torso_5:")
    print("Translation:", left_target[:3, 3])
    print("Rotation matrix:\n", left_target[:3, :3])
    
    print("Right Arm Target Pose relative to link_torso_5:")
    print("Translation:", right_target[:3, 3])
    print("Rotation matrix:\n", right_target[:3, :3])
    
    # # Initialize transformation matrices
    # T_torso = np.eye(4)
    # T_right = np.eye(4)
    # T_left = np.eye(4)

    # # Define transformation matrices
    # angle = np.pi / 6
    # T_torso[:3, :3] = np.array([[np.cos(angle), 0, np.sin(angle)],
    #                             [0, 1, 0],
    #                             [-np.sin(angle), 0, np.cos(angle)]])
    # T_torso[:3, 3] = [0.1, 0, 1.2]

    # angle = -np.pi / 4
    # T_right[:3, :3] = np.array([[np.cos(angle), 0, np.sin(angle)],
    #                             [0, 1, 0],
    #                             [-np.sin(angle), 0, np.cos(angle)]])
    # T_right[:3, 3] = [0.45, -0.4, -0.1]

    # T_left[:3, :3] = np.array([[np.cos(angle), 0, np.sin(angle)],
    #                            [0, 1, 0],
    #                            [-np.sin(angle), 0, np.cos(angle)]])
    # T_left[:3, 3] = [0.45, 0.4, -0.1]

    # Build commands
    # torso_command = ImpedanceControlCommandBuilder().set_command_header(
    #     CommandHeaderBuilder().set_control_hold_time(MINIMUM_TIME)
    # ).set_reference_link_name("base").set_link_name("link_torso_5").set_translation_weight(
    #     [1000, 1000, 1000]).set_rotation_weight([100, 100, 100]).set_transformation(T_torso)

    right_arm_command = ImpedanceControlCommandBuilder().set_command_header(
        CommandHeaderBuilder().set_control_hold_time(MINIMUM_TIME)
    ).set_reference_link_name("link_torso_5").set_link_name("ee_right").set_translation_weight(
        [stf_px_r, stf_py_r, stf_pz_r]).set_rotation_weight([stf_rr_r, stf_rp_r, stf_ry_r]).set_transformation(right_target)

    left_arm_command = ImpedanceControlCommandBuilder().set_command_header(
        CommandHeaderBuilder().set_control_hold_time(MINIMUM_TIME)
    ).set_reference_link_name("link_torso_5").set_link_name("ee_left").set_translation_weight(
        [stf_px_l, stf_py_l, stf_pz_l]).set_rotation_weight([stf_rr_l, stf_rp_l, stf_ry_l]).set_transformation(left_target)

    # Send command
    rc = RobotCommandBuilder().set_command(
        ComponentBasedCommandBuilder().set_body_command(
            BodyComponentBasedCommandBuilder()
 #           .set_torso_command(torso_command)
            .set_right_arm_command(right_arm_command)
            .set_left_arm_command(left_arm_command)
        )
    )

    rv = robot.send_command(rc, 10).get()

    if rv.finish_code != RobotCommandFeedback.FinishCode.Ok:
        print("Error: Failed to conduct demo motion.")
        return 1

    return 0


############################ main ################################

def main(address, power_device, servo):
    print("Attempting to connect to the robot...")

    robot = rby1_sdk.create_robot_a(address)

    if not robot.connect():
        print("Error: Unable to establish connection to the robot at")
        sys.exit(1)

    print("Successfully connected to the robot")

    print("Starting state update...")
    robot.start_state_update(cb, 0.1)

    robot.set_parameter("default.acceleration_limit_scaling", "0.8")
    robot.set_parameter("joint_position_command.cutoff_frequency", "5")
    robot.set_parameter("cartesian_command.cutoff_frequency", "5")
    robot.set_parameter("default.linear_acceleration_limit", "5")
    # robot.set_time_scale(1.0)

    print("parameters setting is done")

    if not robot.is_connected():
        print("Robot is not connected")
        exit(1)
        
    if not robot.is_power_on(power_device):
        rv = robot.power_on(power_device)
        if not rv:
            print("Failed to power on")
            exit(1)
            
    print(servo)
    if not robot.is_servo_on(servo):
        rv = robot.servo_on(servo)
        if not rv:
            print("Fail to servo on")
            exit(1)

    
    control_manager_state = robot.get_control_manager_state()

    if (control_manager_state.state == rby1_sdk.ControlManagerState.State.MinorFault or control_manager_state.state == rby1_sdk.ControlManagerState.State.MajorFault):

        if control_manager_state.state == rby1_sdk.ControlManagerState.State.MajorFault:
            print("Warning: Detected a Major Fault in the Control Manager!!!!!!!!!!!!!!!.")
        else:
            print("Warning: Detected a Minor Fault in the Control Manager@@@@@@@@@@@@@@@@.")

        print("Attempting to reset the fault...")
        if not robot.reset_fault_control_manager():
            print("Error: Unable to reset the fault in the Control Manager.")
            sys.exit(1)
        print("Fault reset successfully.")

    print("Control Manager state is normal. No faults detected.")

    print("Enabling the Control Manager...")
    if not robot.enable_control_manager():
        print("Error: Failed to enable the Control Manager.")
        sys.exit(1)
    print("Control Manager enabled successfully.")



    if not move_arms_up(robot):
        print("finish motion")
    if not example_forward_command(robot):
         print("finish motion")
    if not move_both_arms_cartesian(robot, (0.5,0.14,-0.1,0,0,-1.5708), (0.5,-0.14,-0.1,0,0,1.5708)):
        print("finish motion")
    if not move_both_arms_cartesian(robot, (0.5,0.14,-0.18,0,0,-1.5708), (0.5,-0.14,-0.18,0,0,1.5708)):
        print("finish motion")
    
    # i=0
    # while(i<100):
    #     if not example_impedance_control_command_1(robot, (0.5,0.18,-0.18,0,0,-1.5708), (0.5,-0.18,-0.18,0,0,1.5708), 100, 100, 1000, 50, 100, 50, 100, 100, 1000, 50, 100, 50):
    #         print("finish motion")
    #     i=i+1


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="07_impedance_control")
    parser.add_argument('--address', type=str, required=True, help="Robot address")
    parser.add_argument('--device', type=str, default=".*", help="Power device name regex pattern (default: '.*')")
    parser.add_argument('--servo', type=str, default=".*", help="Servo name regex pattern (default: '.*')")
    args = parser.parse_args()


    main(address=args.address,
         power_device=args.device,
         servo = args.servo)
