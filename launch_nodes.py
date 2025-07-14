#!/usr/bin/env python3

import subprocess
import os
import time

def run_command_in_terminal(command, title):
    """Run a command in a new terminal window with a given title"""
    return subprocess.Popen(
        ['xterm', '-T', title, '-e', 'bash', '-c', f'echo "{title}" && {command}; exec bash'],
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE
    )

def main():
    # Get the workspace directory
    workspace_dir = os.path.dirname(os.path.abspath(__file__))
    
    # Source the ROS 2 setup file
    source_cmd = f'source {workspace_dir}/install/setup.bash && '
    
    # List of commands to run with their respective titles
    commands = [
        {
            'command': f'cd {workspace_dir} && {source_cmd} python3 src/rby1_joint_state_publisher/rby1_joint_state_publisher/rby1_state_pub.py',
            'title': 'Joint State Publisher'
        },
        {
            'command': f'cd {workspace_dir} && {source_cmd} ros2 launch rby1_urdf_visualization display.launch.py model:=urdf/rby1_urdf/model.urdf',
            'title': 'URDF Visualization'
        },
        # {
        #     'command': f'cd {workspace_dir} && {source_cmd} python3 src/rby1_joint_state_publisher/rby1_joint_state_publisher/marker_pub.py',
        #     'title': 'Marker Publisher'
        # },
        {
            'command': f'cd {workspace_dir} && {source_cmd} ros2 run rby1_joint_state_publisher odom_publisher',
            'title': 'Odometry Publisher'
        },
        # {
        #     'command': f'cd {workspace_dir} && {source_cmd} python3 src/kinematics_solver/kinematics_solver/fk_solver_03.py',
        #     'title': 'FK Solver'
        # },
        # {
        #     'command': f'cd {workspace_dir} && {source_cmd} python3 src/kinematics_solver/kinematics_solver/tf_api_server.py',
        #     'title': 'TF API Server'
        # }
    ]

    processes = []
    
    try:
        # Launch each command in a new terminal
        for cmd in commands:
            print(f"Starting {cmd['title']}...")
            processes.append(run_command_in_terminal(cmd['command'], cmd['title']))
            time.sleep(1)  # Small delay between starting processes
            
        print("\nAll nodes have been launched in separate terminal windows.")
        print("Press Ctrl+C in this terminal to terminate all processes.")
        
        # Keep the script running until interrupted
        while True:
            time.sleep(1)
            
    except KeyboardInterrupt:
        print("\nShutting down all processes...")
        for process in processes:
            process.terminate()
        print("All processes have been terminated.")

if __name__ == "__main__":
    main()
