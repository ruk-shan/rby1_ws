#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os
import json


def generate_launch_description():
    # Set default values
    default_urdf_path = os.path.join(get_package_share_directory('rby1_visualizer'), 'rby1_urdf', 'model.urdf')
    default_reference_link = 'base'
    default_target_links = [
        'link_right_arm_3',  # Right arm end effector
        'link_left_arm_3',   # Left arm end effector
        'link_head_2',       # Head
        'link_torso_5',      # Torso
        'wheel_r',           # Right wheel
        'wheel_l'            # Left wheel
    ]
    
    # Declare launch arguments
    urdf_path_arg = DeclareLaunchArgument(
        'urdf_path',
        default_value=default_urdf_path,
        description='Path to the URDF file'
    )
    
    reference_link_arg = DeclareLaunchArgument(
        'reference_link',
        default_value=default_reference_link,
        description='Reference link for forward kinematics calculations'
    )
    
    target_links_arg = DeclareLaunchArgument(
        'target_links',
        default_value=json.dumps(default_target_links),
        description='List of target links to calculate poses for (JSON array format)'
    )
    
    publish_rate_arg = DeclareLaunchArgument(
        'publish_rate',
        default_value='10.0',
        description='Rate at which to publish poses (Hz)'
    )
    
    # Create the FK solver node
    fk_solver_node = Node(
        package='rby1_visualizer',
        executable='fk_solver.py',
        name='forward_kinematics_solver',
        output='screen',
        parameters=[{
            'urdf_path': LaunchConfiguration('urdf_path'),
            'reference_link': LaunchConfiguration('reference_link'),
            'target_links': LaunchConfiguration('target_links'),
            'publish_rate': LaunchConfiguration('publish_rate'),
        }]
    )
    
    # Return the launch description
    return LaunchDescription([
        urdf_path_arg,
        reference_link_arg,
        target_links_arg,
        publish_rate_arg,
        fk_solver_node
    ])
