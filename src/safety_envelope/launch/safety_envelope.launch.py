"""
Launch file for the safety envelope node.

This launch file starts the safety envelope node with parameters loaded from
the default_boundaries.yaml configuration file.
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    """Generate launch description for the safety envelope node."""
    
    # Get the package share directory
    pkg_dir = get_package_share_directory('safety_envelope')
    
    # Path to the configuration file
    config_file = os.path.join(pkg_dir, 'config', 'default_boundaries.yaml')
    
    return LaunchDescription([
        Node(
            package='safety_envelope',
            executable='safety_envelope_node',
            name='safety_envelope',
            output='screen',
            parameters=[config_file]
        )
    ])
