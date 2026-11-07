"""
Launch file for Pure Pursuit Controller - Racing Mode

This launch file starts the pure pursuit controller with racing configuration.
Optimized for multi-car racing scenarios.
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python import get_package_share_directory
import os


def generate_launch_description():
    """Generate launch description for racing mode pure pursuit."""
    ld = LaunchDescription()

    # Get the package share directory
    package_share_dir = get_package_share_directory("pure_pursuit")

    # Path to the racing configuration file
    param_path = os.path.join(package_share_dir, "config", "params.yaml")

    # Pure pursuit node with racing parameters
    pure_pursuit_racing = Node(
        package='pure_pursuit',
        executable='pure_pursuit_node',
        name='pure_pursuit_node',
        parameters=[param_path],
        output='screen',
        emulate_tty=True,
        arguments=['--ros-args', '--log-level', 'info']
    )

    ld.add_action(pure_pursuit_racing)

    return ld
