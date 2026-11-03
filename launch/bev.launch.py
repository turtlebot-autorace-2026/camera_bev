"""Only the BEV processor; does not start a camera, bag, bridge, or controller."""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = Path(get_package_share_directory('camera_bev'))
    defaults = dict(input_topic='/camera_raw_16_9',
                    calibration_file=str(share / 'config/camera_1280x720.yaml'),
                    settings_file=str(share / 'config/estimated.yaml'), use_sim_time='false')
    arguments = [DeclareLaunchArgument(key, default_value=value) for key, value in defaults.items()]
    return LaunchDescription(arguments + [Node(
        package='camera_bev', executable='bev_node', name='camera_bev', output='screen',
        parameters=[{key: LaunchConfiguration(key) for key in defaults}])])
