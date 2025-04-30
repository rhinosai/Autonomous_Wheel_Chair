# fall_detection.launch.py
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    config_path = os.path.join(
    get_package_share_directory('antifall'),
    'config',
    'parameter.yaml'
)
    return LaunchDescription([
        Node(
            package='antifall',
            executable='tof_publisher',
            name='tof_publisher',
            parameters=["config/parameter.yaml"]
        ),
        Node(
            package='antifall',
            executable='fall_alert_node',
            name='fall_alert_node'
        ),

    ])
