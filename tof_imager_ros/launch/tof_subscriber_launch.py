from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='tof_imager_ros',
            executable='tof_imager_subscriber',
            name='tof_subscriber'
        )
    ]) 