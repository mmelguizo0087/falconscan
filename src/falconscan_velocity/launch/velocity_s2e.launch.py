"""Lanza el driver del RPLIDAR S2E (sllidar_ros2, UDP) y velocity_node juntos.

    ros2 launch falconscan_velocity velocity_s2e.launch.py
    ros2 launch falconscan_velocity velocity_s2e.launch.py udp_ip:=192.168.8.8 inverted:=true
    ros2 launch falconscan_velocity velocity_s2e.launch.py scan_topic:=/lidar_velocity/scan

El LiDAR longitudinal (velocidad) está en 192.168.8.8 (D22): el PC debe tener IP en esa red
(192.168.8.100/24; la RPi la tiene en eth0 junto con 192.168.11.100/24 del transversal).
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    velocity_launch = os.path.join(
        get_package_share_directory("falconscan_velocity"), "launch", "velocity.launch.py")
    scan_topic = LaunchConfiguration("scan_topic")
    return LaunchDescription([
        DeclareLaunchArgument("scan_topic", default_value="/scan"),
        DeclareLaunchArgument("udp_ip", default_value="192.168.8.8"),
        DeclareLaunchArgument("udp_port", default_value="8089"),
        DeclareLaunchArgument("frame_id", default_value="lidar_velocity"),
        DeclareLaunchArgument("inverted", default_value="false"),
        DeclareLaunchArgument("scan_mode", default_value="Sensitivity"),
        Node(
            package="sllidar_ros2",
            executable="sllidar_node",
            name="sllidar_velocity",
            parameters=[{
                "channel_type": "udp",
                "udp_ip": LaunchConfiguration("udp_ip"),
                "udp_port": ParameterValue(LaunchConfiguration("udp_port"), value_type=int),
                "frame_id": LaunchConfiguration("frame_id"),
                "inverted": ParameterValue(LaunchConfiguration("inverted"), value_type=bool),
                "angle_compensate": True,
                "scan_mode": LaunchConfiguration("scan_mode"),
            }],
            remappings=[("scan", scan_topic)],
            output="screen",
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(velocity_launch),
            launch_arguments={"use_sim": "false", "scan_topic": scan_topic}.items(),
        ),
    ])
