"""Pórtico REAL completo: LiDAR de velocidad (S2E) + velocity_node + LiDAR de volumen + volume_node.

    ros2 launch falconscan_volume portico.launch.py
    ros2 launch falconscan_volume portico.launch.py volume_lidar:=a1 rviz:=true

Los dos S2E traen la misma IP de fábrica (192.168.11.2): cambiar la del LiDAR de volumen
(por defecto se espera en 192.168.11.3) antes de conectarlos al mismo switch.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    vel = os.path.join(get_package_share_directory("falconscan_velocity"), "launch", "velocity_s2e.launch.py")
    vol = os.path.join(get_package_share_directory("falconscan_volume"), "launch", "volume_lidar.launch.py")
    arg = LaunchConfiguration
    return LaunchDescription([
        DeclareLaunchArgument("velocity_udp_ip", default_value="192.168.11.2"),
        DeclareLaunchArgument("volume_lidar", default_value="s2e", description="s2e o a1"),
        DeclareLaunchArgument("volume_udp_ip", default_value="192.168.11.3"),
        DeclareLaunchArgument("serial_port", default_value="/dev/ttyUSB0"),
        DeclareLaunchArgument("rviz", default_value="false"),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(vel),
            launch_arguments={"scan_topic": "/lidar_velocity/scan", "udp_ip": arg("velocity_udp_ip")}.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(vol),
            launch_arguments={"lidar": arg("volume_lidar"), "udp_ip": arg("volume_udp_ip"),
                              "serial_port": arg("serial_port"), "rviz": arg("rviz")}.items(),
        ),
    ])
