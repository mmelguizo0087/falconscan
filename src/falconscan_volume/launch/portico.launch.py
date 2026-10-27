"""Pórtico REAL completo: LiDAR de velocidad (S2E) + velocity_node + LiDAR de volumen + volume_node.

    ros2 launch falconscan_volume portico.launch.py
    ros2 launch falconscan_volume portico.launch.py volume_lidar:=a1 rviz:=true

Direcciones (D22): LiDAR de velocidad (longitudinal) en 192.168.8.8 y LiDAR de volumen
(transversal) en 192.168.11.2. El host necesita IP en las dos redes (192.168.8.100/24 y
192.168.11.100/24 en la misma interfaz).
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
        DeclareLaunchArgument("velocity_udp_ip", default_value="192.168.8.8"),
        DeclareLaunchArgument("volume_lidar", default_value="s2e", description="s2e o a1"),
        DeclareLaunchArgument("volume_udp_ip", default_value="192.168.11.2"),
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
