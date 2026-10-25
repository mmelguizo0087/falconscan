"""LiDAR de volumen REAL (driver sllidar_ros2) + volume_node.

    ros2 launch falconscan_volume volume_lidar.launch.py                      # S2E por UDP
    ros2 launch falconscan_volume volume_lidar.launch.py lidar:=a1 serial_port:=/dev/ttyUSB0

Necesita velocity_node corriendo (velocity_s2e.launch.py con scan_topic:=/lidar_velocity/scan),
o usar portico.launch.py, que lanza los dos.

OJO con dos S2E en el mismo switch: los dos traen 192.168.11.2 de fábrica; hay que cambiar la
IP de uno (udp_ip) antes de conectarlos juntos.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, LaunchConfigurationEquals
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    cfg = os.path.join(get_package_share_directory("falconscan_velocity"), "config", "falconscan.yaml")
    rviz_cfg = os.path.join(get_package_share_directory("falconscan_volume"), "rviz", "volumen.rviz")
    arg = LaunchConfiguration
    scan_topic = arg("scan_topic")
    return LaunchDescription([
        DeclareLaunchArgument("lidar", default_value="s2e", description="s2e (UDP) o a1 (serie)"),
        DeclareLaunchArgument("scan_topic", default_value="/lidar_volume/scan"),
        DeclareLaunchArgument("frame_id", default_value="lidar_volume"),
        DeclareLaunchArgument("udp_ip", default_value="192.168.11.3"),
        DeclareLaunchArgument("udp_port", default_value="8089"),
        DeclareLaunchArgument("serial_port", default_value="/dev/ttyUSB0"),
        DeclareLaunchArgument("serial_baudrate", default_value="115200"),
        DeclareLaunchArgument("inverted", default_value="false"),
        DeclareLaunchArgument("rviz", default_value="false"),
        Node(
            package="sllidar_ros2", executable="sllidar_node", name="sllidar_volume",
            condition=LaunchConfigurationEquals("lidar", "s2e"),
            parameters=[{
                "channel_type": "udp",
                "udp_ip": arg("udp_ip"),
                "udp_port": ParameterValue(arg("udp_port"), value_type=int),
                "frame_id": arg("frame_id"),
                "inverted": ParameterValue(arg("inverted"), value_type=bool),
                "angle_compensate": True,
                "scan_mode": "Sensitivity",
            }],
            remappings=[("scan", scan_topic)], output="screen",
        ),
        Node(
            package="sllidar_ros2", executable="sllidar_node", name="sllidar_volume",
            condition=LaunchConfigurationEquals("lidar", "a1"),
            parameters=[{
                "channel_type": "serial",
                "serial_port": arg("serial_port"),
                "serial_baudrate": ParameterValue(arg("serial_baudrate"), value_type=int),
                "frame_id": arg("frame_id"),
                "inverted": ParameterValue(arg("inverted"), value_type=bool),
                "angle_compensate": True,
            }],
            remappings=[("scan", scan_topic)], output="screen",
        ),
        Node(
            package="falconscan_volume", executable="volume_node", name="volume_node",
            parameters=[cfg], remappings=[("scan", scan_topic)], output="screen",
        ),
        Node(
            package="tf2_ros", executable="static_transform_publisher", name="tf_seccion",
            arguments=["--x", "1.5", "--frame-id", "falconscan_paso", "--child-frame-id", arg("frame_id")],
            condition=IfCondition(arg("rviz")),
        ),
        Node(
            package="rviz2", executable="rviz2", name="rviz2", arguments=["-d", rviz_cfg],
            condition=IfCondition(arg("rviz")),
        ),
    ])
