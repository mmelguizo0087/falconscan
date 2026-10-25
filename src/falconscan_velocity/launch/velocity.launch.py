"""Lanza velocity_node. Con use_sim:=true también lanza el LiDAR simulado.

    ros2 launch falconscan_velocity velocity.launch.py
    ros2 launch falconscan_velocity velocity.launch.py use_sim:=true truck_speed:=2.0
    ros2 launch falconscan_velocity velocity.launch.py scan_topic:=/lidar_velocity/scan
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    cfg = os.path.join(get_package_share_directory("falconscan_velocity"), "config", "falconscan.yaml")
    use_sim = LaunchConfiguration("use_sim")
    scan_topic = LaunchConfiguration("scan_topic")
    return LaunchDescription([
        DeclareLaunchArgument("use_sim", default_value="false"),
        DeclareLaunchArgument("scan_topic", default_value="/scan"),
        DeclareLaunchArgument("truck_speed", default_value="1.5"),
        Node(
            package="falconscan_velocity",
            executable="sim_publisher",
            name="lidar_sim",
            condition=IfCondition(use_sim),
            parameters=[{"truck_speed_mps": ParameterValue(LaunchConfiguration("truck_speed"), value_type=float)}],
            remappings=[("scan", scan_topic)],
        ),
        Node(
            package="falconscan_velocity",
            executable="velocity_node",
            name="velocity_node",
            parameters=[cfg],
            remappings=[("scan", scan_topic)],
            output="screen",
        ),
    ])
