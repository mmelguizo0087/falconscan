"""Pórtico completo en simulación: sim_portico (los dos LiDAR) + velocity_node + volume_node + RViz.

    ros2 launch falconscan_volume volume_sim.launch.py
    ros2 launch falconscan_volume volume_sim.launch.py speed_kmh:=5.0 direction:=-1
    ros2 launch falconscan_volume volume_sim.launch.py lane_shift_m:=0.9 rviz:=false

Los dos nodos leen el mismo archivo de parámetros (falconscan_velocity/config/falconscan.yaml).
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

from falconscan_velocity.sim import SIM_VELOCITY_OVERRIDES
from falconscan_volume.sim import SIM_VOLUME_OVERRIDES


def generate_launch_description():
    cfg = os.path.join(get_package_share_directory("falconscan_velocity"), "config", "falconscan.yaml")
    rviz_cfg = os.path.join(get_package_share_directory("falconscan_volume"), "rviz", "volumen.rviz")
    arg = LaunchConfiguration

    def f(name):
        return ParameterValue(arg(name), value_type=float)

    return LaunchDescription([
        DeclareLaunchArgument("rviz", default_value="true"),
        DeclareLaunchArgument("speed_kmh", default_value="10.0"),
        DeclareLaunchArgument("direction", default_value="0", description="+1, -1 o 0 (alterna)"),
        DeclareLaunchArgument("yaw_deg", default_value="0.0"),
        DeclareLaunchArgument("y_offset_m", default_value="0.0"),
        DeclareLaunchArgument("lane_shift_m", default_value="0.0"),
        DeclareLaunchArgument("gap_s", default_value="4.0"),
        Node(
            package="falconscan_volume", executable="sim_portico", name="sim_portico",
            parameters=[{"speed_kmh": f("speed_kmh"), "direction": ParameterValue(arg("direction"), value_type=int),
                         "yaw_deg": f("yaw_deg"), "y_offset_m": f("y_offset_m"),
                         "lane_shift_m": f("lane_shift_m"), "gap_s": f("gap_s")}],
        ),
        Node(
            package="falconscan_velocity", executable="velocity_node", name="velocity_node",
            parameters=[cfg, SIM_VELOCITY_OVERRIDES], remappings=[("scan", "/lidar_velocity/scan")], output="screen",
        ),
        Node(
            package="falconscan_volume", executable="volume_node", name="volume_node",
            parameters=[cfg, SIM_VOLUME_OVERRIDES], remappings=[("scan", "/lidar_volume/scan")], output="screen",
        ),
        # El perfil transversal en vivo se dibuja 1.5 m delante del frente de la nube
        Node(
            package="tf2_ros", executable="static_transform_publisher", name="tf_seccion",
            arguments=["--x", "1.5", "--frame-id", "falconscan_paso", "--child-frame-id", "lidar_volume"],
        ),
        Node(
            package="rviz2", executable="rviz2", name="rviz2", arguments=["-d", rviz_cfg],
            condition=IfCondition(arg("rviz")),
        ),
    ])
