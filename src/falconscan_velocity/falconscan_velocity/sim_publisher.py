"""Nodo ROS 2 que publica /scan simulado: permite probar velocity_node sin hardware.

    ros2 run falconscan_velocity sim_publisher --ros-args -p truck_speed_mps:=1.5
    ros2 run falconscan_velocity sim_publisher --ros-args -p alternate_direction:=true   # +x, −x, +x…
"""
from __future__ import annotations

import math

import numpy as np
import rclpy
from builtin_interfaces.msg import Time as TimeMsg
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

from .sim import LidarSim, LidarSimConfig, TruckMotion


class SimPublisher(Node):
    def __init__(self) -> None:
        super().__init__("lidar_sim")
        self.v = self.declare_parameter("truck_speed_mps", 1.5).value
        self.gap = self.declare_parameter("gap_between_trucks_s", 4.0).value
        self.alternate = self.declare_parameter("alternate_direction", False).value
        hz = self.declare_parameter("scan_hz", 10.0).value
        self.frame_id = self.declare_parameter("frame_id", "lidar_velocity").value
        self.sim = LidarSim(LidarSimConfig(scan_hz=hz))
        self.t = 0.0
        self.period = 1.0 / hz
        self.pass_dur = 26.5 / max(self.v, 1e-3)
        self.pub = self.create_publisher(LaserScan, "scan", 10)
        self.create_timer(self.period, self.tick)

    def tick(self) -> None:
        ciclo = self.pass_dur + self.gap
        t0_ciclo = self.t - self.t % ciclo
        # Con alternate_direction los camiones impares van hacia −x (entran por x máx)
        v = -self.v if self.alternate and int(round(t0_ciclo / ciclo)) % 2 else self.v
        motion = TruckMotion(x0=(-10.0 if v > 0 else 16.5) - v * t0_ciclo, v=v)
        s = self.sim.scan_at(self.t, motion)
        stamp = self.get_clock().now().to_msg()
        msg = LaserScan()
        msg.header.stamp = stamp
        msg.header.frame_id = self.frame_id
        msg.angle_min = float(s.angle_min)
        msg.angle_increment = float(s.angle_increment)
        msg.angle_max = float(s.angle_min + (len(s.ranges) - 1) * s.angle_increment)
        msg.time_increment = float(s.time_increment)
        msg.scan_time = self.period
        msg.range_min = float(s.range_min)
        msg.range_max = float(s.range_max)
        msg.ranges = np.asarray(s.ranges, dtype=np.float32).tolist()
        self.pub.publish(msg)
        self.t += self.period


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SimPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
