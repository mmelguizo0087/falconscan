"""Nodo ROS 2: /scan (LaserScan del LiDAR de velocidad) -> /velocity_estimate.

Toda la lógica vive en ``falconscan_velocity.core``; este archivo solo hace de
puente con ROS. Los parámetros se declaran automáticamente a partir de
``VelocityParams``, así que config/velocity.yaml y el código no se desincronizan.
"""
from __future__ import annotations

import math

import numpy as np
import rclpy
from builtin_interfaces.msg import Time as TimeMsg
from falconscan_interfaces.msg import VelocityEstimate as VelocityMsg
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32MultiArray

from .core import Scan, VelocityEstimator, VelocityParams


def _to_sec(t: TimeMsg) -> float:
    return t.sec + t.nanosec * 1e-9


def _to_time(s: float) -> TimeMsg:
    sec, nsec = divmod(int(round(s * 1e9)), 1_000_000_000)
    return TimeMsg(sec=sec, nanosec=nsec)


class VelocityNode(Node):
    def __init__(self) -> None:
        super().__init__("velocity_node")
        defaults = VelocityParams()
        values = {}
        for name in VelocityParams.field_names():
            values[name] = self.declare_parameter(name, getattr(defaults, name)).value
        self.publish_profile = self.declare_parameter("publish_profile", False).value

        self.params = VelocityParams(**values)
        self.estimator = VelocityEstimator(self.params)

        self.pub = self.create_publisher(VelocityMsg, "velocity_estimate", 10)
        self.pub_profile = (self.create_publisher(Float32MultiArray, "height_profile", 5)
                            if self.publish_profile else None)
        self.create_subscription(LaserScan, "scan", self.on_scan, qos_profile_sensor_data)
        self.get_logger().info(
            f"velocity_node listo: H={self.params.mount_height_m} m, ROI x=[{self.params.roi_x_min_m}, "
            f"{self.params.roi_x_max_m}] m, dx={self.params.grid_dx_m} m, baseline={self.params.baseline_scans}")

    def on_scan(self, msg: LaserScan) -> None:
        scan = Scan(
            stamp=_to_sec(msg.header.stamp),
            angle_min=msg.angle_min,
            angle_increment=msg.angle_increment,
            ranges=np.asarray(msg.ranges, dtype=np.float32),
            time_increment=msg.time_increment,
            range_min=msg.range_min,
            range_max=msg.range_max,
        )
        e = self.estimator.process(scan)

        out = VelocityMsg()
        out.header.frame_id = msg.header.frame_id
        out.header.stamp = _to_time(e.mid_stamp) if math.isfinite(e.mid_stamp) else msg.header.stamp
        out.scan_stamp = msg.header.stamp
        out.status = int(e.status)
        out.status_text = e.status.name
        out.valid = e.valid
        out.velocity_mps = float(e.velocity_mps)
        out.velocity_filtered_mps = float(e.velocity_filtered_mps)
        out.displacement_m = float(e.displacement_m)
        out.dt_s = float(e.dt_s)
        out.shift_bins = float(e.shift_bins)
        out.peak_correlation = float(e.peak)
        out.peak_prominence = float(e.prominence)
        out.n_overlap = int(e.n_overlap)
        x0, x1 = e.object_extent if e.object_extent else (math.nan, math.nan)
        out.object_x_start_m, out.object_x_end_m = float(x0), float(x1)
        self.pub.publish(out)

        if self.pub_profile is not None and self.estimator.last_profile is not None:
            self.pub_profile.publish(Float32MultiArray(data=self.estimator.last_profile.astype(np.float32).tolist()))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = VelocityNode()
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
