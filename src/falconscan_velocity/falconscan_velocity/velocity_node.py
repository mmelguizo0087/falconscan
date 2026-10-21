"""Nodo ROS 2: /scan (LaserScan del LiDAR de velocidad) -> /velocity_estimate, /pass_state, /pass_event.

Toda la lógica vive en ``falconscan_velocity.core``; este archivo solo hace de
puente con ROS. Los parámetros se declaran automáticamente a partir de
``VelocityParams``, así que config/velocity.yaml y el código no se desincronizan.
"""
from __future__ import annotations

import math

import numpy as np
import rclpy
from builtin_interfaces.msg import Time as TimeMsg
from falconscan_interfaces.msg import PassEvent as PassEventMsg
from falconscan_interfaces.msg import PassState as PassStateMsg
from falconscan_interfaces.msg import VelocityEstimate as VelocityMsg
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32MultiArray

from .core import PassEvent, PassTracker, Scan, VelocityParams, VelocityPipeline


def _to_sec(t: TimeMsg) -> float:
    return t.sec + t.nanosec * 1e-9


def _to_time(s: float) -> TimeMsg:
    sec, nsec = divmod(int(round(s * 1e9)), 1_000_000_000)
    return TimeMsg(sec=sec, nanosec=nsec)


def pass_state_msg(tracker: PassTracker, stamp: TimeMsg, frame_id: str) -> PassStateMsg:
    m = PassStateMsg()
    m.header.stamp, m.header.frame_id = stamp, frame_id
    m.state = int(tracker.state)
    m.state_text = tracker.state.name
    m.pass_open = tracker.open_pass
    m.pass_id = tracker.pass_id if tracker.open_pass else 0
    m.ready = tracker.ready
    return m


def pass_event_msg(ev: PassEvent, stamp: TimeMsg, frame_id: str, sign: int = 1) -> PassEventMsg:
    """`sign` = referencia de signo del usuario (VelocityParams.velocity_sign)."""
    ev = ev.with_sign(sign)
    m = PassEventMsg()
    m.header.stamp, m.header.frame_id = stamp, frame_id
    m.pass_id = ev.pass_id
    m.event = PassEventMsg.EVENT_INICIO if ev.event == "INICIO" else PassEventMsg.EVENT_FIN
    m.event_text = ev.event
    m.valid = ev.valid
    m.t_start = _to_time(ev.t_start)
    m.t_end = _to_time(ev.t_end) if math.isfinite(ev.t_end) else TimeMsg()
    m.duration_s = float(ev.duration_s)
    m.flags = int(ev.flags)
    m.flag_names = ev.flags.nombres()
    m.n_gaps, m.n_scans, m.n_valid = ev.n_gaps, ev.n_scans, ev.n_valid
    m.v_median_mps, m.v_min_mps, m.v_max_mps = float(ev.v_median_mps), float(ev.v_min_mps), float(ev.v_max_mps)
    m.direction = int(ev.direction)
    m.advance_median_mps = float(ev.advance_median_mps)
    m.advance_min_mps, m.advance_max_mps = float(ev.advance_min_mps), float(ev.advance_max_mps)
    m.start_x_start_m, m.start_x_end_m = (float(v) for v in (ev.ext_start or (math.nan, math.nan)))
    m.end_x_start_m, m.end_x_end_m = (float(v) for v in (ev.ext_end or (math.nan, math.nan)))
    return m


class VelocityNode(Node):
    def __init__(self) -> None:
        super().__init__("velocity_node")
        defaults = VelocityParams()
        values = {}
        for name in VelocityParams.field_names():
            values[name] = self.declare_parameter(name, getattr(defaults, name)).value
        self.publish_profile = self.declare_parameter("publish_profile", False).value

        self.params = VelocityParams(**values)
        # Estimador + máquina de paso con la política de reinicio común (core/pipeline.py)
        self.pipeline = VelocityPipeline(self.params)
        self.estimator, self.tracker = self.pipeline.estimator, self.pipeline.tracker
        self.frame_id = ""

        self.pub = self.create_publisher(VelocityMsg, "velocity_estimate", 10)
        self.pub_state = self.create_publisher(PassStateMsg, "pass_state", 10)
        # Los eventos son pocos y no deben perderse: confiables y con cola amplia
        self.pub_event = self.create_publisher(
            PassEventMsg, "pass_event", QoSProfile(depth=50, reliability=ReliabilityPolicy.RELIABLE))
        self.pub_profile = (self.create_publisher(Float32MultiArray, "height_profile", 5)
                            if self.publish_profile else None)
        self.create_subscription(LaserScan, "scan", self.on_scan, qos_profile_sensor_data)
        # SIN_DATOS (T11) se detecta aunque no lleguen escaneos
        self.create_timer(max(0.02, self.params.scan_timeout_s / 5), self.on_tick)
        self._publish_state(self.get_clock().now().to_msg())
        self.get_logger().info(
            f"velocity_node listo: H={self.params.mount_height_m} m, ROI x=[{self.params.roi_x_min_m}, "
            f"{self.params.roi_x_max_m}] m, dx={self.params.grid_dx_m} m, baseline={self.params.baseline_scans}, "
            f"sentidos={'ambos' if self.params.pass_both_directions else 'solo +x'}, "
            f"positivo={'hacia −x' if self.params.invert_velocity_sign else 'hacia +x'}")

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
        self.frame_id = msg.header.frame_id
        prev = self.tracker.state
        e, events = self.pipeline.process(scan, ahora=self._now())

        out = VelocityMsg()
        out.header.frame_id = msg.header.frame_id
        out.header.stamp = _to_time(e.mid_stamp) if math.isfinite(e.mid_stamp) else msg.header.stamp
        out.scan_stamp = msg.header.stamp
        out.status = int(e.status)
        out.status_text = e.status.name
        out.valid = e.valid
        es = e.with_sign(self.params.velocity_sign)     # el tracker ya usó e en el marco mundo
        out.velocity_mps = float(es.velocity_mps)
        out.velocity_filtered_mps = float(es.velocity_filtered_mps)
        out.displacement_m = float(es.displacement_m)
        out.dt_s = float(e.dt_s)
        out.shift_bins = float(es.shift_bins)
        out.peak_correlation = float(e.peak)
        out.peak_prominence = float(e.prominence)
        out.n_overlap = int(e.n_overlap)
        x0, x1 = e.object_extent if e.object_extent else (math.nan, math.nan)
        out.object_x_start_m, out.object_x_end_m = float(x0), float(x1)
        out.pass_id = self.tracker.pass_id if self.tracker.open_pass else 0
        d = self.tracker.direction                       # marco mundo; FIN ya lo dejó en 0
        out.direction = d * self.params.velocity_sign
        out.advance_mps = float(d * e.velocity_mps) if d and e.valid else math.nan
        self.pub.publish(out)
        self._publish_events(events, msg.header.stamp)
        self._publish_state(msg.header.stamp)
        if self.tracker.state != prev:
            self.get_logger().info(f"estado de paso: {prev.name} -> {self.tracker.state.name}")

        if self.pub_profile is not None and self.estimator.last_profile is not None:
            self.pub_profile.publish(Float32MultiArray(data=self.estimator.last_profile.astype(np.float32).tolist()))


    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def on_tick(self) -> None:
        prev = self.tracker.state
        events = self.pipeline.tick(self._now())
        if events or self.tracker.state != prev:
            stamp = self.get_clock().now().to_msg()
            self._publish_events(events, stamp)
            self._publish_state(stamp)
            self.get_logger().warning(f"estado de paso: {prev.name} -> {self.tracker.state.name} (sin escaneos)")

    def _publish_state(self, stamp: TimeMsg) -> None:
        self.pub_state.publish(pass_state_msg(self.tracker, stamp, self.frame_id))

    def _publish_events(self, events: list[PassEvent], stamp: TimeMsg) -> None:
        for ev in events:
            sign = self.params.velocity_sign
            self.pub_event.publish(pass_event_msg(ev, stamp, self.frame_id, sign))
            ev = ev.with_sign(sign)
            sentido = {1: "+", -1: "−", 0: "?"}[ev.direction]
            if ev.event == "INICIO":
                self.get_logger().info(f"paso #{ev.pass_id}: INICIO (sentido {sentido})")
            else:
                self.get_logger().info(
                    f"paso #{ev.pass_id}: FIN sentido {sentido}, {ev.duration_s:.2f} s, v mediana {ev.v_median_mps:.2f} m/s "
                    f"(avance {ev.advance_median_mps:.2f}), "
                    f"{ev.n_valid}/{ev.n_scans} válidos, banderas: {', '.join(ev.flags.nombres()) or 'ninguna'}")


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
