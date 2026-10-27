"""Nodo ROS 2: /scan (LaserScan del LiDAR de velocidad) -> velocidad y pasos de camión.

Publica:
- /velocity_estimate (VelocityEstimate): una por escaneo, con pass_id, direction y advance_mps.
- /pass_state (PassState): estado del PassTracker, una por escaneo y en cada cambio sin escaneos.
- /pass_event (PassEvent, transient local): INICIO y FIN de cada paso.

Todo en el marco mundo (+x): ``invert_velocity_sign`` es solo para mostrar (visor, HMI) y no se
aplica aquí, porque el volume_node integra ``velocity_mps`` tal como sale (D20).

Toda la lógica vive en ``falconscan_velocity.core`` (``VelocityPipeline`` = estimador +
PassTracker); este archivo solo hace de puente con ROS. Los parámetros se declaran
automáticamente a partir de ``VelocityParams``, así que config/falconscan.yaml y el código
no se desincronizan.
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
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32MultiArray

from .core import PassEvent, PassTracker, Scan, VelocityParams, VelocityPipeline


def _to_sec(t: TimeMsg) -> float:
    return t.sec + t.nanosec * 1e-9


def _to_time(s: float) -> TimeMsg:
    sec, nsec = divmod(int(round(s * 1e9)), 1_000_000_000)
    return TimeMsg(sec=sec, nanosec=nsec)


def pass_state_msg(tracker: PassTracker, header) -> PassStateMsg:
    m = PassStateMsg()
    m.header = header
    m.state = int(tracker.state)
    m.state_text = tracker.state.name
    m.pass_id = tracker.current_pass_id
    m.pass_open = tracker.open_pass
    m.direction = tracker.direction
    m.ready = tracker.ready
    return m


def pass_event_msg(ev: PassEvent, header) -> PassEventMsg:
    m = PassEventMsg()
    m.header = header
    m.pass_id = ev.pass_id
    m.event = PassEventMsg.EVENT_INICIO if ev.kind == "INICIO" else PassEventMsg.EVENT_FIN
    m.event_text = ev.kind
    m.valid = ev.valid
    m.direction = int(ev.direction)
    m.t_start = _to_time(ev.t_start)
    if ev.kind == "FIN":
        m.t_end = _to_time(ev.t_end)
    m.duration_s = float(ev.duration_s)
    m.flags = list(ev.flags)
    m.n_gaps, m.n_scans, m.n_valid = ev.n_gaps, ev.n_scans, ev.n_valid
    m.v_median_mps, m.v_min_mps, m.v_max_mps = float(ev.v_median_mps), float(ev.v_min_mps), float(ev.v_max_mps)
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
        self.pipeline = VelocityPipeline(self.params)
        self.estimator = self.pipeline.estimator
        self.tracker = self.pipeline.tracker

        latched = QoSProfile(depth=50, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.pub = self.create_publisher(VelocityMsg, "velocity_estimate", 10)
        self.pub_state = self.create_publisher(PassStateMsg, "pass_state", 10)
        self.pub_event = self.create_publisher(PassEventMsg, "pass_event", latched)
        self.pub_profile = (self.create_publisher(Float32MultiArray, "height_profile", 5)
                            if self.publish_profile else None)
        self.create_subscription(LaserScan, "scan", self.on_scan, qos_profile_sensor_data)
        # SIN_DATOS (T11) se detecta aunque no lleguen escaneos
        self.create_timer(max(0.02, self.params.scan_timeout_s / 5), self.on_timer)
        self.get_logger().info(
            f"velocity_node listo: H={self.params.mount_height_m} m, ROI x=[{self.params.roi_x_min_m}, "
            f"{self.params.roi_x_max_m}] m, dx={self.params.grid_dx_m} m, baseline={self.params.baseline_scans}, "
            f"búsqueda={self.params.search_mode}, rayos invertidos={self.params.scan_time_reversed}")

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

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
        prev = self.tracker.state
        e, events = self.pipeline.process(scan, now=self._now())

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
        # Paso al que pertenece este escaneo (si el escaneo cerró el paso, el del FIN)
        fin = next((ev for ev in events if ev.kind == "FIN"), None)
        out.pass_id = fin.pass_id if fin else self.tracker.current_pass_id
        out.direction = fin.direction if fin else self.tracker.direction
        out.advance_mps = float(out.direction * e.velocity_mps) if (e.valid and out.direction) else math.nan
        self.pub.publish(out)
        self._publish_pass(events, msg.header)
        if self.tracker.state != prev:
            self.get_logger().info(f"estado de paso: {prev.name} -> {self.tracker.state.name}")

        if self.pub_profile is not None and self.estimator.last_profile is not None:
            self.pub_profile.publish(Float32MultiArray(data=self.estimator.last_profile.astype(np.float32).tolist()))

    def on_timer(self) -> None:
        prev = self.tracker.state
        events = self.pipeline.tick(self._now())
        if events or self.tracker.state != prev:
            hdr = VelocityMsg().header
            hdr.stamp = self.get_clock().now().to_msg()
            self._publish_pass(events, hdr)
            self.get_logger().warning(f"estado de paso: {prev.name} -> {self.tracker.state.name} (sin escaneos)")

    def _publish_pass(self, events: list[PassEvent], header) -> None:
        self.pub_state.publish(pass_state_msg(self.tracker, header))
        for ev in events:
            self.pub_event.publish(pass_event_msg(ev, header))
            self.get_logger().info(
                f"paso {ev.pass_id} {ev.kind} sentido={ev.direction:+d}"
                + (f" {ev.duration_s:.1f} s v={ev.v_median_mps:.2f} m/s {ev.n_valid}/{ev.n_scans} válidos "
                   f"banderas={ev.flags or 'ninguna'}" if ev.kind == "FIN" else ""))


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
