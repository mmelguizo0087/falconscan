"""Nodo ROS 2: reconstructor de volumen (LiDAR transversal + velocidad del LiDAR de velocidad).

Suscribe:
- scan (LaserScan del LiDAR de volumen; remapear a /lidar_volume/scan)
- velocity_estimate (VelocityEstimate) y pass_event (PassEvent) del velocity_node

Publica:
- volume/state (VolumeState): estado de la máquina, en cada perfil y en el timer
- volume/section (PointCloud2, frame del LiDAR de volumen): perfil transversal actual
- volume/cloud (PointCloud2, transient local, frame del paso): nube provisional y final
- volume/surface (Marker TRIANGLE_LIST, transient local): superficie de la medición final
- volume/measurement (VolumeMeasurement, transient local): resultado

Toda la lógica vive en ``falconscan_volume.core``; este archivo solo convierte mensajes. Los
parámetros se declaran desde ``VolumeParams`` (sección volume_node de config/falconscan.yaml,
el archivo compartido con velocity_node).
"""
from __future__ import annotations

import math
import time
from pathlib import Path

import numpy as np
import rclpy
from builtin_interfaces.msg import Time as TimeMsg
from falconscan_interfaces.msg import PassEvent as PassEventMsg
from falconscan_interfaces.msg import VelocityEstimate as VelocityMsg
from falconscan_interfaces.msg import VolumeMeasurement as MeasurementMsg
from falconscan_interfaces.msg import VolumeState as VolumeStateMsg
from falconscan_velocity.core import PassEvent, Scan
from geometry_msgs.msg import Point
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from sensor_msgs.msg import LaserScan, PointCloud2, PointField
from std_msgs.msg import ColorRGBA, Header
from visualization_msgs.msg import Marker

from .core import (CaptureStarted, CloudUpdate, Measurement, Reconstruction, VelocitySample, VolumeParams,
                   VolumeReconstructor, surface_triangles, write_json, write_ply)

_SURFACE_STEP = 2          # diezmado de columnas de la malla para RViz (2 cm -> 4 cm)


def _to_sec(t: TimeMsg) -> float:
    return t.sec + t.nanosec * 1e-9


def _to_time(s: float) -> TimeMsg:
    if not math.isfinite(s):
        return TimeMsg()
    sec, nsec = divmod(int(round(s * 1e9)), 1_000_000_000)
    return TimeMsg(sec=sec, nanosec=nsec)


def _cloud(points: np.ndarray, header: Header) -> PointCloud2:
    """PointCloud2 con campos x, y, z, intensity (float32)."""
    pts = np.ascontiguousarray(points[:, :4], dtype=np.float32)
    msg = PointCloud2()
    msg.header = header
    msg.height, msg.width = 1, pts.shape[0]
    msg.fields = [PointField(name=n, offset=4 * i, datatype=PointField.FLOAT32, count=1)
                  for i, n in enumerate(("x", "y", "z", "intensity"))]
    msg.is_bigendian = False
    msg.point_step = 16
    msg.row_step = 16 * pts.shape[0]
    msg.is_dense = True
    msg.data = pts.tobytes()
    return msg


def _height_color(z: float, z_max: float = 3.5) -> ColorRGBA:
    """Azul (bajo) -> verde -> amarillo (alto)."""
    u = min(max(z / z_max, 0.0), 1.0)
    return ColorRGBA(r=float(min(1.0, 2 * u)), g=float(min(1.0, 0.3 + u)), b=float(max(0.0, 1.0 - 2 * u)), a=1.0)


class VolumeNode(Node):
    def __init__(self) -> None:
        super().__init__("volume_node")
        defaults = VolumeParams()
        values = {n: self.declare_parameter(n, getattr(defaults, n)).value for n in VolumeParams.field_names()}
        self.pass_frame = self.declare_parameter("pass_frame_id", "falconscan_paso").value
        self.params = VolumeParams(**values)
        self.rec = VolumeReconstructor(self.params)
        self.lidar_frame = "lidar_volume"

        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        latched_events = QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.pub_state = self.create_publisher(VolumeStateMsg, "volume/state", 10)
        self.pub_section = self.create_publisher(PointCloud2, "volume/section", 5)
        self.pub_cloud = self.create_publisher(PointCloud2, "volume/cloud", latched)
        self.pub_surface = self.create_publisher(Marker, "volume/surface", latched)
        self.pub_meas = self.create_publisher(MeasurementMsg, "volume/measurement", latched_events)
        self.create_subscription(LaserScan, "scan", self.on_scan, qos_profile_sensor_data)
        self.create_subscription(VelocityMsg, "velocity_estimate", self.on_velocity, 50)
        self.create_subscription(PassEventMsg, "pass_event", self.on_pass_event, latched_events)
        self.create_timer(0.1, self.on_timer)
        p = self.params
        self.get_logger().info(
            f"volume_node listo: H={p.mount_height_m} m, ROI y=[{p.vol_roi_y_min_m}, {p.vol_roi_y_max_m}] m, "
            f"plano a {p.vol_plane_offset_m} m del LiDAR de velocidad, bordes por velocidad="
            f"{p.vol_edge_from_velocity}, export='{p.vol_export_dir}'")

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    # ---------------------------------------------------------------- entradas
    def on_scan(self, msg: LaserScan) -> None:
        self.lidar_frame = msg.header.frame_id or self.lidar_frame
        scan = Scan(stamp=_to_sec(msg.header.stamp), angle_min=msg.angle_min, angle_increment=msg.angle_increment,
                    ranges=np.asarray(msg.ranges, dtype=np.float32), time_increment=msg.time_increment,
                    range_min=msg.range_min, range_max=msg.range_max)
        outs = self.rec.on_scan(scan, now=self._now())
        sec = self.rec.last_section
        if sec is not None and sec.points.size:
            y, z, _ = sec.points.T
            pts = np.column_stack([np.zeros_like(y), y, z, z])
            self.pub_section.publish(_cloud(pts, Header(stamp=msg.header.stamp, frame_id=self.lidar_frame)))
        self._dispatch(outs, msg.header.stamp)

    def on_velocity(self, m: VelocityMsg) -> None:
        s = VelocitySample(stamp=_to_sec(m.scan_stamp), mid_stamp=_to_sec(m.header.stamp),
                           velocity_mps=m.velocity_mps, valid=m.valid,
                           x_start=m.object_x_start_m, x_end=m.object_x_end_m,
                           pass_id=m.pass_id, direction=m.direction)
        if not m.valid:
            s.mid_stamp = math.nan
        self._dispatch(self.rec.on_velocity(s, now=self._now()), m.header.stamp)

    def on_pass_event(self, m: PassEventMsg) -> None:
        ev = PassEvent("INICIO" if m.event == PassEventMsg.EVENT_INICIO else "FIN", m.pass_id, m.direction,
                       _to_sec(m.t_start), _to_sec(m.t_end), list(m.flags))
        self._dispatch(self.rec.on_pass_event(ev), m.header.stamp)

    def on_timer(self) -> None:
        now = self.get_clock().now()
        self._dispatch(self.rec.tick(now.nanoseconds * 1e-9), now.to_msg())

    # ---------------------------------------------------------------- salidas
    def _dispatch(self, outs: list, stamp: TimeMsg) -> None:
        for o in outs:
            if isinstance(o, CaptureStarted):
                self.get_logger().info(f"medición {o.measurement_id}: inicio de captura (paso {o.pass_id})")
            elif isinstance(o, CloudUpdate):
                self.pub_cloud.publish(_cloud(o.reconstruction.points, Header(stamp=stamp, frame_id=self.pass_frame)))
                if not o.provisional:
                    self.pub_surface.publish(self._surface(o.reconstruction, stamp))
            elif isinstance(o, Measurement):
                self._publish_measurement(o, stamp)
        st = VolumeStateMsg()
        st.header = Header(stamp=stamp, frame_id=self.pass_frame)
        st.mode = self.rec.mode
        st.state = int(self.rec.state)
        st.state_text = self.rec.state.name
        st.ready = self.rec.ready
        st.measurement_id = self.rec.measurement_id
        st.pass_id = self.rec.current_pass_id
        st.n_profiles = len(self.rec.cap.sections) if self.rec.cap else 0
        st.partial_volume_m3 = float(self.rec.partial_volume_m3)
        self.pub_state.publish(st)

    def _surface(self, rec: Reconstruction, stamp: TimeMsg) -> Marker:
        m = Marker()
        m.header = Header(stamp=stamp, frame_id=self.pass_frame)
        m.ns, m.id = "volumen", 0
        m.type, m.action = Marker.TRIANGLE_LIST, Marker.ADD
        m.pose.orientation.w = 1.0
        m.scale.x = m.scale.y = m.scale.z = 1.0
        tri = surface_triangles(rec, step=_SURFACE_STEP)
        for t in tri:
            for v in t:
                m.points.append(Point(x=float(v[0]), y=float(v[1]), z=float(v[2])))
                m.colors.append(_height_color(float(v[2])))
        return m

    def _publish_measurement(self, o: Measurement, stamp: TimeMsg) -> None:
        msg = MeasurementMsg()
        msg.header = Header(stamp=_to_time(o.t_end), frame_id=self.pass_frame)
        for name in ("measurement_id", "pass_id", "direction", "result", "result_text", "volume_m3",
                     "end_uncertainty_m3", "edge_method", "length_m", "max_width_m", "max_height_m",
                     "mean_advance_mps", "yaw_rad", "center_offset_m", "max_lateral_dev_m", "path_rms_m",
                     "n_profiles", "max_dx_m", "nan_fraction", "v_interp_fraction", "sync_offset_s",
                     "incoherent_fraction", "n_gaps"):
            v = getattr(o, name)
            setattr(msg, name, float(v) if isinstance(v, float) else v)
        msg.t_start, msg.t_end = _to_time(o.t_start), _to_time(o.t_end)
        msg.flags = list(o.flags)
        self.pub_meas.publish(msg)
        log = self.get_logger().info if o.result <= 1 else self.get_logger().warning
        log(f"medición {o.measurement_id} (paso {o.pass_id}, sentido {o.direction:+d}): "
            f"V = {o.volume_m3:.2f} m³ ± {o.end_uncertainty_m3:.2f}, L = {o.length_m:.2f} m, "
            f"{o.n_profiles} perfiles, {o.result_text} {o.flags}")
        if self.params.vol_export_dir and o.result != 3:
            base = Path(self.params.vol_export_dir) / time.strftime(f"medicion_{o.measurement_id:04d}_%Y%m%d_%H%M%S")
            try:
                write_ply(base.with_suffix(".ply"), o.reconstruction.points)
                data = {k: v for k, v in o.__dict__.items() if k != "reconstruction"}
                write_json(base.with_suffix(".json"), data)
                self.get_logger().info(f"exportada en {base}.ply/.json")
            except OSError as e:
                self.get_logger().error(f"no se pudo exportar a {self.params.vol_export_dir}: {e}")


def main(args=None) -> None:
    rclpy.init(args=args)
    node = VolumeNode()
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
