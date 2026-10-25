"""Nodo ROS 2 que simula el pórtico completo: publica los escaneos de los DOS LiDAR viendo pasar
volquetas, para probar velocity_node + volume_node + RViz sin hardware.

    ros2 run falconscan_volume sim_portico --ros-args -p speed_kmh:=7.5 -p direction:=0

Parámetros:
- speed_kmh: rapidez de paso.
- direction: +1, -1, o 0 para alternar el sentido en cada paso.
- yaw_deg, y_offset_m, lane_shift_m: trayectoria (diagonal, descentrado, cambio de carril).
- gap_s: tiempo entre pasos.
- vol_plane_x_m: posición verdadera del plano de volumen (para ensayar errores de montaje).
- march_step_m: paso de la marcha de rayos (más grande = más rápido y menos exacto). Cada
  tick genera los dos escaneos (~35 ms cada uno en un PC); si el PC no alcanza los 10 Hz,
  subir este valor.
"""
from __future__ import annotations

import math

import numpy as np
import rclpy
from builtin_interfaces.msg import Time as TimeMsg
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

from .sim import DumpTruck3D, Motion3D, PorticoSim, PorticoSimConfig, lane_change

_D0 = -6.0      # el frente arranca 6 m antes del plano de volumen
_AFTER = 7.0    # metros que recorre la cola después del plano antes de terminar el paso


def _to_time(s: float) -> TimeMsg:
    sec, nsec = divmod(int(round(s * 1e9)), 1_000_000_000)
    return TimeMsg(sec=sec, nanosec=nsec)


class SimPortico(Node):
    def __init__(self) -> None:
        super().__init__("sim_portico")
        self.v = self.declare_parameter("speed_kmh", 10.0).value / 3.6
        self.direction = int(self.declare_parameter("direction", 0).value)
        self.yaw = math.radians(self.declare_parameter("yaw_deg", 0.0).value)
        self.y_offset = self.declare_parameter("y_offset_m", 0.0).value
        self.lane_shift = self.declare_parameter("lane_shift_m", 0.0).value
        self.gap = self.declare_parameter("gap_s", 4.0).value
        hz = self.declare_parameter("scan_hz", 10.0).value
        plane = self.declare_parameter("vol_plane_x_m", 0.0).value
        step = self.declare_parameter("march_step_m", 0.01).value
        self.vel_frame = self.declare_parameter("velocity_frame_id", "lidar_velocity").value
        self.vol_frame = self.declare_parameter("volume_frame_id", "lidar_volume").value
        self.cfg = PorticoSimConfig(scan_hz=hz, vol_plane_x=plane, march_step_m=step)
        self.truck = DumpTruck3D()
        self.period = 1.0 / hz
        self.pass_dur = (abs(_D0) + self.truck.length + _AFTER) / self.v
        self.t = 0.0
        self.t0 = self.get_clock().now().nanoseconds * 1e-9
        self.n_pass = -1
        self.sim = None
        self.pub_vel = self.create_publisher(LaserScan, "lidar_velocity/scan", 10)
        self.pub_vol = self.create_publisher(LaserScan, "lidar_volume/scan", 10)
        self.create_timer(self.period, self.tick)
        self.get_logger().info(
            f"sim_portico: {self.v * 3.6:.1f} km/h, sentido {'alterno' if self.direction == 0 else self.direction}, "
            f"volumen verdadero de la envolvente {self.truck.volume_truth():.2f} m³")

    def _motion(self, k: int, t_start: float) -> Motion3D:
        d = self.direction if self.direction else (1 if k % 2 == 0 else -1)
        lat = lane_change(self.lane_shift, -2.0, 6.0) if self.lane_shift else None
        return Motion3D(direction=d, distance=lambda t: _D0 + self.v * (np.asarray(t) - t_start),
                        y_offset=self.y_offset, yaw_rad=self.yaw, lateral=lat)

    def tick(self) -> None:
        cycle = self.pass_dur + self.gap
        k = int(self.t // cycle)
        if k != self.n_pass:
            self.n_pass = k
            self.sim = PorticoSim(self.cfg, self.truck, self._motion(k, k * cycle))
            self.get_logger().info(f"paso simulado {k + 1}: sentido {self.sim.motion.direction:+d}")
        self.pub_vel.publish(self._msg(self.sim.velocity_scan(self.t), self.t0, self.vel_frame))
        self.pub_vol.publish(self._msg(self.sim.volume_scan(self.t + self.cfg.vol_phase_s), self.t0, self.vol_frame))
        self.t += self.period

    def _msg(self, s, t0: float, frame: str) -> LaserScan:
        m = LaserScan()
        m.header.stamp = _to_time(t0 + s.stamp)
        m.header.frame_id = frame
        m.angle_min = float(s.angle_min)
        m.angle_increment = float(s.angle_increment)
        m.angle_max = float(s.angle_min + (len(s.ranges) - 1) * s.angle_increment)
        m.time_increment = float(s.time_increment)
        m.scan_time = self.period
        m.range_min = float(s.range_min)
        m.range_max = float(s.range_max)
        m.ranges = np.asarray(s.ranges, dtype=np.float32).tolist()
        return m


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SimPortico()
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
