"""Simulador 3D del pórtico: los DOS LiDAR viendo pasar el mismo objeto.

Sirve para desarrollar y hacer pruebas de regresión del reconstructor de volumen. NO valida
el sistema para campo: geometría, materiales, ruido y sincronía reales se caracterizan con
los sensores (y la validación necesita un objeto patrón de volumen conocido).

Modelo:
- El objeto es un campo de alturas h(s, w) en su propio marco: s = distancia desde el frente
  hacia la cola, w = lateral (izquierda +). El volumen VERDADERO de la envolvente es
  ∫∫ h ds dw (lo que mide el método, por definición).
- El objeto avanza por una recta con ángulo ``yaw_rad`` respecto al eje x, desplazada
  ``y_offset`` del eje del carril, en sentido ±1. La distancia recorrida d(t) puede ser
  constante, con aceleración o con paradas.
- Cada LiDAR es un plano de escaneo vertical: el de velocidad contiene el eje x (en y = 0),
  el de volumen es transversal (en x = ``vol_plane_x``). Cada rayo se dispara en su propio
  instante (el objeto se mueve durante el barrido) y se intersecta por marcha sobre el rayo.
- Ruido de rango, pérdida de puntos, outliers, escaneos completos perdidos y un error de
  reloj inyectable en los stamps del LiDAR de volumen.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Callable, Iterator

import numpy as np

from falconscan_velocity.core import Scan


# ------------------------------------------------------------------ objetos
@dataclass
class DumpTruck3D:
    """Volqueta aproximada (cabina + chasís + volco con carga). Medidas por verificar."""

    cab_len: float = 2.2
    cab_w: float = 2.45
    cab_h: float = 3.05
    hood_h: float = 2.8
    hood_len: float = 0.3
    mirror_w: float = 0.25          # espejos laterales (sobresalen a cada lado de la cabina)
    gap_len: float = 0.3
    chassis_w: float = 1.0
    chassis_h: float = 1.3
    box_len: float = 5.0
    box_w: float = 2.5
    wall_t: float = 0.08
    wall_h: float = 2.7
    box_floor_h: float = 1.4
    load_level: float = 2.5
    heap_amp: float = 0.6
    rough_amp: float = 0.05
    seed: int = 0

    def __post_init__(self):
        rng = np.random.default_rng(self.seed)
        self._k = rng.uniform([1.0, 1.0], [6.0, 6.0], size=(6, 2))
        self._ph = rng.uniform(0, 2 * np.pi, 6)

    @property
    def length(self) -> float:
        return self.cab_len + self.gap_len + self.box_len

    @property
    def half_width(self) -> float:
        return max(self.cab_w / 2 + self.mirror_w, self.box_w / 2)

    def height(self, s: np.ndarray, w: np.ndarray) -> np.ndarray:
        s, w = np.asarray(s, float), np.asarray(w, float)
        aw = np.abs(w)
        h = np.zeros(np.broadcast(s, w).shape)
        # cabina: capó inclinado al frente, techo, caída trasera
        cab = (s >= 0) & (s < self.cab_len) & (aw <= self.cab_w / 2)
        hc = self.hood_h + (self.cab_h - self.hood_h) * np.clip(s / self.hood_len, 0, 1)
        hc = np.where(s > self.cab_len - 0.1, self.chassis_h + (self.cab_h - self.chassis_h)
                      * np.clip((self.cab_len - s) / 0.1, 0, 1), hc)
        h = np.where(cab, hc, h)
        mir = (s >= 0.5) & (s < 0.7) & (aw > self.cab_w / 2) & (aw <= self.cab_w / 2 + self.mirror_w)
        h = np.where(mir, 2.45, h)
        s_box = self.cab_len + self.gap_len
        gap = (s >= self.cab_len) & (s < s_box) & (aw <= self.chassis_w / 2)
        h = np.where(gap, self.chassis_h, h)
        # volco: paredes + carga (montículo con rugosidad suave y determinista)
        box = (s >= s_box) & (s < self.length) & (aw <= self.box_w / 2)
        wall = box & ((aw > self.box_w / 2 - self.wall_t) | (s < s_box + self.wall_t)
                      | (s >= self.length - self.wall_t))
        u = np.clip((s - s_box) / self.box_len, 0, 1)
        iw = self.box_w / 2 - self.wall_t
        lat = np.sqrt(np.clip(1 - (w / iw) ** 2, 0, 1))
        rough = sum(np.sin(k[0] * s + k[1] * w + ph) for k, ph in zip(self._k, self._ph))
        load = self.load_level + self.heap_amp * np.sin(np.pi * u) ** 0.7 * lat + self.rough_amp * rough / 6 ** 0.5
        h = np.where(box, np.where(wall, self.wall_h, np.maximum(self.box_floor_h, load)), h)
        return h

    def volume_truth(self, ds: float = 0.004) -> float:
        s = np.arange(ds / 2, self.length, ds)
        w = np.arange(-self.half_width + ds / 2, self.half_width, ds)
        S, W = np.meshgrid(s, w, indexing="ij")
        return float(self.height(S, W).sum() * ds * ds)


@dataclass
class Box3D:
    """Caja simple (persona, moto, obstáculo, objeto patrón)."""

    length: float = 0.5
    width: float = 0.6
    h: float = 1.7

    @property
    def half_width(self) -> float:
        return self.width / 2

    def height(self, s, w):
        s, w = np.asarray(s, float), np.asarray(w, float)
        return np.where((s >= 0) & (s < self.length) & (np.abs(w) <= self.width / 2), self.h, 0.0)

    def volume_truth(self, ds: float = 0.004) -> float:
        return self.length * self.width * self.h


# ------------------------------------------------------------------ movimiento
@dataclass
class Motion3D:
    """Trayectoria del frente del objeto; el cuerpo se alinea con la tangente (sólido rígido).

    Recta base: P0(d) = (0, y_offset) + d · u0, con u0 = (sentido·cos ψ, sin ψ). Con
    ``lateral`` (función de d) el frente se desvía lat(d) hacia la izquierda del camión
    (cambio de carril / giro bajo el pórtico) y el cuerpo gira con la tangente.

    d(t) < 0 antes de que el frente llegue a x = 0. Por defecto d(t) = d0 + v·t.

    Ojo: en una recta en diagonal (ψ ≠ 0, sin ``lateral``) cada punto del camión recorre la
    misma recta, así que el centro de la sección en un plano FIJO no se mueve. Solo una
    trayectoria que cambia de dirección mueve el centro.
    """

    direction: int = 1
    v: float = 2.78
    d0: float = -9.0
    distance: Callable[[np.ndarray], np.ndarray] | None = None
    y_offset: float = 0.0
    yaw_rad: float = 0.0
    lateral: Callable[[np.ndarray], np.ndarray] | None = None

    def d(self, t):
        t = np.asarray(t, float)
        return self.distance(t) if self.distance is not None else self.d0 + self.v * t

    def frame(self, t) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
        """(Fx, Fy, ux, uy): frente y dirección del cuerpo en t."""
        d = self.d(t)
        c, s = math.cos(self.yaw_rad), math.sin(self.yaw_rad)
        u0x, u0y = self.direction * c, s
        n0x, n0y = -u0y, u0x
        if self.lateral is None:
            lat, dlat = 0.0, 0.0
        else:
            lat = self.lateral(d)
            dlat = (self.lateral(d + 1e-3) - self.lateral(d - 1e-3)) / 2e-3
        fx = d * u0x + lat * n0x
        fy = self.y_offset + d * u0y + lat * n0y
        tx, ty = u0x + dlat * n0x, u0y + dlat * n0y
        nrm = np.hypot(tx, ty)
        return fx, fy, tx / nrm, ty / nrm

    def front(self, t) -> tuple[np.ndarray, np.ndarray]:
        fx, fy, _, _ = self.frame(t)
        return fx, fy

    def speed(self, t: float, h: float = 1e-4) -> float:
        return float((self.d(t + h) - self.d(t - h)) / (2 * h))

    def velocity_x(self, t: float, h: float = 1e-4) -> float:
        """Velocidad verdadera en el eje x (lo que debe medir el LiDAR de velocidad)."""
        return float((self.front(t + h)[0] - self.front(t - h)[0]) / (2 * h))


def lane_change(shift: float, d_a: float, d_b: float) -> Callable:
    """Desvío lateral suave de ``shift`` metros entre d_a y d_b (cambio de carril bajo el pórtico)."""
    def lat(d):
        x = np.clip((np.asarray(d, float) - d_a) / (d_b - d_a), 0, 1)
        return shift * (3 * x ** 2 - 2 * x ** 3)
    return lat


def stop_and_go(v: float, d0: float, d_stop: float, t_stop: float) -> Callable:
    """Avanza a v, se detiene t_stop segundos al llegar a d_stop y sigue."""
    t1 = (d_stop - d0) / v

    def d(t):
        t = np.asarray(t, float)
        return np.where(t < t1, d0 + v * t, np.where(t < t1 + t_stop, d_stop, d_stop + v * (t - t1 - t_stop)))
    return d


# ------------------------------------------------------------------ sensores
@dataclass
class PorticoSimConfig:
    vel_height: float = 5.0
    vol_height: float = 5.0
    vol_plane_x: float = 0.0           # posición VERDADERA del plano de volumen en x
    vol_sensor_y: float = 0.0          # posición lateral verdadera del LiDAR de volumen
    scan_hz: float = 10.0
    samples_per_rev: int = 3200        # ~S2E: 0.1125°
    vel_sector_deg: tuple[float, float] = (-70.0, 70.0)   # como el simulador 2D: el techo se ve en toda la ROI
    # Ángulo ROS del rayo al nadir del LiDAR de velocidad: S2E con la flecha 0° al piso (D15), igual
    # que el simulador 2D, para que valga la geometría nominal del yaml (angle_offset_rad = +π/2).
    vel_nadir_angle_deg: float = 180.0
    vol_sector_deg: tuple[float, float] = (-50.0, 50.0)
    vol_phase_s: float = 0.037         # desfase entre las rotaciones de los dos sensores
    vol_clock_offset_s: float = 0.0    # error de reloj inyectado en los stamps del LiDAR de volumen
    range_noise_m: float = 0.015
    dropout_prob: float = 0.01
    outlier_prob: float = 0.002
    scan_dropout_prob: float = 0.0
    march_step_m: float = 0.01
    h_max: float = 3.6                 # altura máxima del objeto (acota la marcha)
    reach_m: float = 4.5               # alcance horizontal de la marcha; más allá solo hay piso
    seed: int = 1


@dataclass
class PorticoSim:
    cfg: PorticoSimConfig
    obj: object                        # DumpTruck3D, Box3D o cualquier objeto con .height(s, w)
    motion: Motion3D
    rng: np.random.Generator = field(init=False)

    def __post_init__(self):
        c = self.cfg
        self.rng = np.random.default_rng(c.seed)
        self.dtheta = 2 * math.pi / c.samples_per_rev
        self.time_increment = 1.0 / (c.scan_hz * c.samples_per_rev)

    def _cast(self, origin_xy, e_h, H, theta, t_rays) -> np.ndarray:
        c = self.cfg
        ct, st = np.cos(theta), np.sin(theta)
        t_floor = H / ct
        with np.errstate(divide="ignore"):
            t_hi = np.minimum(t_floor, c.reach_m / np.maximum(np.abs(st), 1e-9))
        t_lo = np.minimum(np.maximum(0.0, (H - c.h_max) / ct), t_hi)
        n = int(np.ceil(np.max(t_hi - t_lo) / c.march_step_m)) + 2
        T = np.minimum(t_lo[:, None] + np.arange(n)[None, :] * c.march_step_m, t_hi[:, None])
        Px = origin_xy[0] + T * (st * e_h[0])[:, None]
        Py = origin_xy[1] + T * (st * e_h[1])[:, None]
        Z = H - T * ct[:, None]
        fx, fy, ux, uy = (np.broadcast_to(a, t_rays.shape)[:, None] for a in self.motion.frame(t_rays))
        dX, dY = Px - fx, Py - fy
        s = -(dX * ux + dY * uy)
        w = -dX * uy + dY * ux
        f = Z - self.obj.height(s, w)
        hit = f <= 1e-9
        reached = hit.any(axis=1)
        k1 = np.argmax(hit, axis=1)
        rows = np.arange(theta.size)
        k0 = np.maximum(k1 - 1, 0)
        f0, f1 = f[rows, k0], f[rows, k1]
        T0, T1 = T[rows, k0], T[rows, k1]
        with np.errstate(invalid="ignore", divide="ignore"):
            frac = np.where((k1 > 0) & (f0 > f1), f0 / (f0 - f1), 0.0)
        # sin impacto dentro del alcance: el rayo llega al piso
        return np.where(reached, T0 + frac * (T1 - T0), t_floor)

    def _scan(self, stamp, origin_xy, e_h, H, sector, report_offset=0.0, nadir_angle=0.0) -> Scan:
        """θ del rayo respecto al nadir; el escaneo se publica con ángulos θ + ``nadir_angle``."""
        c = self.cfg
        a0, a1 = (math.radians(x) for x in sector)
        n = int(round((a1 - a0) / self.dtheta)) + 1
        theta = a0 + np.arange(n) * self.dtheta
        t_rays = stamp + np.arange(n) * self.time_increment
        r = self._cast(np.asarray(origin_xy, float), np.asarray(e_h, float), H, theta, t_rays)
        r = r + c.range_noise_m * self.rng.standard_normal(n)
        r[self.rng.random(n) < c.dropout_prob] = np.inf
        out = self.rng.random(n) < c.outlier_prob
        r[out] = self.rng.uniform(0.3, 8.0, out.sum())
        if c.scan_dropout_prob and self.rng.random() < c.scan_dropout_prob:
            r[:] = np.inf
        return Scan(stamp=stamp + report_offset, angle_min=a0 + nadir_angle, angle_increment=self.dtheta,
                    ranges=r.astype(np.float32), time_increment=self.time_increment,
                    range_min=0.05, range_max=30.0)

    def velocity_scan(self, stamp: float) -> Scan:
        """Plano x–z en y = 0 (θ > 0 hacia +x), nadir en ``vel_nadir_angle_deg``."""
        c = self.cfg
        return self._scan(stamp, (0.0, 0.0), (1.0, 0.0), c.vel_height, c.vel_sector_deg,
                          nadir_angle=math.radians(c.vel_nadir_angle_deg))

    def volume_scan(self, stamp: float) -> Scan:
        """Plano y–z en x = vol_plane_x (θ > 0 hacia +y)."""
        c = self.cfg
        return self._scan(stamp, (c.vol_plane_x, c.vol_sensor_y), (0.0, 1.0), c.vol_height, c.vol_sector_deg,
                          report_offset=c.vol_clock_offset_s)

    def run(self, duration_s: float, t0: float = 0.0) -> Iterator[tuple[float, str, Scan]]:
        """(instante de entrega, "vel"|"vol", escaneo) en orden temporal. Un escaneo se entrega
        al terminar su vuelta (stamp + período)."""
        period = 1.0 / self.cfg.scan_hz
        n = int(duration_s * self.cfg.scan_hz)
        for k in range(n):
            tv = t0 + k * period
            yield tv + period, "vel", self.velocity_scan(tv)
            tw = tv + self.cfg.vol_phase_s
            yield tw + period, "vol", self.volume_scan(tw)
