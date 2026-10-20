"""Simulador 2D de un LiDAR rotativo sobre el pórtico viendo pasar una volqueta.

Sirve para desarrollar y hacer pruebas de regresión del algoritmo antes de tener
datos reales. NO valida el algoritmo para campo: la geometría, el ruido y los
materiales reales deben caracterizarse con el sensor (reporte §8).

Modelo:
- Plano vertical (x, z), piso en z = 0, sensor en (0, H).
- Volqueta = polígono lateral (cabina + volco con montículo de material).
- Cada rayo se dispara en su propio instante t_i = stamp + i * time_increment,
  así que el objeto se mueve DURANTE el barrido (distorsión de rotación incluida).
- Ruido gaussiano de rango, pérdidas aleatorias y outliers.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Callable, Iterator

import numpy as np

from ..core.scan import Scan


def make_dump_truck(rng: np.random.Generator | None = None, heap_amplitude: float = 0.5,
                    heap_roughness: float = 0.08) -> np.ndarray:
    """Contorno lateral de una volqueta en coordenadas locales (frente en x = 0, cola en x < 0).

    Devuelve los vértices de un polígono cerrado (N, 2). Medidas aproximadas de
    una volqueta doble troque; ajustar con medidas reales cuando se tengan.
    """
    rng = rng or np.random.default_rng(0)
    box_front, box_rear, wall = -2.5, -7.5, 2.6
    xs = np.linspace(box_front - 0.1, box_rear + 0.1, 21)
    s = (xs - box_front) / (box_rear - box_front)
    heap = wall + heap_amplitude * np.sin(np.pi * s) ** 0.7 + heap_roughness * rng.standard_normal(xs.size)
    top = [(0.0, 0.6), (0.0, 2.8), (-0.3, 3.05), (-2.1, 3.05), (-2.2, 1.3), (-2.4, 1.3),
           (box_front, wall + 0.1)]
    top += list(zip(xs, heap))
    top += [(box_rear, wall + 0.1), (box_rear, 0.6)]
    return np.asarray(top, dtype=float)


@dataclass
class LidarSimConfig:
    height_m: float = 5.0
    scan_hz: float = 10.0
    samples_per_rev: int = 3200        # ~RPLIDAR S2E: 32 kS/s a 10 Hz -> 0.1125°
    sector_deg: tuple[float, float] = (-70.0, 70.0)  # theta respecto al nadir
    range_noise_m: float = 0.015
    dropout_prob: float = 0.01
    outlier_prob: float = 0.002
    range_max_m: float = 30.0
    seed: int = 1


@dataclass
class TruckMotion:
    """Posición del frente de la volqueta en x(t). Por defecto velocidad constante."""

    x0: float = -10.0
    v: float = 1.5
    position: Callable[[np.ndarray], np.ndarray] | None = None

    def x(self, t: np.ndarray) -> np.ndarray:
        if self.position is not None:
            return self.position(t)
        return self.x0 + self.v * t

    def velocity(self, t: float, h: float = 1e-4) -> float:
        return float((self.x(np.array([t + h])) - self.x(np.array([t - h])))[0] / (2 * h))


@dataclass
class LidarSim:
    cfg: LidarSimConfig = field(default_factory=LidarSimConfig)
    truck: np.ndarray = field(default_factory=make_dump_truck)

    def __post_init__(self) -> None:
        self.rng = np.random.default_rng(self.cfg.seed)
        c = self.cfg
        self.dtheta = 2 * math.pi / c.samples_per_rev
        self.time_increment = 1.0 / (c.scan_hz * c.samples_per_rev)
        a0, a1 = (math.radians(d) for d in c.sector_deg)
        self.angle_min = a0
        self.n_rays = int(round((a1 - a0) / self.dtheta)) + 1
        p = self.truck
        self._seg_p = p
        self._seg_q = np.roll(p, -1, axis=0)

    def _raycast(self, phi: np.ndarray, shift: np.ndarray) -> np.ndarray:
        """Distancia al primer impacto (piso o volqueta desplazada `shift` en x) por rayo."""
        H = self.cfg.height_m
        d = np.stack([np.cos(phi), np.sin(phi)], axis=1)               # (N,2)
        # Piso z = 0
        with np.errstate(divide="ignore", invalid="ignore"):
            t_floor = np.where(d[:, 1] < -1e-9, -H / d[:, 1], np.inf)
        # Segmentos de la volqueta, desplazados por rayo
        P = self._seg_p[None, :, :] + np.stack([shift, np.zeros_like(shift)], 1)[:, None, :]  # (N,M,2)
        E = (self._seg_q - self._seg_p)[None, :, :]                                            # (1,M,2)
        O = np.array([0.0, H])
        W = P - O                                                                              # (N,M,2)
        dd = d[:, None, :]
        den = dd[..., 0] * E[..., 1] - dd[..., 1] * E[..., 0]
        with np.errstate(divide="ignore", invalid="ignore"):
            t = (W[..., 0] * E[..., 1] - W[..., 1] * E[..., 0]) / den
            u = (W[..., 0] * dd[..., 1] - W[..., 1] * dd[..., 0]) / den
        hit = (np.abs(den) > 1e-12) & (t > 1e-6) & (u >= 0) & (u <= 1)
        t = np.where(hit, t, np.inf).min(axis=1)
        return np.minimum(t, t_floor)

    def scan_at(self, stamp: float, motion: TruckMotion) -> Scan:
        c = self.cfg
        i = np.arange(self.n_rays)
        theta = self.angle_min + i * self.dtheta
        t_ray = stamp + i * self.time_increment
        phi = theta - math.pi / 2                     # theta = 0 al nadir
        r = self._raycast(phi, motion.x(t_ray))
        r = r + c.range_noise_m * self.rng.standard_normal(r.size)
        r[self.rng.random(r.size) < c.dropout_prob] = np.inf
        out = self.rng.random(r.size) < c.outlier_prob
        r[out] = self.rng.uniform(0.3, 8.0, out.sum())
        r[r > c.range_max_m] = np.inf
        return Scan(stamp=stamp, angle_min=self.angle_min, angle_increment=self.dtheta,
                    ranges=r.astype(np.float32), time_increment=self.time_increment,
                    range_min=0.05, range_max=c.range_max_m)

    def run(self, motion: TruckMotion, duration_s: float, t0: float = 0.0,
            jitter_s: float = 0.0) -> Iterator[tuple[Scan, float]]:
        """Genera (scan, velocidad_de_referencia). La referencia es v en el centro del escaneo."""
        period = 1.0 / self.cfg.scan_hz
        n = int(duration_s * self.cfg.scan_hz)
        for k in range(n):
            stamp = t0 + k * period + (self.rng.normal(0, jitter_s) if jitter_s else 0.0)
            yield self.scan_at(stamp, motion), motion.velocity(stamp + 0.5 * self.n_rays * self.time_increment)
