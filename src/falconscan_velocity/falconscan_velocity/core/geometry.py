"""Etapas 4-5: geometría de montaje, perfil de altura en malla ESPACIAL y segmentación.

Montaje asumido para el LiDAR de velocidad
------------------------------------------
El plano de escaneo es VERTICAL y CONTIENE el eje de avance del camión.
Marco mundo: x = eje de avance (positivo en el sentido de circulación),
z = vertical hacia arriba, z = 0 en el piso.

Ángulo global de cada rayo, medido desde +x hacia +z (reporte §5.4):
    phi_i = angle_direction * theta_i + angle_offset
    x_i   = x_offset + r_i * cos(phi_i)
    z_i   = H        + r_i * sin(phi_i)

Ejemplo: sensor mirando hacia abajo con theta = 0 al nadir -> angle_offset = -pi/2.
Si el sensor se monta invertido, angle_direction = -1.

Por qué malla espacial y no h(theta)
------------------------------------
Una traslación rígida del camión en x es una traslación pura de h(x), pero NO
es una traslación pura de h(theta): el corrimiento angular depende de la altura
y la posición de cada punto. Remuestrear a una malla uniforme en x hace que el
lag de la correlación sea directamente un desplazamiento físico, Δs = k* · dx
(resuelve la "decisión técnica prioritaria" del reporte §10).
"""
from __future__ import annotations

from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class MountGeometry:
    height_m: float
    angle_offset_rad: float
    angle_direction: int = 1
    x_offset_m: float = 0.0


@dataclass(frozen=True)
class ProfileGrid:
    """Malla uniforme en x que define también la ROI longitudinal."""

    x_min: float
    x_max: float
    dx: float

    @property
    def n_bins(self) -> int:
        return int(round((self.x_max - self.x_min) / self.dx))

    @property
    def centers(self) -> np.ndarray:
        return self.x_min + (np.arange(self.n_bins) + 0.5) * self.dx


def polar_to_xz(r: np.ndarray, theta: np.ndarray, g: MountGeometry) -> tuple[np.ndarray, np.ndarray]:
    """Polar -> cartesiano en el marco mundo. Los NaN se propagan."""
    phi = g.angle_direction * np.asarray(theta) + g.angle_offset_rad
    x = g.x_offset_m + r * np.cos(phi)
    z = g.height_m + r * np.sin(phi)
    return x, z


def fan_mask(theta: np.ndarray, g: MountGeometry, fan_angle_rad: float, fan_center_rad: float = 0.0) -> np.ndarray:
    """Rayos dentro del abanico de interés, medido en el marco mundo desde la vertical hacia abajo.

    El ángulo de cada rayo respecto del nadir es delta = phi + π/2 (positivo hacia +x). Se aceptan
    los rayos con |delta − fan_center_rad| <= fan_angle_rad / 2. Con fan_angle_rad >= 2π no se
    filtra nada. Al estar en el marco mundo, el abanico sigue al nadir calibrado (angle_offset).
    """
    theta = np.asarray(theta, dtype=np.float64)
    if fan_angle_rad >= 2 * np.pi:
        return np.ones(theta.shape, dtype=bool)
    delta = g.angle_direction * theta + g.angle_offset_rad + np.pi / 2 - fan_center_rad
    delta = (delta + np.pi) % (2 * np.pi) - np.pi
    return np.abs(delta) <= fan_angle_rad / 2


def height_profile(
    x: np.ndarray,
    z: np.ndarray,
    grid: ProfileGrid,
    floor_z: float = 0.0,
    z_min: float = -0.5,
    z_max: float = 10.0,
) -> tuple[np.ndarray, np.ndarray]:
    """Perfil h(x) = max(z - floor_z) por celda de la malla.

    Se usa el máximo porque, mirando desde arriba, la superficie superior es lo
    que caracteriza la carga; las celdas sobre caras verticales también reciben
    puntos intermedios que no deben bajar la altura.

    Devuelve (h, counts). Las celdas sin puntos válidos quedan en NaN (no en 0):
    una sombra/oclusión no es piso.
    """
    h = np.full(grid.n_bins, -np.inf)
    counts = np.zeros(grid.n_bins, dtype=np.int64)
    hz = np.asarray(z) - floor_z
    keep = np.isfinite(x) & np.isfinite(hz) & (hz >= z_min) & (hz <= z_max)
    keep &= (x >= grid.x_min) & (x < grid.x_min + grid.n_bins * grid.dx)
    idx = np.floor((np.asarray(x)[keep] - grid.x_min) / grid.dx).astype(np.int64)
    np.maximum.at(h, idx, hz[keep])
    np.add.at(counts, idx, 1)
    h[~np.isfinite(h)] = np.nan
    return h, counts


def object_mask(h: np.ndarray, h_threshold: float, min_run: int = 1) -> np.ndarray:
    """Celdas por encima del umbral que forman tramos continuos de al menos ``min_run`` celdas.

    El criterio de continuidad (reporte §5.5) descarta outliers aislados que,
    sin él, se contarían como objeto.
    """
    with np.errstate(invalid="ignore"):
        m = np.isfinite(h) & (h > h_threshold)
    if min_run <= 1 or not m.any():
        return m
    edges = np.diff(np.r_[0, m.astype(np.int8), 0])
    starts, ends = np.flatnonzero(edges == 1), np.flatnonzero(edges == -1)
    for s, e in zip(starts, ends):
        if e - s < min_run:
            m[s:e] = False
    return m


def object_extent(h: np.ndarray, grid: ProfileGrid, h_threshold: float,
                  min_run: int = 1) -> tuple[float, float] | None:
    """(x_inicio, x_fin) del objeto, o None: CENTROS de las celdas ocupadas extremas.

    El borde real cae en algún punto de la celda extrema; su centro es la estimación sin sesgo.
    Con los bordes de celda el largo salía +1 celda (2 cm) más largo en promedio, y el volumen,
    que ubica frente y cola con esta extensión, +0.3 % (simulación, 2026-10-24).
    """
    idx = np.flatnonzero(object_mask(h, h_threshold, min_run))
    if idx.size == 0:
        return None
    return grid.x_min + (idx[0] + 0.5) * grid.dx, grid.x_min + (idx[-1] + 0.5) * grid.dx
