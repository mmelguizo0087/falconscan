"""Etapas 1-3 del reporte técnico: adquisición, validación de rangos y filtrado espacial.

Convención: las muestras inválidas se representan con NaN durante TODO el
procesamiento (reporte §5.2). Nunca se reemplazan por 0 ni por r_max.
"""
from __future__ import annotations

import warnings
from dataclasses import dataclass

import numpy as np
from numpy.lib.stride_tricks import sliding_window_view


@dataclass(frozen=True)
class Scan:
    """Escaneo independiente de ROS (equivalente a sensor_msgs/LaserScan).

    stamp: marca temporal del primer rayo del escaneo [s].
    """

    stamp: float
    angle_min: float
    angle_increment: float
    ranges: np.ndarray
    time_increment: float = 0.0
    range_min: float = 0.0
    range_max: float = float("inf")

    @property
    def angles(self) -> np.ndarray:
        return self.angle_min + np.arange(np.asarray(self.ranges).size) * self.angle_increment


def validate_ranges(ranges: np.ndarray, r_min: float, r_max: float) -> np.ndarray:
    """Devuelve una copia en float64 con NaN en toda muestra no finita o fuera de [r_min, r_max].

    La máscara de validez es implícita: ``np.isfinite(resultado)``.
    """
    r = np.asarray(ranges, dtype=np.float64).copy()
    with np.errstate(invalid="ignore"):
        valid = np.isfinite(r) & (r >= r_min) & (r <= r_max)
    r[~valid] = np.nan
    return r


def median_filter(r: np.ndarray, window: int = 3, min_valid: int | None = None,
                  max_spread: float = 0.15) -> np.ndarray:
    """Mediana angular que ignora NaN (reporte §5.3).

    - Una muestra inválida sigue siendo inválida (no se rellenan huecos).
    - Si la ventana tiene menos de ``min_valid`` muestras válidas, la salida es NaN.
    - Ventana INCOMPLETA (hay NaN): la mediana de un número par de valores es un
      promedio y dejaría pasar medio outlier. Por eso, si los valores válidos
      discrepan más de ``max_spread`` [m], la salida es NaN.
    - Extremos del vector: se rellenan con NaN (sin vecindad circular).
    """
    r = np.asarray(r, dtype=np.float64)
    if window <= 1:
        return r.copy()
    if window % 2 == 0:
        raise ValueError("window debe ser impar")
    half = window // 2
    if min_valid is None:
        min_valid = half + 1
    padded = np.pad(r, half, mode="constant", constant_values=np.nan)
    win = sliding_window_view(padded, window)
    count = np.sum(np.isfinite(win), axis=1)
    with warnings.catch_warnings():
        warnings.simplefilter("ignore", category=RuntimeWarning)
        med = np.nanmedian(win, axis=1)
        spread = np.nanmax(win, axis=1) - np.nanmin(win, axis=1)
    ok = np.isfinite(r) & (count >= min_valid)
    ok &= (count == window) | (spread <= max_spread)
    return np.where(ok, med, np.nan)
