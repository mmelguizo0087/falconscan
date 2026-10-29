"""Perfil transversal: un escaneo del LiDAR de volumen -> sección del objeto (§3.1).

Marco: y = transversal (0 = eje del carril), z = vertical (0 = piso). El perfil es h(y) en
una malla uniforme; inválido = NaN siempre (una sombra no es piso). Dentro del objeto, los
huecos cortos de NaN se interpolan entre dos alturas medidas (un modelo, no un 0); los
largos se marcan como ``shadow``.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field

import numpy as np

from falconscan_velocity.core import Scan, fan_mask, height_profile, median_filter, polar_to_xz, validate_ranges

from .params import VolumeParams

_CLOUD_MARGIN_M = 0.3      # puntos de piso a cada lado del objeto que se dejan en la nube (solo visual)
_MIN_FLOOR_CELLS = 10      # celdas de piso visibles mínimas para evaluar la deriva


@dataclass
class Section:
    stamp: float                       # instante del perfil (media de los rayos que tocan el objeto)
    occupied: bool
    no_data: bool = False              # escaneo perdido o casi sin puntos: NO es un perfil vacío
    area_m2: float = 0.0
    y0: float = math.nan               # extensión lateral del objeto
    y1: float = math.nan
    max_h: float = math.nan
    h: np.ndarray | None = None        # h(y) por celda (NaN = sin dato), con el tramo del objeto interpolado
    nan_cells: int = 0                 # celdas interpoladas dentro del objeto
    span_cells: int = 0
    truncated: bool = False            # toca el borde lateral de la ROI
    shadow: bool = False               # hueco de NaN demasiado largo dentro del objeto
    high: bool = False                 # puntos por encima de vol_roi_z_max_m sobre el objeto
    floor_drift: bool = False          # el piso a los lados no está en 0 ± tolerancia
    floor_dev_m: float = math.nan
    points: np.ndarray = field(default_factory=lambda: np.empty((0, 3)))   # (y, z, t) para la nube

    @property
    def yc(self) -> float:
        return 0.5 * (self.y0 + self.y1)

    @property
    def width(self) -> float:
        return self.y1 - self.y0


def _runs(mask: np.ndarray) -> list[tuple[int, int]]:
    """Tramos [inicio, fin) donde mask es True."""
    e = np.diff(np.r_[0, mask.astype(np.int8), 0])
    return list(zip(np.flatnonzero(e == 1), np.flatnonzero(e == -1)))


class SectionProcessor:
    def __init__(self, params: VolumeParams):
        self.p = params
        self.grid = params.grid
        self.geom = params.geometry
        self._min_run = max(1, int(round(params.vol_object_min_run_m / params.vol_grid_dy_m)))
        self._max_nan_run = max(1, int(round(params.vol_max_nan_run_m / params.vol_grid_dy_m)))
        self._edge_cells = int(round(params.vol_edge_margin_m / params.vol_grid_dy_m))

    @property
    def centers(self) -> np.ndarray:
        return self.grid.centers

    def process(self, scan: Scan) -> Section:
        p, g = self.p, self.grid
        r_min = max(p.range_min_m, scan.range_min)
        r_max = min(p.range_max_m, scan.range_max)
        r = (validate_ranges(scan.ranges, r_min, r_max) - p.range_bias_m) * p.range_scale
        r[~fan_mask(scan.angles, self.geom, p.fan_angle_rad, p.fan_center_rad)] = np.nan
        r = median_filter(r, p.median_window, max_spread=p.median_max_spread_m)
        y, z = polar_to_xz(r, scan.angles, self.geom)
        t = scan.stamp + p.vol_time_offset_s + np.arange(r.size) * scan.time_increment
        ok = np.isfinite(y)
        y, z, t = y[ok], z[ok], t[ok]
        hz = z - p.floor_z_m

        h, _ = height_profile(y, z, g, p.floor_z_m, z_max=p.vol_roi_z_max_m)
        dy = g.dx
        if np.isfinite(h).mean() < p.vol_min_data_fraction:
            # Sin datos no se sabe si hay objeto: tratarlo como vacío pondría A = 0 en medio
            # del camión (con 5 % de escaneos perdidos, −6 % de volumen en simulación).
            return Section(stamp=self._nadir_time(scan), occupied=False, no_data=True, h=h)
        with np.errstate(invalid="ignore"):
            above = np.isfinite(h) & (h > p.vol_h_threshold_m)
        # Tramos del objeto: celdas sobre el umbral unidas a través de huecos de NaN (un rayo
        # perdido al pie de una pared no debe partir el objeto). Un tramo cuenta si tiene al
        # menos _min_run celdas sobre el umbral (descarta outliers aislados).
        runs = [(a, b) for a, b in _runs(above | ~np.isfinite(h)) if above[a:b].sum() >= self._min_run]
        n_above = sum(int(above[a:b].sum()) for a, b in runs)
        if n_above * dy < p.vol_object_min_width_m:
            sec = Section(stamp=self._nadir_time(scan), occupied=False, h=h)
            self._floor_check(sec, h, np.ones(g.n_bins, dtype=bool))
            return sec

        i0 = runs[0][0] + int(np.argmax(above[runs[0][0]:runs[0][1]]))
        i1 = runs[-1][1] - int(np.argmax(above[runs[-1][0]:runs[-1][1]][::-1]))
        span = h[i0:i1].copy()
        nan = ~np.isfinite(span)
        shadow = any(b - a > self._max_nan_run for a, b in _runs(nan))
        if nan.any():
            k = np.arange(span.size)
            span[nan] = np.interp(k[nan], k[~nan], span[~nan])
        h = h.copy()
        h[i0:i1] = span
        c0, c1 = g.x_min + i0 * dy, g.x_min + i1 * dy
        y0, y1 = self._edges(y, hz, c0, c1)

        in_span = (y >= c0) & (y < c1)
        # Área: celdas completas, salvo las de borde, que cuentan solo hasta el borde real.
        hs = np.clip(span, 0.0, None)
        widths = np.full(span.size, dy)
        widths[0] -= y0 - c0
        widths[-1] -= c1 - y1
        sec = Section(
            stamp=float(np.mean(t[in_span])) if in_span.any() else self._nadir_time(scan),
            occupied=True,
            area_m2=float(np.sum(hs * np.clip(widths, 0.0, None))),
            y0=y0, y1=y1,
            max_h=float(np.max(span)),
            h=h,
            nan_cells=int(nan.sum()),
            span_cells=int(span.size),
            truncated=i0 <= self._edge_cells or i1 >= g.n_bins - self._edge_cells,
            shadow=shadow,
            high=int(np.sum(in_span & (hz > p.vol_roi_z_max_m))) >= 3,
        )
        outside = np.ones(g.n_bins, dtype=bool)
        outside[max(0, i0 - self._min_run):i1 + self._min_run] = False
        self._floor_check(sec, h, outside)
        near = (y >= y0 - _CLOUD_MARGIN_M) & (y < y1 + _CLOUD_MARGIN_M) & (hz <= p.vol_roi_z_max_m)
        sec.points = np.column_stack([y[near], hz[near], t[near]])
        return sec

    def _edges(self, y: np.ndarray, hz: np.ndarray, c0: float, c1: float) -> tuple[float, float]:
        """Bordes laterales del objeto con resolución de punto (no de celda).

        Con la malla de 2 cm, la celda de borde contaría completa aunque el objeto la cubra en
        parte (sesgo de hasta +1 % por lado en una cabina). El borde se ubica en el punto del
        objeto más externo, más media separación entre puntos vecinos.
        """
        dy = self.grid.dx
        obj = (hz > self.p.vol_h_threshold_m) & (y >= c0) & (y < c1)
        if obj.sum() < 2:
            return c0, c1
        yo = np.sort(y[obj])
        half = 0.5 * float(np.median(np.diff(yo)))
        e0 = yo[0] - half if yo[0] < c0 + dy else c0
        e1 = yo[-1] + half if yo[-1] >= c1 - dy else c1
        return max(c0, e0), min(c1, e1)

    def _floor_check(self, sec: Section, h: np.ndarray, region: np.ndarray) -> None:
        """Piso visible fuera del objeto: debe estar en 0 ± vol_floor_drift_tol_m."""
        with np.errstate(invalid="ignore"):
            floor = region & np.isfinite(h) & (h < self.p.vol_h_threshold_m)
        if floor.sum() >= _MIN_FLOOR_CELLS:
            sec.floor_dev_m = float(np.median(h[floor]))
            sec.floor_drift = abs(sec.floor_dev_m) > self.p.vol_floor_drift_tol_m

    def _nadir_time(self, scan: Scan) -> float:
        """Instante del rayo que apunta al nadir (perfil vacío: no hay rayos de objeto)."""
        g = self.geom
        theta_nadir = (-math.pi / 2 - g.angle_offset_rad) / g.angle_direction
        d = np.angle(np.exp(1j * (scan.angles - theta_nadir)))
        i = int(np.argmin(np.abs(d)))
        return scan.stamp + self.p.vol_time_offset_s + i * scan.time_increment
