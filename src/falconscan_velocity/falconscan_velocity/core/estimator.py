"""Etapas 8-10: conversión física, velocidad, confianza y filtrado temporal.

``VelocityEstimator.process(scan)`` es la única entrada. No depende de ROS:
el nodo solo convierte LaserScan -> Scan y publica el resultado.
"""
from __future__ import annotations

import math
from collections import deque
from dataclasses import dataclass, fields
from enum import IntEnum

import numpy as np

from .correlation import CorrelationResult, find_shift
from .geometry import MountGeometry, ProfileGrid, height_profile, object_extent, object_mask, polar_to_xz
from .scan import Scan, median_filter, validate_ranges


class Status(IntEnum):
    OK = 0
    NO_OBJECT = 1         # escaneo sin objeto en la ROI (se salta sin borrar el historial)
    WARMING_UP = 2        # aún no hay perfil de referencia
    BAD_DT = 3            # Δt <= 0 o demasiado grande
    LOW_OVERLAP = 4       # pocos puntos válidos en común / sin variación
    LOW_PEAK = 5          # máximo de correlación bajo
    AMBIGUOUS = 6         # segundo máximo demasiado cercano al primero
    AT_SEARCH_LIMIT = 7   # máximo en el borde de la ventana: la velocidad real puede estar fuera
    IMPLAUSIBLE = 8       # salto de velocidad incompatible con la aceleración máxima


@dataclass
class VelocityParams:
    # --- Montaje (ficha de calibración, reporte §6 fase 0) ---
    mount_height_m: float = 5.0
    angle_offset_rad: float = -math.pi / 2
    angle_direction: int = 1
    x_offset_m: float = 0.0
    floor_z_m: float = 0.0
    range_bias_m: float = 0.0          # sesgo constante caracterizado; 0 hasta medirlo
    # --- Validación / filtrado ---
    range_min_m: float = 0.2
    range_max_m: float = 12.0
    median_window: int = 3
    median_max_spread_m: float = 0.15  # ventana incompleta: si los válidos discrepan más, se invalida
    # --- Malla / ROI ---
    roi_x_min_m: float = -3.0
    roi_x_max_m: float = 3.0
    grid_dx_m: float = 0.02
    roi_z_max_m: float = 4.5           # descarta puntos por encima de esta altura (estructura del pórtico)
    # --- Segmentación ---
    object_h_threshold_m: float = 0.4
    object_min_bins: int = 25
    object_min_run_bins: int = 5       # tramo continuo mínimo (10 cm con dx = 2 cm)
    empty_reset_scans: int = 5         # escaneos vacíos SEGUIDOS para reiniciar el historial (objeto ido)
    # --- Correlación ---
    baseline_scans: int = 3            # comparar con el escaneo de hace N (Δt mayor = menos ruido en v)
    v_min_mps: float = -0.5            # con search_mode = "sentido"/"fija" (ver _search_range)
    v_max_mps: float = 6.0
    search_mode: str = "simetrica"     # "simetrica" | "sentido" | "fija" (ver _search_range)
    min_overlap_bins: int = 60
    min_profile_std_m: float = 0.05
    peak_exclusion_bins: int = 3
    deskew_iterations: int = 1         # pasadas extra de compensación por rayo (0 = sin deskew)
    # --- Aceptación ---
    min_peak: float = 0.85
    min_prominence: float = 0.05
    max_dt_s: float = 0.5
    max_accel_mps2: float = 3.0
    accel_margin_mps: float = 0.3
    # --- Filtro temporal (EMA sobre v ya estimada, reporte §5.10) ---
    ema_alpha: float = 0.4
    # --- Paso de camión (PassTracker, docs/maquina_estados_paso.md §2) ---
    pass_enter_scans: int = 3          # escaneos ocupados para confirmar la entrada
    pass_exit_scans: int = 5           # escaneos vacíos para confirmar la salida
    pass_empty_scans: int = 5          # escaneos vacíos para salir de ARRANQUE / OBSTRUIDO
    pass_max_s: float = 30.0           # duración máxima de un paso -> TIMEOUT
    scan_timeout_s: float = 0.5        # sin escaneos -> SIN_DATOS
    pass_edge_margin_m: float = 1.0    # franja de borde de la ROI para entrada/salida
    pass_v_min_mps: float = 1.39       # 5 km/h
    pass_v_max_mps: float = 2.78       # 10 km/h
    pass_v_tolerance_mps: float = 0.3
    pass_reverse_v_mps: float = -0.2   # v_avance por debajo = reversa
    pass_reverse_scans: int = 3
    pass_min_valid_fraction: float = 0.6

    @classmethod
    def field_names(cls) -> list[str]:
        return [f.name for f in fields(cls)]

    @property
    def geometry(self) -> MountGeometry:
        return MountGeometry(self.mount_height_m, self.angle_offset_rad, int(self.angle_direction), self.x_offset_m)

    @property
    def grid(self) -> ProfileGrid:
        return ProfileGrid(self.roi_x_min_m, self.roi_x_max_m, self.grid_dx_m)


@dataclass
class VelocityEstimate:
    """Resultado por escaneo.

    ``velocity_mps`` es la velocidad MEDIA del intervalo [stamp - dt_s, stamp];
    con aceleración, corresponde al instante ``mid_stamp`` y no a ``stamp``.
    Para integrar posición/volumen use ``mid_stamp``.
    """

    stamp: float
    status: Status
    mid_stamp: float = float("nan")
    velocity_mps: float = float("nan")            # cruda del intervalo (NaN si no es válida)
    velocity_filtered_mps: float = float("nan")   # EMA; NaN hasta la primera válida
    displacement_m: float = float("nan")
    dt_s: float = float("nan")
    shift_bins: float = float("nan")
    peak: float = float("nan")
    prominence: float = float("nan")
    n_overlap: int = 0
    object_extent: tuple[float, float] | None = None

    @property
    def valid(self) -> bool:
        return self.status == Status.OK


@dataclass
class _Frame:
    """Puntos validados de un escaneo. tau = instante de cada rayo respecto a stamp [s]."""

    stamp: float
    x: np.ndarray
    z: np.ndarray
    tau: np.ndarray


class VelocityEstimator:
    def __init__(self, params: VelocityParams | None = None):
        self.p = params or VelocityParams()
        self._grid = self.p.grid
        self._geom = self.p.geometry
        self._history: deque[_Frame] = deque(maxlen=max(1, self.p.baseline_scans) + 1)
        self._last_valid: tuple[float, float] | None = None  # (stamp, v)
        self._v_filt = float("nan")
        self._n_empty = 0                 # escaneos vacíos consecutivos
        self._direction = 0               # sentido del paso (+1/-1); 0 = desconocido
        self.last_profile: np.ndarray | None = None
        self.last_correlation: CorrelationResult | None = None

    # ------------------------------------------------------------------ API
    def reset(self) -> None:
        self._history.clear()
        self._last_valid = None
        self._v_filt = float("nan")
        self._n_empty = 0

    def set_direction(self, direction: int) -> None:
        """Sentido del paso en curso (+1 / -1), o 0 si no se conoce. Lo fija el PassTracker."""
        self._direction = int(np.sign(direction))

    def _search_range(self) -> tuple[float, float]:
        """Rango de velocidades [m/s] en el que se busca el pico de correlación.

        - "fija": [v_min, v_max] en el marco x del sensor (comportamiento original; no ve
          camiones en -x).
        - "simetrica": [-v_max, v_max] siempre.
        - "sentido": sentido · [v_min, v_max] con el sentido del paso; simétrico mientras no
          se conozca. Mismo ancho que "fija" una vez conocido.
        """
        p = self.p
        if p.search_mode == "fija":
            return p.v_min_mps, p.v_max_mps
        if p.search_mode == "sentido" and self._direction != 0:
            lo, hi = self._direction * p.v_min_mps, self._direction * p.v_max_mps
            return min(lo, hi), max(lo, hi)
        return -p.v_max_mps, p.v_max_mps

    def _frame_from_scan(self, scan: Scan) -> _Frame:
        p = self.p
        r_min = max(p.range_min_m, scan.range_min)
        r_max = min(p.range_max_m, scan.range_max)
        r = validate_ranges(scan.ranges, r_min, r_max) - p.range_bias_m
        r = median_filter(r, p.median_window, max_spread=p.median_max_spread_m)
        x, z = polar_to_xz(r, scan.angles, self._geom)
        tau = np.arange(r.size) * scan.time_increment
        ok = np.isfinite(x)
        return _Frame(scan.stamp, x[ok], z[ok], tau[ok])

    def _profile(self, f: _Frame, v: float) -> np.ndarray:
        """Perfil h(x) con compensación de movimiento durante el barrido (deskew).

        El rayo i se dispara tau_i después de `stamp`; si el objeto avanza a v,
        el punto estaba en x - v·tau_i en el instante `stamp`. Sin esto, la
        velocidad queda sesgada ~ v / v_barrido (crece con v²).
        """
        x = f.x - v * f.tau if v else f.x
        h, _ = height_profile(x, f.z, self._grid, self.p.floor_z_m, z_max=self.p.roi_z_max_m)
        return h

    def profile_from_scan(self, scan: Scan, v: float = 0.0) -> np.ndarray:
        return self._profile(self._frame_from_scan(scan), v)

    def process(self, scan: Scan) -> VelocityEstimate:
        p = self.p
        frame = self._frame_from_scan(scan)
        v_hint = self._v_filt if (np.isfinite(self._v_filt) and p.deskew_iterations > 0) else 0.0
        h = self._profile(frame, v_hint)
        self.last_profile = h
        mask = object_mask(h, p.object_h_threshold_m, p.object_min_run_bins)
        out = VelocityEstimate(stamp=scan.stamp, status=Status.NO_OBJECT)

        if int(mask.sum()) < p.object_min_bins:
            # Un escaneo vacío aislado (dropout, hueco cabina–volco) se salta sin borrar el
            # historial: el siguiente escaneo con objeto se compara con la referencia guardada
            # (Δt algo mayor) y no vuelve a WARMING_UP. Solo se reinicia cuando el objeto se fue.
            # Un hueco largo lo acota igual la verificación BAD_DT.
            self._n_empty += 1
            if self._n_empty >= p.empty_reset_scans:
                self.reset()
            return out
        self._n_empty = 0
        out.object_extent = object_extent(h, self._grid, p.object_h_threshold_m, p.object_min_run_bins)

        self._history.append(frame)
        out.velocity_filtered_mps = self._v_filt
        if len(self._history) < self._history.maxlen:
            out.status = Status.WARMING_UP
            return out

        ref = self._history[0]
        dt = scan.stamp - ref.stamp
        out.dt_s = dt
        out.mid_stamp = ref.stamp + 0.5 * dt
        if not (0.0 < dt <= p.max_dt_s * max(1, p.baseline_scans)):
            out.status = Status.BAD_DT
            self._history.clear()
            self._history.append(frame)
            return out

        dx = self._grid.dx
        v_lo, v_hi = self._search_range()
        k_min = math.floor(v_lo * dt / dx) - 1
        k_max = math.ceil(v_hi * dt / dx) + 1
        deskew = p.deskew_iterations > 0 and bool(np.any(frame.tau > 0))
        n_pass = 1 + p.deskew_iterations if deskew else 1
        v_pass = v_hint if deskew else 0.0
        for _ in range(n_pass):
            corr = find_shift(self._profile(ref, v_pass), self._profile(frame, v_pass), k_min, k_max,
                              p.min_overlap_bins, p.min_profile_std_m, p.peak_exclusion_bins)
            if not corr.found or not deskew:
                break
            v_pass = corr.shift_bins * dx / dt
        self.last_correlation = corr
        out.shift_bins, out.peak, out.prominence, out.n_overlap = (
            corr.shift_bins, corr.peak, corr.prominence, corr.n_overlap)

        if not corr.found:
            out.status = Status.LOW_OVERLAP
            return out
        ds = corr.shift_bins * dx
        v = ds / dt
        out.displacement_m = ds
        if corr.at_search_limit:
            out.status = Status.AT_SEARCH_LIMIT
        elif corr.peak < p.min_peak:
            out.status = Status.LOW_PEAK
        elif corr.prominence < p.min_prominence:
            out.status = Status.AMBIGUOUS
        elif self._last_valid is not None and abs(v - self._last_valid[1]) > (
                p.max_accel_mps2 * (scan.stamp - self._last_valid[0]) + p.accel_margin_mps):
            out.status = Status.IMPLAUSIBLE
        else:
            out.status = Status.OK

        if out.status != Status.OK:
            return out

        out.velocity_mps = v
        self._last_valid = (scan.stamp, v)
        a = p.ema_alpha
        self._v_filt = v if not np.isfinite(self._v_filt) else a * v + (1 - a) * self._v_filt
        out.velocity_filtered_mps = self._v_filt
        return out
