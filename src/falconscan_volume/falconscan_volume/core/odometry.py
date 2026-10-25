"""Posición de avance x(t) a partir de /velocity_estimate (§3.2).

- Se integra la velocidad en ``mid_stamp``; NO se suma ``displacement_m``: con
  ``baseline_scans = 3`` los intervalos se solapan y se contaría tres veces.
- v(t) es lineal a trozos entre muestras OK; x(t) es su integral exacta.
- Huecos de velocidad ≤ ``max_gap_s`` se interpolan (marca ``interpolada``); mayores dejan el
  instante sin posición (``sin_x``). Fuera del rango de muestras se extrapola con la última
  velocidad OK solo si se pide (marca ``extrapolada``).

También guarda la extensión del objeto vista por el LiDAR de velocidad en cada escaneo, que
sirve para ubicar el frente y la cola del camión en el plano de volumen y para los chequeos
de sincronía y coherencia entre los dos LiDAR.
"""
from __future__ import annotations

import math
from collections import deque
from dataclasses import dataclass

import numpy as np

_EDGE_MAX_STEP_M = 1.0     # salto máximo del borde entre escaneos para aceptar el cruce
_EDGE_MAX_DT_S = 0.5       # separación máxima entre los dos escaneos del cruce


@dataclass
class VelocitySample:
    stamp: float                  # instante del escaneo de velocidad (extensión del objeto)
    mid_stamp: float              # instante de la velocidad (punto medio del intervalo)
    velocity_mps: float           # marco x del LiDAR de velocidad, con signo; NaN si no es válida
    valid: bool
    x_start: float = math.nan     # extensión del objeto en la ROI de velocidad
    x_end: float = math.nan
    pass_id: int = 0
    direction: int = 0


@dataclass
class Positions:
    x: np.ndarray                 # avance [m] (origen arbitrario, común a todas las consultas)
    interpolated: np.ndarray      # bool: hueco de velocidad interpolado
    missing: np.ndarray           # bool: sin posición (hueco > max_gap_s)
    extrapolated: np.ndarray      # bool: fuera del rango de muestras


class VelocityHistory:
    def __init__(self, max_age_s: float = 120.0):
        self.max_age_s = max_age_s
        self._s: deque[VelocitySample] = deque()

    def __len__(self) -> int:
        return len(self._s)

    def append(self, s: VelocitySample) -> None:
        self._s.append(s)
        while self._s and self._s[-1].stamp - self._s[0].stamp > self.max_age_s:
            self._s.popleft()

    def _valid(self) -> tuple[np.ndarray, np.ndarray]:
        ok = [(s.mid_stamp, s.velocity_mps) for s in self._s
              if s.valid and math.isfinite(s.mid_stamp) and math.isfinite(s.velocity_mps)]
        if not ok:
            return np.empty(0), np.empty(0)
        a = np.array(sorted(ok))
        # mid_stamp repetidos (no debería pasar): se conserva el último
        keep = np.r_[np.diff(a[:, 0]) > 1e-9, True]
        return a[keep, 0], a[keep, 1]

    def covers(self, t: float) -> bool:
        """Hay una velocidad OK con mid_stamp ≥ t (se puede interpolar hasta t)."""
        tv, _ = self._valid()
        return tv.size > 0 and tv[-1] >= t

    def last_valid_velocity(self) -> float:
        _, v = self._valid()
        return float(v[-1]) if v.size else math.nan

    def median_velocity(self, t0: float, t1: float) -> float:
        tv, v = self._valid()
        sel = (tv >= t0) & (tv <= t1)
        return float(np.median(v[sel])) if sel.any() else math.nan

    def positions(self, times, direction: int, max_gap_s: float, extrapolate: bool = False) -> Positions:
        """Avance ``direction · ∫ v dt`` en los instantes pedidos."""
        t = np.atleast_1d(np.asarray(times, dtype=float))
        n = t.size
        tv, v = self._valid()
        if tv.size == 0 or direction == 0:
            nan = np.full(n, math.nan)
            return Positions(nan, np.zeros(n, bool), np.ones(n, bool), np.zeros(n, bool))
        a = direction * v
        if tv.size == 1:
            x = a[0] * (t - tv[0])
            dist = np.abs(t - tv[0])
            missing = dist > (max_gap_s if extrapolate else 0.5 * max_gap_s)
            return Positions(x, dist > 1e-9, missing, dist > 1e-9)
        gaps = np.diff(tv)
        nominal = float(np.median(gaps))
        X = np.r_[0.0, np.cumsum(0.5 * (a[1:] + a[:-1]) * gaps)]

        k = np.clip(np.searchsorted(tv, t, side="right") - 1, 0, tv.size - 2)
        tau = t - tv[k]
        g = gaps[k]
        slope = (a[k + 1] - a[k]) / g
        x = X[k] + a[k] * tau + 0.5 * slope * tau ** 2
        before, after = t < tv[0], t > tv[-1]
        # fuera del rango: velocidad constante (la del extremo)
        x = np.where(before, a[0] * (t - tv[0]), x)
        x = np.where(after, X[-1] + a[-1] * (t - tv[-1]), x)

        inside = ~before & ~after
        interpolated = inside & (g > 1.5 * nominal)
        missing = inside & (g > max_gap_s)
        dist = np.where(before, tv[0] - t, np.where(after, t - tv[-1], 0.0))
        lim = max_gap_s if extrapolate else 0.5 * max_gap_s
        extrapolated = (before | after) & (dist > 1.5 * nominal)
        missing |= (before | after) & (dist > lim)
        return Positions(x, interpolated | extrapolated, missing, extrapolated)

    def advance(self, t0: float, t1: float, direction: int) -> float:
        """Avance entre dos instantes (extrapolando); NaN si no hay velocidad."""
        pos = self.positions([t0, t1], direction or 1, max_gap_s=math.inf, extrapolate=True)
        return float(pos.x[1] - pos.x[0])

    def edge_crossing(self, x_plane: float, direction: int, edge: str, t0: float, t1: float) -> float:
        """Instante en que el frente (``edge="front"``) o la cola (``"tail"``) del objeto, visto
        por el LiDAR de velocidad, cruza ``x_plane``. NaN si no se observó el cruce.

        Con ``direction = +1`` el frente es ``x_end`` y la cola ``x_start``; con -1, al revés.
        Las extensiones están referidas al instante ``stamp`` del escaneo (el estimador
        compensa el barrido).
        """
        if direction == 0:
            return math.nan
        use_end = (edge == "front") == (direction > 0)
        pts = [(s.stamp, s.x_end if use_end else s.x_start) for s in self._s
               if t0 <= s.stamp <= t1 and math.isfinite(s.x_start) and math.isfinite(s.x_end)]
        for (ta, ea), (tb, eb) in zip(pts, pts[1:]):
            ga, gb = direction * (ea - x_plane), direction * (eb - x_plane)
            if ga < 0 <= gb and tb - ta <= _EDGE_MAX_DT_S and abs(eb - ea) <= _EDGE_MAX_STEP_M:
                return ta + (tb - ta) * (-ga) / (gb - ga)
        return math.nan

    def extent_near(self, t: float, max_dt: float) -> tuple[float, float] | None:
        """Extensión del objeto en el escaneo de velocidad más cercano a t (o None)."""
        best, best_dt = None, max_dt
        for s in reversed(self._s):
            dt = abs(s.stamp - t)
            if dt <= best_dt:
                best, best_dt = s, dt
            elif s.stamp < t - max_dt:
                break
        if best is None:
            return None
        if not (math.isfinite(best.x_start) and math.isfinite(best.x_end)):
            return (math.nan, math.nan)
        return best.x_start, best.x_end
