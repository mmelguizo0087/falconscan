"""Etapa 7: correlación cruzada normalizada con máscara de validez (reporte §5.7).

Convención de signo: si b(j + k) ≈ a(j), el patrón avanzó +k celdas en +x.
"""
from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np


@dataclass
class CorrelationResult:
    shift_bins: float = float("nan")      # k* con refinamiento subcelda
    k_int: int = 0                         # k* entero
    peak: float = float("nan")             # C(k*)
    second_peak: float = float("nan")      # mayor máximo local fuera de la zona del pico
    n_overlap: int = 0                     # puntos válidos usados en C(k*)
    at_search_limit: bool = False          # el máximo cayó en el borde de la ventana K
    lags: np.ndarray = field(default_factory=lambda: np.empty(0, dtype=int))
    curve: np.ndarray = field(default_factory=lambda: np.empty(0))
    counts: np.ndarray = field(default_factory=lambda: np.empty(0, dtype=int))

    @property
    def found(self) -> bool:
        return np.isfinite(self.peak)

    @property
    def prominence(self) -> float:
        """Diferencia entre el máximo y el segundo máximo. Si no hay rival, = 1 + peak."""
        if not np.isfinite(self.peak):
            return float("nan")
        if not np.isfinite(self.second_peak):
            return 1.0 + self.peak
        return self.peak - self.second_peak


def masked_ncc(
    a: np.ndarray,
    b: np.ndarray,
    k_min: int,
    k_max: int,
    min_overlap: int,
    min_std: float,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """C(k) para k en [k_min, k_max] usando solo índices válidos en ambos perfiles (Ω_k).

    Medias y varianzas se recalculan sobre Ω_k. C(k) = NaN si |Ω_k| < min_overlap o
    si alguno de los dos tramos tiene desviación estándar < min_std.
    """
    n = a.size
    lags = np.arange(k_min, k_max + 1)
    curve = np.full(lags.size, np.nan)
    counts = np.zeros(lags.size, dtype=np.int64)
    for i, k in enumerate(lags):
        if abs(k) >= n:
            continue
        if k >= 0:
            aa, bb = a[: n - k], b[k:]
        else:
            aa, bb = a[-k:], b[: n + k]
        m = np.isfinite(aa) & np.isfinite(bb)
        cnt = int(m.sum())
        counts[i] = cnt
        if cnt < min_overlap:
            continue
        xa = aa[m] - aa[m].mean()
        xb = bb[m] - bb[m].mean()
        sa = np.sqrt(np.dot(xa, xa))
        sb = np.sqrt(np.dot(xb, xb))
        if sa / np.sqrt(cnt) < min_std or sb / np.sqrt(cnt) < min_std:
            continue
        curve[i] = float(np.dot(xa, xb) / (sa * sb))
    return lags, curve, counts


def _parabolic_offset(cm1: float, c0: float, cp1: float) -> float:
    """Desplazamiento subcelda del vértice de la parábola por tres puntos, en [-0.5, 0.5]."""
    if not (np.isfinite(cm1) and np.isfinite(cp1)):
        return 0.0
    den = cm1 - 2.0 * c0 + cp1
    if den >= 0:  # no es un máximo cóncavo
        return 0.0
    return float(np.clip(0.5 * (cm1 - cp1) / den, -0.5, 0.5))


def find_shift(
    a: np.ndarray,
    b: np.ndarray,
    k_min: int,
    k_max: int,
    min_overlap: int,
    min_std: float,
    peak_exclusion: int = 2,
) -> CorrelationResult:
    lags, curve, counts = masked_ncc(a, b, k_min, k_max, min_overlap, min_std)
    res = CorrelationResult(lags=lags, curve=curve, counts=counts)
    if not np.any(np.isfinite(curve)):
        return res
    i = int(np.nanargmax(curve))
    res.k_int = int(lags[i])
    res.peak = float(curve[i])
    res.n_overlap = int(counts[i])
    res.at_search_limit = i == 0 or i == lags.size - 1
    cm1 = curve[i - 1] if i > 0 else np.nan
    cp1 = curve[i + 1] if i < lags.size - 1 else np.nan
    res.shift_bins = res.k_int + _parabolic_offset(cm1, res.peak, cp1)

    # Segundo máximo LOCAL fuera de la zona del pico -> ambigüedad (perfiles periódicos/planos)
    c = np.where(np.isfinite(curve), curve, -np.inf)
    left = np.r_[-np.inf, c[:-1]]
    right = np.r_[c[1:], -np.inf]
    is_local_max = np.isfinite(c) & (c >= left) & (c >= right)
    far = np.abs(lags - res.k_int) > peak_exclusion
    rivals = c[is_local_max & far]
    if rivals.size:
        res.second_peak = float(rivals.max())
    return res
