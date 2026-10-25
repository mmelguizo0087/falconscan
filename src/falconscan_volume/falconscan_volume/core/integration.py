"""Integración del volumen V = ∫ A(x) dx sobre los perfiles ubicados en x (§3.3).

1. Se ordenan los perfiles por x (si el camión se detuvo o retrocedió, x no es monótona).
2. Los perfiles a menos de ``merge_dx`` se fusionan (camión detenido): mediana.
3. El objeto queda entre el último perfil vacío antes del frente y el primero después de la
   cola; los perfiles vacíos interiores (espacio entre partes) cuentan con A = 0.
4. Extremos:
   - "punto_medio": trapecios con A = 0 en los perfiles vacíos que acotan el objeto. Equivale
     a suponer el frente/la cola a mitad de camino entre perfiles: sin sesgo en promedio,
     σ = A·Δx/√12 por extremo.
   - "velocidad": el LiDAR de velocidad da la posición del frente y la cola (resolución de la
     malla, 2 cm). Se supone la cara vertical: A del primer/último perfil ocupado hasta el borde.
"""
from __future__ import annotations

import math
import warnings
from dataclasses import dataclass, field, replace

import numpy as np

from .section import Section

_EDGE_SIGMA_M = 0.03     # incertidumbre supuesta del borde ubicado con el LiDAR de velocidad


@dataclass
class Profile:
    x: float                 # avance [m] en el instante del perfil
    section: Section
    v: float = 0.0           # rapidez de avance en ese instante (ubica cada punto en x)


@dataclass
class IntegrationResult:
    volume_m3: float
    end_uncertainty_m3: float
    edge_method: str
    profiles: list[Profile] = field(default_factory=list)   # del vacío antes del frente al vacío después de la cola
    x_front: float = math.nan
    x_tail: float = math.nan
    max_dx_m: float = math.nan
    n_occupied: int = 0
    front_edge_used: bool = False
    tail_edge_used: bool = False

    @property
    def length_m(self) -> float:
        return self.x_tail - self.x_front


def _merge(group: list[Profile]) -> Profile:
    if len(group) == 1:
        return group[0]
    secs = [g.section for g in group]
    occ = [s for s in secs if s.occupied]
    base = occ[len(occ) // 2] if len(occ) * 2 > len(secs) else next(s for s in secs if not s.occupied)
    merged = replace(base)
    if merged.occupied:
        merged.area_m2 = float(np.median([s.area_m2 for s in occ]))
        with warnings.catch_warnings():
            warnings.simplefilter("ignore", RuntimeWarning)     # columnas todo-NaN siguen en NaN
            merged.h = np.nanmedian(np.vstack([s.h for s in occ]), axis=0)
    return Profile(float(np.mean([g.x for g in group])), merged)


def order_and_merge(profiles: list[Profile], merge_dx: float) -> list[Profile]:
    ps = sorted((p for p in profiles if math.isfinite(p.x)), key=lambda p: p.x)
    out: list[Profile] = []
    group: list[Profile] = []
    for p in ps:
        if group and p.x - group[0].x >= merge_dx:
            out.append(_merge(group))
            group = []
        group.append(p)
    if group:
        out.append(_merge(group))
    return out


def integrate(profiles: list[Profile], merge_dx: float,
              x_front: float = math.nan, x_tail: float = math.nan) -> IntegrationResult:
    ps = order_and_merge(profiles, merge_dx)
    occ = [i for i, p in enumerate(ps) if p.section.occupied]
    if not occ:
        return IntegrationResult(math.nan, math.nan, "punto_medio", ps)
    i_f, i_l = occ[0], occ[-1]
    i_b = i_f - 1 if i_f > 0 else None              # último vacío antes del frente
    i_a = i_l + 1 if i_l + 1 < len(ps) else None    # primer vacío después de la cola
    kept = ps[(i_b if i_b is not None else i_f):(i_a + 1 if i_a is not None else i_l + 1)]

    x = np.array([p.x for p in ps])
    area = np.array([p.section.area_m2 if p.section.occupied else 0.0 for p in ps])
    xi, ai = x[i_f:i_l + 1], area[i_f:i_l + 1]
    v_core = float(np.sum(0.5 * (ai[1:] + ai[:-1]) * np.diff(xi))) if xi.size > 1 else 0.0

    def end(i_occ: int, i_empty: int | None, x_edge: float, sign: int) -> tuple[float, float, float, bool]:
        """(volumen del extremo, σ, posición del borde, se usó el borde de velocidad)."""
        a, xo = area[i_occ], x[i_occ]
        if i_empty is not None:
            xe = x[i_empty]
            lo, hi = min(xe, xo), max(xe, xo)
            if math.isfinite(x_edge) and lo <= x_edge <= hi:
                return a * abs(xo - x_edge), a * _EDGE_SIGMA_M, x_edge, True
            gap = abs(xo - xe)
            return 0.5 * a * gap, a * gap / math.sqrt(12), 0.5 * (xo + xe), False
        # sin perfil vacío que acote (captura cortada): media separación típica hacia afuera
        if math.isfinite(x_edge) and sign * (xo - x_edge) >= 0:
            return a * abs(xo - x_edge), a * _EDGE_SIGMA_M, x_edge, True
        gap = float(np.median(np.diff(xi))) if xi.size > 1 else 0.0
        return 0.5 * a * gap, a * gap / math.sqrt(12), xo - sign * 0.5 * gap, False

    v_f, s_f, xf, used_f = end(i_f, i_b, x_front, +1)
    v_t, s_t, xt, used_t = end(i_l, i_a, x_tail, -1)
    method = "velocidad" if used_f and used_t else ("mixto" if used_f or used_t else "punto_medio")
    xk = np.array([p.x for p in kept])
    return IntegrationResult(
        volume_m3=v_core + v_f + v_t,
        end_uncertainty_m3=math.hypot(s_f, s_t),
        edge_method=method,
        profiles=kept,
        x_front=xf, x_tail=xt,
        max_dx_m=float(np.max(np.diff(xk))) if xk.size > 1 else math.nan,
        n_occupied=len(occ),
        front_edge_used=used_f, tail_edge_used=used_t,
    )
