"""Chequeos de la medición (§3.4): trayectoria, sincronía, coherencia entre LiDAR y banderas.

Cada bandera es INVALIDANTE (anula la medición) o ADVERTENCIA. Las banderas heredadas del
paso de velocidad llegan con prefijo ``PASO_`` y son advertencias: el volumen ya revisa por su
cuenta lo que le importa de la velocidad (cobertura, huecos).
"""
from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

from .integration import IntegrationResult
from .odometry import VelocityHistory
from .params import VolumeParams
from .section import Section

INVALIDATING = {
    "TRUNCADO_LATERAL", "VELOCIDAD_INSUFICIENTE", "HUECO_LONGITUDINAL", "SOMBRAS",
    "LARGO_FUERA_DE_RANGO", "POCOS_PERFILES", "TIMEOUT", "SENSOR_PERDIDO", "SIN_VELOCIDAD",
    "DESINCRONIZADO", "INCOHERENCIA_LIDAR", "CONDUCCION_NO_PARALELA", "DESCENTRADO", "SIN_SENTIDO",
}
WARNINGS = {
    "VELOCIDAD_INTERPOLADA", "VELOCIDAD_EXTRAPOLADA", "DERIVA_PISO", "CARGA_ALTA",
    "BORDES_SIN_VELOCIDAD", "SINCRONIA_NO_VERIFICADA", "SIN_PASO",
}

RESULT_VALID, RESULT_WARNINGS, RESULT_INVALID, RESULT_DISCARDED = 0, 1, 2, 3
RESULT_TEXT = {0: "VALIDA", 1: "CON_ADVERTENCIAS", 2: "INVALIDA", 3: "DESCARTADA"}

_COHERENCE_MAX_DT_S = 0.15     # escaneo de velocidad más cercano a un perfil
_COHERENCE_MARGIN_M = 0.3      # holgura de la extensión del LiDAR de velocidad
_EDGE_MATCH_M = 0.10           # bordes de perfiles vecinos "iguales" (perfil estable, no de transición)


@dataclass
class Trajectory:
    center_offset_m: float = math.nan     # mediana del centro transversal (respecto al eje y = 0)
    max_lateral_dev_m: float = math.nan   # máx |centro − mediana|
    yaw_rad: float = math.nan             # pendiente de la recta centro(x)
    path_rms_m: float = math.nan
    n: int = 0


def stable_profiles(secs: list[Section]) -> np.ndarray:
    """Perfiles cuyos DOS bordes coinciden (± _EDGE_MATCH_M) con los de algún vecino en x.

    Excluye los perfiles de transición: cuando el barrido cruza un escalón (frente, fin de la
    cabina, inicio del volco, cola), un lado ve un nivel y el otro, otro; el centro de ese
    perfil no es el del camión. Un camión en diagonal mueve los dos bordes juntos y poco
    (Δx · tan ψ ≈ 3 cm a 10 km/h con 6°), así que sus perfiles siguen contando.
    """
    y0 = np.array([s.y0 for s in secs])
    y1 = np.array([s.y1 for s in secs])
    ok = np.zeros(len(secs), bool)
    for k in range(len(secs)):
        for j in (k - 1, k + 1):
            if 0 <= j < len(secs) and abs(y0[k] - y0[j]) <= _EDGE_MATCH_M and abs(y1[k] - y1[j]) <= _EDGE_MATCH_M:
                ok[k] = True
    return ok


def trajectory(xs: np.ndarray, secs: list[Section]) -> Trajectory:
    """Centro transversal de los perfiles estables (ordenados por x) a lo largo de x."""
    pairs = [(x, s) for x, s in zip(xs, secs) if s.occupied]
    if len(pairs) < 3:
        return Trajectory(n=len(pairs))
    keep = stable_profiles([p[1] for p in pairs]) & np.array([not p[1].truncated for p in pairs])
    x = np.array([p[0] for p in pairs])[keep]
    yc = np.array([p[1].yc for p in pairs])[keep]
    if yc.size < 3:
        return Trajectory(n=int(yc.size))
    med = float(np.median(yc))
    tr = Trajectory(center_offset_m=med, max_lateral_dev_m=float(np.max(np.abs(yc - med))), n=int(yc.size))
    a, b = np.polyfit(x, yc, 1)
    res = yc - (a * x + b)
    mad = 1.4826 * np.median(np.abs(res - np.median(res)))
    inl = np.abs(res) <= max(3 * mad, 1e-3)
    if inl.sum() >= 3:
        a, b = np.polyfit(x[inl], yc[inl], 1)
        res = yc[inl] - (a * x[inl] + b)
    tr.yaw_rad = float(math.atan(a))
    tr.path_rms_m = float(np.sqrt(np.mean(res ** 2)))
    return tr


def sync_offset(secs_time_order: list[Section], hist: VelocityHistory, x_plane: float,
                direction: int) -> tuple[float, float, float]:
    """Δt_sync = t_borde(LiDAR de velocidad) − t_borde(LiDAR de volumen).

    El LiDAR de volumen acota el cruce del frente entre el último perfil vacío y el primero
    ocupado (y la cola entre el último ocupado y el primero vacío); se toma el punto medio.
    Devuelve (Δt medio, t_frente_vel, t_cola_vel); NaN donde no se pudo.
    """
    occ = [i for i, s in enumerate(secs_time_order) if s.occupied]
    if not occ:
        return math.nan, math.nan, math.nan
    t = [s.stamp for s in secs_time_order]
    i_f, i_l = occ[0], occ[-1]
    t0 = t[0] - 2.0
    t1 = t[-1] + 2.0
    tf = hist.edge_crossing(x_plane, direction, "front", t0, t1)
    tt = hist.edge_crossing(x_plane, direction, "tail", t0, t1)
    d = []
    if math.isfinite(tf) and i_f > 0:
        d.append(tf - 0.5 * (t[i_f - 1] + t[i_f]))
    if math.isfinite(tt) and i_l + 1 < len(t):
        d.append(tt - 0.5 * (t[i_l] + t[i_l + 1]))
    return (float(np.mean(d)) if d else math.nan), tf, tt


def incoherent_fraction(secs: list[Section], hist: VelocityHistory, x_plane: float) -> float:
    """Fracción de perfiles ocupados en que el LiDAR de velocidad NO ve objeto en el plano."""
    n = bad = 0
    for s in secs:
        if not s.occupied:
            continue
        ext = hist.extent_near(s.stamp, _COHERENCE_MAX_DT_S)
        if ext is None:
            continue
        n += 1
        x0, x1 = ext
        if not (math.isfinite(x0) and x0 - _COHERENCE_MARGIN_M <= x_plane <= x1 + _COHERENCE_MARGIN_M):
            bad += 1
    return bad / n if n else math.nan


def classify(flags: list[str], discarded: bool) -> int:
    if discarded:
        return RESULT_DISCARDED
    if any(f in INVALIDATING for f in flags):
        return RESULT_INVALID
    return RESULT_WARNINGS if flags else RESULT_VALID


def measurement_flags(p: VolumeParams, res: IntegrationResult, secs: list[Section], traj: Trajectory,
                      n_missing_x: int, v_interp_fraction: float, extrapolated: bool,
                      dt_sync: float, incoherent: float, has_pass: bool) -> tuple[list[str], bool]:
    """Banderas de la medición y si el objeto se descarta por corto (no reconocible)."""
    f: list[str] = []
    occ = [s for s in secs if s.occupied]
    if occ and sum(s.truncated for s in occ) / len(occ) > p.vol_max_truncated_fraction:
        f.append("TRUNCADO_LATERAL")
    if n_missing_x:
        f.append("VELOCIDAD_INSUFICIENTE")
    if math.isfinite(res.max_dx_m) and res.max_dx_m > p.vol_max_dx_m:
        f.append("HUECO_LONGITUDINAL")
    span = sum(s.span_cells for s in occ)
    shadow_frac = sum(s.shadow for s in occ) / len(occ) if occ else 0.0
    if shadow_frac > p.vol_max_shadow_fraction or (span and sum(s.nan_cells for s in occ) / span > p.vol_max_nan_fraction):
        f.append("SOMBRAS")
    length = res.length_m
    # No reconocible: más corto que el mínimo, o (sin largo medible) pocos perfiles y ningún
    # paso del LiDAR de velocidad asociado (p. ej. una persona que solo cruzó el plano).
    discarded = (math.isfinite(length) and length < p.vol_min_length_m) or (
        not math.isfinite(length) and res.n_occupied < p.vol_min_profiles and not has_pass)
    if math.isfinite(length) and length > p.vol_max_length_m:
        f.append("LARGO_FUERA_DE_RANGO")
    if res.n_occupied < p.vol_min_profiles:
        f.append("POCOS_PERFILES")
    if math.isfinite(traj.center_offset_m) and abs(traj.center_offset_m) > p.vol_max_center_offset_m:
        f.append("DESCENTRADO")
    if math.isfinite(traj.max_lateral_dev_m) and traj.max_lateral_dev_m > p.vol_max_lateral_dev_m:
        f.append("CONDUCCION_NO_PARALELA")
    if math.isfinite(dt_sync):
        if abs(dt_sync) > p.vol_sync_tol_s:
            f.append("DESINCRONIZADO")
    else:
        f.append("SINCRONIA_NO_VERIFICADA")
    if math.isfinite(incoherent) and incoherent > p.vol_max_incoherent_fraction:
        f.append("INCOHERENCIA_LIDAR")
    if v_interp_fraction > p.vol_max_v_interp_fraction:
        f.append("VELOCIDAD_INTERPOLADA")
    if extrapolated:
        f.append("VELOCIDAD_EXTRAPOLADA")
    if occ and sum(s.floor_drift for s in occ) / len(occ) > 0.5:
        f.append("DERIVA_PISO")
    if any(s.high for s in occ):
        f.append("CARGA_ALTA")
    if p.vol_edge_from_velocity and res.edge_method != "velocidad":
        f.append("BORDES_SIN_VELOCIDAD")
    if not has_pass:
        f.append("SIN_PASO")
    return f, discarded
