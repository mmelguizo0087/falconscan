"""Máquina de estados del reconstructor de volumen (docs/maquina_estados_volumen.md §2).

Entradas (el nodo solo convierte mensajes):
- ``on_scan(scan, now)``: perfil transversal del LiDAR de volumen.
- ``on_velocity(sample, now)``: /velocity_estimate.
- ``on_pass_event(ev)``: /pass_event (INICIO/FIN del PassTracker).
- ``tick(now)``: reloj del nodo (SIN_DATOS / SIN_VELOCIDAD / espera de cierre).

Salidas: lista de ``CaptureStarted``, ``CloudUpdate`` (nube provisional o final) y
``Measurement``. Los números Vn de los comentarios son las transiciones de la tabla del doc.

La calibración dinámica del sensor (supervisor CALIBRACION) aún no está implementada: el nodo
opera con la geometría del archivo de parámetros (``mode = "OPERACION"``).
"""
from __future__ import annotations

import math
from collections import deque
from dataclasses import dataclass, field
from enum import IntEnum

import numpy as np

from falconscan_velocity.core import Scan

from .checks import RESULT_TEXT, classify, incoherent_fraction, measurement_flags, sync_offset, trajectory
from .integration import Profile, integrate
from .odometry import VelocityHistory, VelocitySample
from .params import VolumeParams
from .reconstruction import Reconstruction, build
from .section import Section, SectionProcessor


class VolState(IntEnum):
    ARRANQUE = 0
    VACIO = 1
    DETECTANDO = 2
    CAPTURANDO = 3
    HUECO = 4
    CERRANDO = 5
    INTEGRANDO = 6
    OBSTRUIDO = 7
    SIN_DATOS = 8
    SIN_VELOCIDAD = 9


READY = {VolState.VACIO, VolState.DETECTANDO, VolState.CAPTURANDO, VolState.HUECO,
         VolState.CERRANDO, VolState.INTEGRANDO}
CAPTURING = {VolState.CAPTURANDO, VolState.HUECO, VolState.CERRANDO}


@dataclass
class PassInfo:
    pass_id: int
    direction: int
    flags: list[str] = field(default_factory=list)
    closed: bool = False


@dataclass
class CaptureStarted:
    measurement_id: int
    t: float
    pass_id: int


@dataclass
class CloudUpdate:
    measurement_id: int
    provisional: bool
    reconstruction: Reconstruction


@dataclass
class Measurement:
    measurement_id: int
    pass_id: int
    direction: int
    t_start: float
    t_end: float
    result: int
    result_text: str
    flags: list[str]
    volume_m3: float
    end_uncertainty_m3: float
    edge_method: str
    length_m: float
    max_width_m: float
    max_height_m: float
    mean_advance_mps: float
    yaw_rad: float
    center_offset_m: float
    max_lateral_dev_m: float
    path_rms_m: float
    n_profiles: int
    max_dx_m: float
    nan_fraction: float
    v_interp_fraction: float
    sync_offset_s: float
    incoherent_fraction: float
    n_gaps: int
    area_profile: list[tuple[float, float]] = field(default_factory=list)   # (x, A) en el marco del paso
    reconstruction: Reconstruction | None = None


@dataclass
class _Capture:
    measurement_id: int
    t_start: float
    sections: list[Section] = field(default_factory=list)        # en orden de llegada
    prov: list[Profile] = field(default_factory=list)            # x provisional (solo para la vista)
    pass_info: PassInfo | None = None
    n_gaps: int = 0
    t_last_occ: float = math.nan
    t_after: float = math.nan       # primer perfil vacío después del último ocupado
    t_close: float = math.nan       # reloj del nodo (llegada), no stamp
    partial_volume_m3: float = 0.0


class VolumeReconstructor:
    def __init__(self, params: VolumeParams | None = None):
        self.p = params or VolumeParams()
        self.sections = SectionProcessor(self.p)
        self.vel = VelocityHistory()
        self.state = VolState.ARRANQUE
        self.mode = "OPERACION"
        self.preroll: deque[Section] = deque(maxlen=max(self.p.vol_preroll_scans, self.p.vol_enter_scans + 1))
        self.cap: _Capture | None = None
        self.measurement_id = 0
        self.last_section: Section | None = None
        self.last_measurement: Measurement | None = None
        self._open_pass: PassInfo | None = None
        self._n_ocup = self._n_vacio = 0
        self._t_last_scan = -math.inf
        self._t_last_vel = -math.inf
        self.false_detections = 0

    # ---------------------------------------------------------------- estado
    @property
    def ready(self) -> bool:
        return self.state in READY

    @property
    def partial_volume_m3(self) -> float:
        return self.cap.partial_volume_m3 if self.cap else math.nan

    @property
    def current_pass_id(self) -> int:
        if self.cap and self.cap.pass_info:
            return self.cap.pass_info.pass_id
        return 0

    # ---------------------------------------------------------------- entradas
    def on_velocity(self, s: VelocitySample, now: float | None = None) -> list:
        self.vel.append(s)
        self._t_last_vel = s.stamp if now is None else now
        if self.state == VolState.SIN_VELOCIDAD:                                   # V18
            self._restart()
        return self._check_close(self._t_last_vel)

    def on_pass_event(self, ev) -> list:
        """``ev`` con kind ("INICIO"/"FIN"), pass_id, direction, flags (PassEvent del núcleo)."""
        if ev.kind == "INICIO":
            self._open_pass = PassInfo(ev.pass_id, ev.direction)
            if self.cap is not None and self.cap.pass_info is None:
                self.cap.pass_info = self._open_pass
        else:
            info = self._open_pass if (self._open_pass and self._open_pass.pass_id == ev.pass_id) else None
            if self.cap is not None and self.cap.pass_info and self.cap.pass_info.pass_id == ev.pass_id:
                info = self.cap.pass_info
            if info is not None:
                info.flags, info.closed = list(ev.flags), True
                info.direction = ev.direction or info.direction
            if self._open_pass and self._open_pass.pass_id == ev.pass_id:
                self._open_pass = None
        return []

    def on_scan(self, scan: Scan, now: float | None = None) -> list:
        p = self.p
        out: list = []
        self._t_last_scan = scan.stamp if now is None else now
        if self.state == VolState.SIN_DATOS:                                       # V15
            self._restart()
        sec = self.sections.process(scan)
        self.last_section = sec
        if sec.no_data:
            # escaneo perdido: no cuenta como vacío ni como ocupado; deja un hueco en x que
            # revisa HUECO_LONGITUDINAL
            return self._check_close(self._t_last_scan)
        occ, t = sec.occupied, sec.stamp
        if occ:
            self._n_ocup += 1
            self._n_vacio = 0
        else:
            self._n_vacio += 1
            self._n_ocup = 0
        if self.state == VolState.SIN_VELOCIDAD:
            return out

        s = self.state
        added = False
        if s in (VolState.ARRANQUE, VolState.OBSTRUIDO):
            if self._n_vacio >= p.vol_empty_scans:                                 # V1, V13
                self.state = VolState.VACIO
        elif s == VolState.VACIO:
            if occ:                                                                # V2
                self.state = VolState.DETECTANDO
        elif s == VolState.DETECTANDO:
            if not occ:                                                            # V3
                self.state = VolState.VACIO
                self.false_detections += 1
            elif self._n_ocup >= p.vol_enter_scans:                                # V4
                out.append(self._start(sec))
                added = True
        elif s == VolState.CAPTURANDO:
            if not occ:                                                            # V5
                self.state = VolState.HUECO
                self.cap.t_after = t
        elif s == VolState.HUECO:
            if occ:                                                                # V6
                self.state = VolState.CAPTURANDO
                self.cap.n_gaps += 1
            elif self._gap_closed(t):                                              # V7
                self.state = VolState.CERRANDO
                self.cap.t_close = self._t_last_scan     # reloj del nodo, el mismo de tick()
        elif s == VolState.CERRANDO and occ:
            if not self._gap_closed(t):                                            # V10: volvió (reversa)
                self.state = VolState.CAPTURANDO
                self.cap.n_gaps += 1
            else:
                # Ya pasó el espacio de salida: es OTRO objeto. Se cierra esta medición y el
                # perfil arranca la detección del siguiente.
                out += self._finish([])
                self.preroll.append(sec)
                self.state = VolState.DETECTANDO
                added = True

        if not added:
            if self.state in CAPTURING:
                self._add(sec)
            else:
                self.preroll.append(sec)

        if self.state in (VolState.CAPTURANDO, VolState.HUECO) and t - self.cap.t_start > p.vol_max_capture_s:
            out += self._finish(["TIMEOUT"])                           # V12
            self.state = VolState.OBSTRUIDO
        elif self.state in CAPTURING and p.vol_cloud_live:
            rec = build(self.cap.prov, self._prov_front(), self._direction_guess(), self.sections.centers)
            out.append(CloudUpdate(self.cap.measurement_id, True, rec))
        out += self._check_close(self._t_last_scan)
        return out

    def tick(self, now: float) -> list:
        p = self.p
        out: list = []
        if self.state != VolState.SIN_DATOS and now - self._t_last_scan > p.scan_timeout_s:   # V14
            out += self._abort("SENSOR_PERDIDO")
            self.state = VolState.SIN_DATOS
        elif (self.state not in (VolState.SIN_DATOS, VolState.SIN_VELOCIDAD)
              and now - self._t_last_vel > p.vel_timeout_s):                                    # V17
            out += self._abort("SIN_VELOCIDAD")
            self.state = VolState.SIN_VELOCIDAD
        out += self._check_close(now)
        return out

    # ---------------------------------------------------------------- internos
    def _restart(self) -> None:
        self.state = VolState.ARRANQUE
        self._n_ocup = self._n_vacio = 0
        self.preroll.clear()

    def _direction_guess(self) -> int:
        if self.cap and self.cap.pass_info and self.cap.pass_info.direction:
            return self.cap.pass_info.direction
        if self._open_pass and self._open_pass.direction:
            return self._open_pass.direction
        v = self.vel.last_valid_velocity()
        return int(np.sign(v)) if math.isfinite(v) and v != 0 else 1

    def _start(self, sec: Section) -> CaptureStarted:
        self.measurement_id += 1
        secs = list(self.preroll)
        if not secs or secs[-1] is not sec:
            secs.append(sec)
        self.cap = _Capture(self.measurement_id, secs[0].stamp, pass_info=self._open_pass)
        self.state = VolState.CAPTURANDO
        for s in secs:
            self._add(s)
        self.preroll.clear()
        return CaptureStarted(self.measurement_id, sec.stamp, self.current_pass_id)

    def _add(self, sec: Section) -> None:
        c = self.cap
        c.sections.append(sec)
        d = self._direction_guess()
        x = self.vel.advance(c.t_start, sec.stamp, d)
        x = x if math.isfinite(x) else 0.0
        v = self.vel.last_valid_velocity()
        v = d * v if math.isfinite(v) else 0.0
        if sec.occupied:
            c.t_last_occ = sec.stamp
            prev = next((pr for pr in reversed(c.prov) if pr.section.occupied), None)
            if prev is not None:
                c.partial_volume_m3 += 0.5 * (prev.section.area_m2 + sec.area_m2) * abs(x - prev.x)
        c.prov.append(Profile(x, sec, v))

    def _prov_front(self) -> float:
        occ = [pr.x for pr in self.cap.prov if pr.section.occupied]
        return occ[0] if occ else math.nan

    def _gap_closed(self, t: float) -> bool:
        c = self.cap
        if t - c.t_last_occ >= self.p.vol_exit_max_s:
            return True
        adv = self.vel.advance(c.t_last_occ, t, self._direction_guess())
        return math.isfinite(adv) and adv >= self.p.vol_exit_gap_m

    def _check_close(self, now: float) -> list:
        if self.state != VolState.CERRANDO or self.cap is None:
            return []
        c = self.cap
        covered = self.vel.covers(c.t_after + self.p.vol_velocity_margin_s)
        pass_ok = c.pass_info is None or c.pass_info.closed
        if covered and pass_ok:                                                    # V8
            return self._finish([])
        if now - c.t_close >= self.p.vol_close_wait_s:                             # V9
            return self._finish([])
        return []

    def _abort(self, cause: str) -> list:
        if self.state in (VolState.CAPTURANDO, VolState.HUECO):
            return self._finish([cause])
        if self.state == VolState.CERRANDO:
            return self._finish([])       # el objeto ya pasó: se integra con lo que hay
        return []

    def _finish(self, extra_flags: list[str]) -> list:
        """INTEGRANDO (V11): posiciones, integración, chequeos, reconstrucción final."""
        p, c = self.p, self.cap
        self.state = VolState.INTEGRANDO
        secs = c.sections
        times = np.array([s.stamp for s in secs])
        occ_idx = [i for i, s in enumerate(secs) if s.occupied]
        flags = list(extra_flags)

        direction = c.pass_info.direction if (c.pass_info and c.pass_info.direction) else 0
        if direction == 0:
            v_med = self.vel.median_velocity(times[0] - 1.0, times[-1] + 1.0)
            direction = int(np.sign(v_med)) if math.isfinite(v_med) and v_med != 0 else 0
        if direction == 0:
            flags.append("SIN_SENTIDO")
            direction = 1

        pos = self.vel.positions(times, direction, p.vol_v_max_gap_s, extrapolate=True)
        h = 0.05
        vel_ = (self.vel.positions(times + h, direction, np.inf, True).x
                - self.vel.positions(times - h, direction, np.inf, True).x) / (2 * h)
        lo = max(0, occ_idx[0] - 1) if occ_idx else 0
        hi = min(len(secs), occ_idx[-1] + 2) if occ_idx else 0
        rel = np.zeros(len(secs), bool)
        rel[lo:hi] = True
        occ_mask = np.array([s.occupied for s in secs])
        n_missing = int(np.sum(pos.missing & rel))
        v_interp = float(np.mean(pos.interpolated[occ_mask])) if occ_mask.any() else 0.0
        extrapolated = bool(np.any(pos.extrapolated & rel))
        profiles = [Profile(float(x), s, float(v)) for x, s, v, m in zip(pos.x, secs, vel_, pos.missing)
                    if not m and math.isfinite(x)]

        dt_sync, tf, tt = sync_offset(secs, self.vel, p.vol_plane_offset_m, direction)
        x_front = x_tail = math.nan
        if p.vol_edge_from_velocity:
            if math.isfinite(tf):
                x_front = float(self.vel.positions([tf], direction, np.inf, True).x[0])
            if math.isfinite(tt):
                x_tail = float(self.vel.positions([tt], direction, np.inf, True).x[0])
        res = integrate(profiles, p.vol_merge_dx_m, x_front, x_tail)
        kept_occ = [pr for pr in res.profiles if pr.section.occupied]
        traj = trajectory(np.array([pr.x for pr in kept_occ]), [pr.section for pr in kept_occ])
        incoh = incoherent_fraction(secs, self.vel, p.vol_plane_offset_m)
        more, discarded = measurement_flags(p, res, [s for s in secs if s.occupied], traj, n_missing, v_interp,
                                            extrapolated, dt_sync, incoh, c.pass_info is not None)
        flags += more
        if c.pass_info:
            flags += [f"PASO_{f}" for f in c.pass_info.flags]
        result = classify(flags, discarded)

        occ_secs = [s for s in secs if s.occupied]
        span = sum(s.span_cells for s in occ_secs)
        t_occ = [s.stamp for s in occ_secs]
        x_occ = [pr.x for pr in kept_occ]
        rec = build(res.profiles, res.x_front, direction, self.sections.centers)
        m = Measurement(
            measurement_id=c.measurement_id,
            pass_id=c.pass_info.pass_id if c.pass_info else 0,
            direction=direction,
            t_start=t_occ[0] if t_occ else c.t_start,
            t_end=t_occ[-1] if t_occ else c.t_start,
            result=result, result_text=RESULT_TEXT[result], flags=flags,
            volume_m3=res.volume_m3, end_uncertainty_m3=res.end_uncertainty_m3, edge_method=res.edge_method,
            length_m=res.length_m,
            max_width_m=max((s.width for s in occ_secs), default=math.nan),
            max_height_m=max((s.max_h for s in occ_secs), default=math.nan),
            mean_advance_mps=((x_occ[-1] - x_occ[0]) / (t_occ[-1] - t_occ[0])
                              if len(t_occ) > 1 and len(x_occ) > 1 and t_occ[-1] > t_occ[0] else math.nan),
            yaw_rad=traj.yaw_rad, center_offset_m=traj.center_offset_m,
            max_lateral_dev_m=traj.max_lateral_dev_m, path_rms_m=traj.path_rms_m,
            n_profiles=res.n_occupied, max_dx_m=res.max_dx_m,
            nan_fraction=sum(s.nan_cells for s in occ_secs) / span if span else math.nan,
            v_interp_fraction=v_interp, sync_offset_s=dt_sync, incoherent_fraction=incoh,
            n_gaps=c.n_gaps,
            area_profile=[(-(pr.x - res.x_front), pr.section.area_m2 if pr.section.occupied else 0.0)
                          for pr in res.profiles],
            reconstruction=rec,
        )
        self.last_measurement = m
        self.cap = None
        self.preroll.clear()
        self.state = VolState.VACIO
        return [m, CloudUpdate(m.measurement_id, False, rec)]
