"""Máquina de operación: convierte la detección por escaneo en PASOS de camión.

Implementa la tabla T1–T12 de ``docs/maquina_estados_paso.md`` §2 (sin la calibración:
T13/T14 quedan para el supervisor). Es bidireccional: cada paso tiene un ``sentido`` ±1
que se decide por el borde de entrada, y la rapidez de avance es ``sentido · v``.

``VelocityPipeline`` junta estimador + tracker tal como los usa el nodo: el sentido del
paso se le pasa al estimador (búsqueda según el sentido) y el historial del estimador se
reinicia en FIN.

Todo está en el marco mundo (+x). La referencia de signo del usuario (``invert_velocity_sign``)
es solo para mostrar: ``PassEvent.with_sign`` (visor, HMI); los mensajes ROS no la aplican.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field, replace
from enum import IntEnum

import numpy as np

from .estimator import VelocityEstimate, VelocityEstimator, VelocityParams
from .scan import Scan


class PassState(IntEnum):
    ARRANQUE = 0
    VACIO = 1
    ENTRANDO = 2
    EN_PASO = 3
    SALIENDO = 4
    OBSTRUIDO = 5
    SIN_DATOS = 6


READY = {PassState.VACIO, PassState.ENTRANDO, PassState.EN_PASO, PassState.SALIENDO}
OPEN = {PassState.EN_PASO, PassState.SALIENDO}       # con pass_id asignado


@dataclass
class PassEvent:
    kind: str                       # "INICIO" | "FIN"
    pass_id: int
    direction: int                  # +1 / -1; 0 si aún no se conoce
    t_start: float
    t_end: float = float("nan")
    flags: list[str] = field(default_factory=list)
    n_gaps: int = 0
    n_scans: int = 0
    n_valid: int = 0
    v_median_mps: float = float("nan")   # rapidez de avance (sentido · v), positiva en ambos sentidos
    v_min_mps: float = float("nan")
    v_max_mps: float = float("nan")
    ext_start: tuple[float, float] | None = None   # extensión del objeto al aparecer [m, marco x]
    ext_end: tuple[float, float] | None = None     # y al desaparecer (solo en FIN)

    @property
    def duration_s(self) -> float:
        return self.t_end - self.t_start

    @property
    def valid(self) -> bool:
        """FIN sin banderas = candidato a medición válida."""
        return self.kind == "FIN" and not self.flags

    def with_sign(self, sign: int) -> PassEvent:
        """Copia con el sentido en la referencia del usuario (sign = ±1), SOLO para mostrar.

        Las velocidades ya son rapidez de avance (positivas en los dos sentidos) y no cambian.
        """
        return self if sign >= 0 else replace(self, direction=-self.direction)


@dataclass
class _Pass:
    pass_id: int
    t_start: float
    ext_initial: tuple[float, float] | None
    direction: int = 0
    ambiguous_entry: bool = False
    ext_final: tuple[float, float] | None = None
    n_scans: int = 0
    v_ok: list[float] = field(default_factory=list)   # velocidades OK con signo (marco x)
    n_gaps: int = 0


class PassTracker:
    def __init__(self, params: VelocityParams | None = None):
        self.p = params or VelocityParams()
        self.state = PassState.ARRANQUE
        self.pass_id = 0
        self._cur: _Pass | None = None
        self._n_ocup = self._n_vacio = 0
        self._t_first_ocup = self._t_last_ocup = math.nan
        self._ext_first: tuple[float, float] | None = None
        self._ext_last: tuple[float, float] | None = None
        self._t_last_scan = -math.inf
        self.false_entries = 0
        self._entry_v: list[float] = []   # velocidades OK de ENTRANDO: pasan al paso en T5
        self._entry_n = 0

    # ---------------------------------------------------------------- API
    @property
    def ready(self) -> bool:
        return self.state in READY

    @property
    def direction(self) -> int:
        """Sentido del paso abierto (0 si no hay paso o aún no se conoce)."""
        return self._cur.direction if self._cur else 0

    @property
    def open_pass(self) -> bool:
        return self.state in OPEN

    @property
    def current_pass_id(self) -> int:
        return self._cur.pass_id if self._cur else 0

    def update(self, t: float, est: VelocityEstimate, now: float | None = None) -> list[PassEvent]:
        """Un escaneo. ``est`` es la salida de ``VelocityEstimator.process``.

        ``now`` = instante de llegada en el reloj del nodo (el mismo de ``tick``). Si se omite se
        usa el stamp ``t``; mezclar los dos relojes hace que un driver con latencia (stamps
        atrasados respecto al reloj del nodo) se vea como SIN_DATOS en cada tick.
        """
        p = self.p
        events: list[PassEvent] = []
        self._t_last_scan = t if now is None else now
        if self.state == PassState.SIN_DATOS:                         # T12
            self.state = PassState.ARRANQUE
            self._n_ocup = self._n_vacio = 0

        ext = est.object_extent
        ocupado = ext is not None
        if ocupado:
            if self._n_ocup == 0:
                self._t_first_ocup, self._ext_first = t, ext
            self._n_ocup += 1
            self._n_vacio = 0
            self._t_last_ocup, self._ext_last = t, ext
        else:
            self._n_vacio += 1
            self._n_ocup = 0

        # Solo cuentan los escaneos OCUPADOS (los vacíos de SALIENDO no diluyen la fracción de
        # válidos), incluidos los de ENTRANDO: el paso empieza en el primer escaneo ocupado.
        c = self._cur
        if self.state == PassState.VACIO and ocupado:
            self._entry_v, self._entry_n = [], 0
        if ocupado and self.state in (PassState.VACIO, PassState.ENTRANDO):
            self._entry_n += 1
            if est.valid:
                self._entry_v.append(est.velocity_mps)
        if ocupado and c is not None and self.state in (PassState.EN_PASO, PassState.SALIENDO):
            c.n_scans += 1
            if est.valid:
                c.v_ok.append(est.velocity_mps)
            if c.direction == 0 and len(c.v_ok) >= 3:
                c.direction = int(np.sign(np.median(c.v_ok))) or 1

        s = self.state
        if s == PassState.ARRANQUE:
            if self._n_vacio >= p.pass_empty_scans:                   # T1
                self.state = PassState.VACIO
            elif ocupado and t - self._t_first_ocup > p.pass_max_s:    # T2
                self.state = PassState.OBSTRUIDO
        elif s == PassState.VACIO:
            if ocupado:                                                # T3
                self.state = PassState.ENTRANDO
        elif s == PassState.ENTRANDO:
            if not ocupado:                                            # T4
                self.state = PassState.VACIO
                self.false_entries += 1
            elif self._n_ocup >= p.pass_enter_scans:                   # T5
                events.append(self._start())
        elif s == PassState.EN_PASO:
            if t - c.t_start > p.pass_max_s:                           # T7
                events.append(self._finish(t, ["TIMEOUT"]))
                self.state = PassState.OBSTRUIDO
            elif not ocupado:                                          # T6
                self.state = PassState.SALIENDO
        elif s == PassState.SALIENDO:
            if ocupado:                                                # T8
                c.n_gaps += 1
                self.state = PassState.EN_PASO
            elif self._n_vacio >= p.pass_exit_scans:                   # T9
                events.append(self._finish(self._t_last_ocup, []))
                self.state = PassState.VACIO
            elif t - c.t_start > p.pass_max_s:                         # T7 (también en SALIENDO)
                events.append(self._finish(t, ["TIMEOUT"]))
                self.state = PassState.OBSTRUIDO
        elif s == PassState.OBSTRUIDO:
            if self._n_vacio >= p.pass_empty_scans:                    # T10
                self.state = PassState.VACIO
        return events

    def tick(self, now: float) -> list[PassEvent]:
        """Reloj del nodo: detecta SIN_DATOS aunque no lleguen escaneos (T11)."""
        if self._t_last_scan == -math.inf:   # nunca llegó un escaneo: el plazo corre desde el primer tick
            self._t_last_scan = now
            return []
        if self.state == PassState.SIN_DATOS or now - self._t_last_scan <= self.p.scan_timeout_s:
            return []
        events = []
        if self._cur is not None:
            events.append(self._finish(self._t_last_ocup, ["SENSOR_PERDIDO"]))
        self.state = PassState.SIN_DATOS
        return events

    # ---------------------------------------------------------- internos
    def _start(self) -> PassEvent:
        p = self.p
        self.pass_id += 1
        ext = self._ext_first
        c = _Pass(self.pass_id, self._t_first_ocup, ext, n_scans=self._entry_n, v_ok=list(self._entry_v))
        near_min = ext[0] <= p.roi_x_min_m + p.pass_edge_margin_m
        near_max = ext[1] >= p.roi_x_max_m - p.pass_edge_margin_m
        if near_min and not near_max:
            c.direction = 1
        elif near_max and not near_min:
            c.direction = -1
        else:
            c.ambiguous_entry = True        # se decide con las primeras velocidades OK
        self._cur = c
        self.state = PassState.EN_PASO
        return PassEvent("INICIO", c.pass_id, c.direction, c.t_start, ext_start=ext)

    def _finish(self, t_end: float, flags: list[str]) -> PassEvent:
        p, c = self.p, self._cur
        c.ext_final = self._ext_last
        flags = list(flags)
        if c.direction == 0:
            c.direction = int(np.sign(np.median(c.v_ok))) if c.v_ok else 0
        d = c.direction
        adv = d * np.asarray(c.v_ok) if c.v_ok and d else np.empty(0)
        if c.ambiguous_entry:
            flags.append("ENTRADA_FUERA_DE_BORDE")
        if c.ext_final is not None and d:
            if (d > 0 and c.ext_final[0] < p.roi_x_max_m - p.pass_edge_margin_m) or (
                    d < 0 and c.ext_final[1] > p.roi_x_min_m + p.pass_edge_margin_m):
                flags.append("SALIDA_FUERA_DE_BORDE")
        if adv.size:
            med = float(np.median(adv))
            if not (p.pass_v_min_mps - p.pass_v_tolerance_mps <= med <= p.pass_v_max_mps + p.pass_v_tolerance_mps):
                flags.append("VELOCIDAD_FUERA_DE_RANGO")
            if int((adv < p.pass_reverse_v_mps).sum()) >= p.pass_reverse_scans:
                flags.append("REVERSA")
            if not c.ambiguous_entry and np.sign(np.median(c.v_ok)) != d:
                flags.append("SENTIDO_INCONSISTENTE")
        if c.n_scans == 0 or len(c.v_ok) / c.n_scans < p.pass_min_valid_fraction:
            flags.append("POCOS_VALIDOS")
        ev = PassEvent("FIN", c.pass_id, d, c.t_start, t_end, flags, c.n_gaps, c.n_scans, len(c.v_ok),
                       ext_start=c.ext_initial, ext_end=c.ext_final)
        if adv.size:
            ev.v_median_mps, ev.v_min_mps, ev.v_max_mps = float(np.median(adv)), float(adv.min()), float(adv.max())
        self._cur = None
        return ev


class VelocityPipeline:
    """Estimador + tracker, como en el nodo: es lo que se evalúa en simulación."""

    def __init__(self, params: VelocityParams | None = None):
        self.p = params or VelocityParams()
        self.estimator = VelocityEstimator(self.p)
        self.tracker = PassTracker(self.p)

    def process(self, scan: Scan, now: float | None = None) -> tuple[VelocityEstimate, list[PassEvent]]:
        est = self.estimator.process(scan)
        events = self.tracker.update(scan.stamp, est, now)
        self._after(events)
        return est, events

    def tick(self, now: float) -> list[PassEvent]:
        events = self.tracker.tick(now)
        self._after(events)
        return events

    def _after(self, events: list[PassEvent]) -> None:
        if any(e.kind == "FIN" for e in events):
            self.estimator.reset()          # el historial se reinicia al cerrar el paso
        self.estimator.set_direction(self.tracker.direction)
