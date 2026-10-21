"""Máquina de estados de paso: convierte la ocupación por escaneo en pasos de camión.

Diseño en ``docs/maquina_estados_paso.md`` (transiciones T1–T12 y banderas de calidad).
Python puro: la usa el nodo ROS y el visor de pruebas por igual.

    tracker = PassTracker(params)
    for scan in escaneos:
        est = estimator.process(scan)
        eventos = tracker.update_from_estimate(est, ahora=reloj())
    eventos = tracker.tick(reloj())       # en un timer: detecta SIN_DATOS sin escaneos

Sentido (docs/maquina_estados_paso.md, «Sentido de circulación»): con ``pass_both_directions``
el vehículo puede cruzar hacia +x (entra por roi_x_min) o hacia −x (entra por roi_x_max). El
sentido del paso se fija al confirmar la entrada (T5) por el borde por donde apareció; si es
ambiguo (en medio de la ROI o tocando los dos bordes) se decide con las primeras velocidades OK
y se marca ENTRADA_FUERA_DE_BORDE. Con el sentido fijo, la rapidez de avance de cada escaneo es
``v_avance = sentido · v``: rango y reversa se evalúan sobre ella, y si al cerrar el signo de la
mediana de v no coincide con el sentido se marca SENTIDO_INCONSISTENTE. Todo está en el marco
mundo (+x); la referencia de signo del usuario (``invert_velocity_sign``) se aplica a la salida
con ``PassEvent.with_sign``.

Relojes: ``t`` es el stamp del escaneo (inicio/fin de paso, timeout del paso); ``ahora`` es el
reloj de quien llama (recepción) y solo se usa para ``scan_timeout_s``. Por defecto ``ahora = t``,
lo que sirve para reproducir rosbags con su propio tiempo.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field, replace
from enum import IntEnum, IntFlag

import numpy as np

from .estimator import Status, VelocityEstimate, VelocityParams


class PassState(IntEnum):
    ARRANQUE = 0     # inicio o retorno de falla: puede haber un camión a medio pasar
    VACIO = 1        # ROI libre, esperando vehículo
    ENTRANDO = 2     # ocupación detectada, sin confirmar
    EN_PASO = 3      # paso confirmado (pass_id asignado)
    SALIENDO = 4     # ROI aparentemente libre; confirmando salida o hueco momentáneo
    OBSTRUIDO = 5    # ocupación demasiado larga
    SIN_DATOS = 6    # no llegan escaneos


READY_STATES = frozenset({PassState.VACIO, PassState.ENTRANDO, PassState.EN_PASO, PassState.SALIENDO})
OPEN_STATES = frozenset({PassState.EN_PASO, PassState.SALIENDO})   # con pass_id asignado
# Hay (o puede haber) un vehículo en curso: el estimador conserva su historial entre escaneos
ACTIVE_STATES = frozenset({PassState.ENTRANDO, PassState.EN_PASO, PassState.SALIENDO})

N_V_SENTIDO = 3   # velocidades OK con que se decide el sentido de una entrada ambigua


class PassFlag(IntFlag):
    NINGUNA = 0
    ENTRADA_FUERA_DE_BORDE = 1
    SALIDA_FUERA_DE_BORDE = 2
    VELOCIDAD_FUERA_DE_RANGO = 4
    REVERSA = 8
    POCOS_VALIDOS = 16
    TIMEOUT = 32
    SENSOR_PERDIDO = 64
    SENTIDO_INCONSISTENTE = 128     # el signo de la mediana de v no coincide con el sentido del paso

    def nombres(self) -> list[str]:
        return [f.name for f in PassFlag if f and f in self]


@dataclass
class PassEvent:
    """Evento INICIO o FIN de un paso. En INICIO solo vienen pass_id, t_start y ext_start."""

    pass_id: int
    event: str                                  # "INICIO" | "FIN"
    t_start: float
    t_end: float = float("nan")
    duration_s: float = float("nan")
    flags: PassFlag = PassFlag.NINGUNA
    n_gaps: int = 0
    n_scans: int = 0                            # escaneos ocupados dentro del paso
    n_valid: int = 0                            # de ellos, con velocidad OK
    v_median_mps: float = float("nan")          # velocidad con signo (marco mundo, +x)
    v_min_mps: float = float("nan")
    v_max_mps: float = float("nan")
    advance_median_mps: float = float("nan")    # rapidez de avance v_avance = direction · v
    advance_min_mps: float = float("nan")       # (positiva en los dos sentidos; reversa < 0)
    advance_max_mps: float = float("nan")
    ext_start: tuple[float, float] | None = None
    ext_end: tuple[float, float] | None = None
    direction: int = 0                          # +1 hacia +x, −1 hacia −x, 0 sin determinar

    @property
    def valid(self) -> bool:
        """Paso sin banderas = candidato a medición válida."""
        return self.event == "FIN" and self.flags == PassFlag.NINGUNA

    def with_sign(self, sign: int) -> PassEvent:
        """Copia con velocidades y sentido en la referencia del usuario (sign = ±1).

        Con sign = −1 el mínimo y el máximo se intercambian (−max es el nuevo mínimo).
        La rapidez de avance y las extensiones no cambian.
        """
        if sign >= 0:
            return self
        return replace(self, v_median_mps=-self.v_median_mps, v_min_mps=-self.v_max_mps,
                       v_max_mps=-self.v_min_mps, direction=-self.direction)


@dataclass
class _Acumulador:
    t_primer_ocup: float
    ext_inicial: tuple[float, float] | None
    n_scans: int = 0
    n_valid: int = 0
    n_gaps: int = 0
    v: list[float] = field(default_factory=list)
    sentido: int = 0                 # fijado en T5 por el borde; 0 = ambiguo hasta tener velocidades
    ambiguo: bool = False


class PassTracker:
    def __init__(self, params: VelocityParams | None = None):
        self.p = params or VelocityParams()
        self.pass_id = 0
        self.n_false_entries = 0      # T4: ocupaciones que no llegaron a confirmarse
        self.n_obstructions = 0       # entradas a OBSTRUIDO
        self.reset()

    # ------------------------------------------------------------------ API
    def reset(self) -> None:
        """Vuelve a ARRANQUE sin tocar pass_id (los ids no se repiten en la sesión)."""
        self.state = PassState.ARRANQUE
        self._reset_counters()
        self._t_rx: float | None = None

    @property
    def ready(self) -> bool:
        return self.state in READY_STATES

    @property
    def open_pass(self) -> bool:
        return self.state in OPEN_STATES

    @property
    def direction(self) -> int:
        """Sentido del paso abierto en el marco mundo (+1/−1), 0 si no hay o aún no se sabe."""
        return self._acc.sentido if self.open_pass and self._acc is not None else 0

    def update_from_estimate(self, est: VelocityEstimate, ahora: float | None = None) -> list[PassEvent]:
        ocupado = est.status != Status.NO_OBJECT
        return self.update(est.stamp, ocupado, est.object_extent, est, ahora)

    def update(self, t: float, ocupado: bool, ext: tuple[float, float] | None,
               est: VelocityEstimate | None = None, ahora: float | None = None) -> list[PassEvent]:
        p = self.p
        self._t_rx = t if ahora is None else ahora
        eventos: list[PassEvent] = []
        if self.state == PassState.SIN_DATOS:                          # T12
            self.state = PassState.ARRANQUE
            self._reset_counters()

        if ocupado:
            if self.n_ocup == 0:
                self._t_inicio_racha = t
            self.n_ocup += 1
            self.n_vacio = 0
            self._t_ultimo_ocup = t
            self._ext_ultimo = ext
        else:
            self.n_vacio += 1
            self.n_ocup = 0

        s = self.state
        if s in (PassState.ENTRANDO, PassState.EN_PASO, PassState.SALIENDO):
            self._acumular(ocupado, est)

        # Prioridad: timeout antes que ocupación (SIN_DATOS se resuelve en tick)
        if s == PassState.EN_PASO and t - self._acc.t_primer_ocup > p.pass_max_s:   # T7
            eventos.append(self._fin(t, PassFlag.TIMEOUT))
            self._a_obstruido()
        elif s == PassState.ARRANQUE:
            if self.n_vacio >= p.pass_empty_scans:                                   # T1
                self.state = PassState.VACIO
            elif ocupado and t - self._t_inicio_racha > p.pass_max_s:                 # T2
                self._a_obstruido()
        elif s == PassState.VACIO:
            if ocupado:                                                               # T3
                self.state = PassState.ENTRANDO
                self._acc = _Acumulador(t_primer_ocup=t, ext_inicial=ext)
                self._acumular(ocupado, est)
                if self.n_ocup >= p.pass_enter_scans:   # pass_enter_scans = 1
                    eventos.append(self._inicio())
        elif s == PassState.ENTRANDO:
            if not ocupado:                                                           # T4
                self.state = PassState.VACIO
                self.n_false_entries += 1
            elif self.n_ocup >= p.pass_enter_scans:                                   # T5
                eventos.append(self._inicio())
        elif s == PassState.EN_PASO:
            if not ocupado:                                                           # T6
                self.state = PassState.SALIENDO
        elif s == PassState.SALIENDO:
            if ocupado:                                                               # T8
                self.state = PassState.EN_PASO
                self._acc.n_gaps += 1
            elif self.n_vacio >= p.pass_exit_scans:                                   # T9
                eventos.append(self._fin(self._t_ultimo_ocup))
                self.state = PassState.VACIO
        elif s == PassState.OBSTRUIDO:
            if self.n_vacio >= p.pass_empty_scans:                                    # T10
                self.state = PassState.VACIO
        return eventos

    def tick(self, ahora: float) -> list[PassEvent]:
        """Detecta SIN_DATOS (T11). Llamar periódicamente con el mismo reloj que ``ahora`` de update."""
        if self.state == PassState.SIN_DATOS:
            return []
        if self._t_rx is None:          # nunca llegó un escaneo: el plazo corre desde el primer tick
            self._t_rx = ahora
            return []
        if ahora - self._t_rx <= self.p.scan_timeout_s:
            return []
        eventos = [self._fin(self._t_ultimo_ocup, PassFlag.SENSOR_PERDIDO)] if self.open_pass else []
        self.state = PassState.SIN_DATOS
        return eventos

    # ------------------------------------------------------------------ internos
    def _reset_counters(self) -> None:
        self.n_ocup = 0
        self.n_vacio = 0
        self._t_inicio_racha = math.nan
        self._t_ultimo_ocup = math.nan
        self._ext_ultimo: tuple[float, float] | None = None
        self._acc: _Acumulador | None = None

    def _a_obstruido(self) -> None:
        self.state = PassState.OBSTRUIDO
        self.n_obstructions += 1

    def _acumular(self, ocupado: bool, est: VelocityEstimate | None) -> None:
        if not ocupado or self._acc is None:
            return
        a = self._acc
        a.n_scans += 1
        if est is not None and est.valid:
            a.n_valid += 1
            a.v.append(est.velocity_mps)
            # Entrada ambigua: el sentido sale de las primeras velocidades OK
            if a.sentido == 0 and a.ambiguo and len(a.v) >= N_V_SENTIDO:
                a.sentido = _signo(float(np.median(a.v[:N_V_SENTIDO])))

    def _sentido_por_borde(self, ext: tuple[float, float] | None) -> int:
        """+1 si toca solo la franja de x mín, −1 si solo la de x máx, 0 si ninguna o ambas."""
        p = self.p
        if ext is None:
            return 0
        en_min = ext[0] <= p.roi_x_min_m + p.pass_edge_margin_m
        en_max = ext[1] >= p.roi_x_max_m - p.pass_edge_margin_m
        if not p.pass_both_directions:
            return 1 if en_min else 0
        return 1 if en_min and not en_max else (-1 if en_max and not en_min else 0)

    def _inicio(self) -> PassEvent:
        """T5: se fija el sentido por el borde de entrada (0 si es ambiguo, se decide después)."""
        self.pass_id += 1
        self.state = PassState.EN_PASO
        a = self._acc
        a.sentido = self._sentido_por_borde(a.ext_inicial)
        a.ambiguo = a.sentido == 0
        if a.sentido == 0 and not self.p.pass_both_directions:
            a.sentido = 1                     # un solo sentido: +x, con ENTRADA_FUERA_DE_BORDE
        elif a.ambiguo and len(a.v) >= N_V_SENTIDO:
            a.sentido = _signo(float(np.median(a.v[:N_V_SENTIDO])))
        return PassEvent(self.pass_id, "INICIO", a.t_primer_ocup, ext_start=a.ext_inicial, direction=a.sentido)

    def _fin(self, t_end: float, extra: PassFlag = PassFlag.NINGUNA) -> PassEvent:
        p, a = self.p, self._acc
        ev = PassEvent(self.pass_id, "FIN", a.t_primer_ocup, t_end=t_end, duration_s=t_end - a.t_primer_ocup,
                       n_gaps=a.n_gaps, n_scans=a.n_scans, n_valid=a.n_valid,
                       ext_start=a.ext_inicial, ext_end=self._ext_ultimo)
        flags = extra
        d = a.sentido
        if d == 0 and a.v:                   # ambiguo y con menos de N_V_SENTIDO velocidades
            d = _signo(float(np.median(a.v)))
        ev.direction = d
        if a.ambiguo or a.ext_inicial is None:
            flags |= PassFlag.ENTRADA_FUERA_DE_BORDE
        if a.v:
            v = np.asarray(a.v)
            ev.v_median_mps, ev.v_min_mps, ev.v_max_mps = float(np.median(v)), float(v.min()), float(v.max())
            if d != 0:
                avance = d * v
                ev.advance_median_mps = float(np.median(avance))
                ev.advance_min_mps, ev.advance_max_mps = float(avance.min()), float(avance.max())
                if not (p.pass_v_min_mps - p.pass_v_tolerance_mps <= ev.advance_median_mps
                        <= p.pass_v_max_mps + p.pass_v_tolerance_mps):
                    flags |= PassFlag.VELOCIDAD_FUERA_DE_RANGO
                if int(np.sum(avance < p.pass_reverse_v_mps)) >= p.pass_reverse_scans:
                    flags |= PassFlag.REVERSA
                if _signo(ev.v_median_mps) not in (0, d):
                    flags |= PassFlag.SENTIDO_INCONSISTENTE
        # Sale por el borde opuesto al de entrada; sin sentido no se puede validar la salida
        e1 = self._ext_ultimo
        if e1 is None or d == 0 or (e1[0] < p.roi_x_max_m - p.pass_edge_margin_m if d > 0
                                    else e1[1] > p.roi_x_min_m + p.pass_edge_margin_m):
            flags |= PassFlag.SALIDA_FUERA_DE_BORDE
        if a.n_scans == 0 or a.n_valid / a.n_scans < p.pass_min_valid_fraction:
            flags |= PassFlag.POCOS_VALIDOS
        ev.flags = flags
        self._acc = None
        return ev


def _signo(v: float) -> int:
    return 1 if v > 0 else (-1 if v < 0 else 0)
