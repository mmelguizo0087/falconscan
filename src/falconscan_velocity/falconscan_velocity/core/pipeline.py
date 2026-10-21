"""Estimador + máquina de estados de paso con la política de reinicio decidida (2026-10-21).

Antes, ``VelocityEstimator`` borraba su historial en cada escaneo vacío: un hueco de 1–2
escaneos dentro de un paso (SALIENDO) obligaba a esperar ``baseline_scans`` escaneos de
WARMING_UP otra vez. Ahora, mientras hay un vehículo en curso (ENTRANDO, EN_PASO, SALIENDO), los
escaneos vacíos se saltan sin borrar el historial, y el estimador se reinicia al salir de esos
estados (FIN, falsa entrada o pérdida del sensor). Fuera de un paso se comporta como antes.

Lo usan ``velocity_node``, el visor y las pruebas, para que todos apliquen la misma política.
"""
from __future__ import annotations

from .estimator import VelocityEstimate, VelocityEstimator, VelocityParams
from .pass_tracker import ACTIVE_STATES, PassEvent, PassTracker
from .scan import Scan


class VelocityPipeline:
    def __init__(self, params: VelocityParams | None = None, reset_on_empty: bool = False):
        """``reset_on_empty=True`` reproduce la política anterior (solo para medir la diferencia)."""
        self.p = params or VelocityParams()
        self.estimator = VelocityEstimator(self.p)
        self.tracker = PassTracker(self.p)
        self.reset_on_empty = reset_on_empty

    def process(self, scan: Scan, ahora: float | None = None) -> tuple[VelocityEstimate, list[PassEvent]]:
        activo = self.tracker.state in ACTIVE_STATES
        est = self.estimator.process(scan, keep_history=activo and not self.reset_on_empty)
        eventos = self.tracker.update_from_estimate(est, ahora)
        if activo and self.tracker.state not in ACTIVE_STATES:
            self.estimator.reset()
        return est, eventos

    def tick(self, ahora: float) -> list[PassEvent]:
        activo = self.tracker.state in ACTIVE_STATES
        eventos = self.tracker.tick(ahora)
        if activo and self.tracker.state not in ACTIVE_STATES:
            self.estimator.reset()
        return eventos
