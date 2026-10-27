"""Corre un escenario completo en simulación: LiDAR de velocidad (estimador + PassTracker) y
reconstructor de volumen, con los escaneos entregados en orden temporal, como en el pórtico.

``true_velocity=True`` reemplaza la velocidad estimada por la verdadera del simulador (las
extensiones del objeto siguen saliendo del estimador): separa el error de área del error de
velocidad.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field

from falconscan_velocity.core import PassEvent, VelocityParams, VelocityPipeline

from ..core import CaptureStarted, Measurement, VelocitySample, VolumeParams, VolumeReconstructor
from .portico_sim import PorticoSim


@dataclass
class ScenarioResult:
    measurements: list[Measurement] = field(default_factory=list)
    pass_events: list[PassEvent] = field(default_factory=list)
    captures: int = 0
    truth_m3: float = math.nan

    @property
    def m(self) -> Measurement | None:
        return self.measurements[0] if self.measurements else None

    @property
    def error_pct(self) -> float:
        m = self.m
        return 100.0 * (m.volume_m3 - self.truth_m3) / self.truth_m3 if m else math.nan


def run_scenario(sim: PorticoSim, duration_s: float, vparams: VelocityParams | None = None,
                 volparams: VolumeParams | None = None, true_velocity: bool = False,
                 tail_s: float = 4.0) -> ScenarioResult:
    pipe = VelocityPipeline(vparams or VelocityParams())
    rec = VolumeReconstructor(volparams or VolumeParams())
    res = ScenarioResult(truth_m3=sim.obj.volume_truth())
    period = 1.0 / sim.cfg.scan_hz

    def collect(outs):
        for o in outs:
            if isinstance(o, Measurement):
                res.measurements.append(o)
            elif isinstance(o, CaptureStarted):
                res.captures += 1

    t_now = 0.0
    for t_now, kind, scan in sim.run(duration_s):
        collect(rec.tick(t_now))
        if kind == "vel":
            est, events = pipe.process(scan)
            for ev in events:
                res.pass_events.append(ev)
                collect(rec.on_pass_event(ev))
            x0, x1 = est.object_extent if est.object_extent else (math.nan, math.nan)
            if true_velocity:
                s = VelocitySample(scan.stamp, scan.stamp, sim.motion.velocity_x(scan.stamp), True, x0, x1)
            else:
                s = VelocitySample(scan.stamp, est.mid_stamp, est.velocity_mps, est.valid, x0, x1)
            collect(rec.on_velocity(s, now=t_now))
        else:
            collect(rec.on_scan(scan, now=t_now))
    # deja correr el reloj para que cierren las esperas (sin escaneos nuevos: SIN_DATOS al final)
    t = t_now
    while t < t_now + tail_s and rec.cap is not None:
        t += period
        collect(rec.tick(t))
    return res
