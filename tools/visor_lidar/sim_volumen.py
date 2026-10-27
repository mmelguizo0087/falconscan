#!/usr/bin/env python3
"""Genera un paso del simulador del pórtico (los DOS LiDAR) y lo guarda para reproducirlo.

El simulador 3D marcha cada rayo (falconscan_volume/sim/portico_sim.py): en la RPi 5 cuesta
~1 s por escaneo, muy lejos de los 10 Hz. Por eso la pestaña «Volumen 3D» del visor no simula
en vivo: genera el paso una vez con este programa (en paralelo, un proceso por núcleo), lo
guarda en un .npz y después lo reproduce como un rosbag (1×, 2×, 5×, máx.).

    python3 tools/visor_lidar/sim_volumen.py --lista
    python3 tools/visor_lidar/sim_volumen.py --caso "10 km/h +x" --salida /tmp/paso.npz
    python3 tools/visor_lidar/sim_volumen.py --kmh 7.5 --sentido -1 --lateral 0.3 --salida /tmp/p.npz

Escenarios = los de tools/evaluate_volume_sim.py (misma matriz de regresión). El ruido de cada
tramo usa su propia semilla, así que el resultado no es idéntico bit a bit al de la matriz,
pero sí estadísticamente equivalente. Resultados SINTÉTICOS: no son evidencia de campo.
"""
from __future__ import annotations

import argparse
import json
import math
import os
import sys
from concurrent.futures import ProcessPoolExecutor
from dataclasses import replace
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path[:0] = [str(ROOT / "tools"), str(ROOT / "src" / "falconscan_volume"), str(ROOT / "src" / "falconscan_velocity")]

from falconscan_volume.sim import DumpTruck3D, Motion3D, PorticoSim, PorticoSimConfig  # noqa: E402

VERSION = 1                      # cambia si cambia el formato del .npz o el simulador
D0_PERSONALIZADO = -6.0          # el frente arranca 6 m antes del plano de volumen
PERSONALIZADO = "Personalizado"


def escenarios() -> list[dict]:
    """Casos de tools/evaluate_volume_sim.py: name, expect, motion, cfg, obj, vol."""
    from evaluate_volume_sim import cases
    return cases()


def nombres() -> list[str]:
    return [c["name"] for c in escenarios()]


def caso_personalizado(kmh: float, sentido: int, lateral_m: float, guinada_deg: float) -> dict:
    v = max(0.5, kmh / 3.6)
    return dict(name=PERSONALIZADO, expect="V", cfg=PorticoSimConfig(), obj=DumpTruck3D(), vol={},
                motion=Motion3D(direction=1 if sentido >= 0 else -1, v=v, d0=D0_PERSONALIZADO,
                                y_offset=lateral_m, yaw_rad=math.radians(guinada_deg)))


def _caso(spec: dict) -> dict:
    if spec["nombre"] == PERSONALIZADO:
        return caso_personalizado(spec["kmh"], spec["sentido"], spec["lateral"], spec["guinada"])
    return next(c for c in escenarios() if c["name"] == spec["nombre"])


def duracion_s(caso: dict) -> float:
    """Como tools/evaluate_volume_sim.py: el objeto entra, cruza y sale, con margen."""
    m, obj = caso["motion"], caso["obj"]
    if m.distance is not None:
        return 25.0
    return (abs(m.d0) + obj.length + 7.0) / max(0.5, abs(m.speed(5.0)))


def _tramo(args):
    """Escaneos k0..k1-1 (velocidad y volumen) de un caso; se reconstruye en el proceso hijo
    porque los movimientos usan lambdas (no se pueden enviar entre procesos)."""
    spec, k0, k1, semilla = args
    c = _caso(spec)
    sim = PorticoSim(replace(c["cfg"], seed=semilla), c["obj"], c["motion"])
    per = 1.0 / sim.cfg.scan_hz
    vel, vol = [], []
    for k in range(k0, k1):
        tv = k * per
        vel.append(sim.velocity_scan(tv))
        vol.append(sim.volume_scan(tv + sim.cfg.vol_phase_s))
    return k0, vel, vol


def generar(spec: dict, salida: Path, procesos: int = 0, progreso=print) -> Path:
    c = _caso(spec)
    cfg = c["cfg"]
    per = 1.0 / cfg.scan_hz
    n = int(duracion_s(c) * cfg.scan_hz)
    procesos = procesos or os.cpu_count() or 1
    paso = max(4, math.ceil(n / (procesos * 4)))          # tramos chicos: el progreso avanza parejo
    tareas = [(spec, k, min(n, k + paso), 1000 * cfg.seed + k) for k in range(0, n, paso)]
    vel, vol = [None] * n, [None] * n
    hechos = 0
    with ProcessPoolExecutor(max_workers=procesos) as ex:
        for k0, sv, sw in ex.map(_tramo, tareas):
            vel[k0:k0 + len(sv)], vol[k0:k0 + len(sw)] = sv, sw
            hechos += len(sv)
            progreso(f"progreso {hechos}/{n}")
    s0, w0 = vel[0], vol[0]
    np.savez_compressed(
        salida, version=VERSION, nombre=c["name"], esperado=c["expect"],
        verdad_m3=float(c["obj"].volume_truth()), hz=cfg.scan_hz,
        vel_stamp=np.array([s.stamp for s in vel]), vol_stamp=np.array([s.stamp for s in vol]),
        # cada escaneo se entrega al terminar su vuelta (stamp + período), como PorticoSim.run
        vel_entrega=np.array([s.stamp + per for s in vel]), vol_entrega=np.array([s.stamp + per for s in vol]),
        vel_ranges=np.stack([s.ranges for s in vel]), vol_ranges=np.stack([s.ranges for s in vol]),
        vel_geo=np.array([s0.angle_min, s0.angle_increment, s0.time_increment, s0.range_min, s0.range_max]),
        vol_geo=np.array([w0.angle_min, w0.angle_increment, w0.time_increment, w0.range_min, w0.range_max]),
        vol_params=np.array(json.dumps(c["vol"])))
    progreso(f"listo {salida}")
    return salida


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--lista", action="store_true", help="lista los escenarios y termina")
    ap.add_argument("--caso", default="", help="nombre exacto de un escenario de la matriz")
    ap.add_argument("--kmh", type=float, default=7.5)
    ap.add_argument("--sentido", type=int, default=1)
    ap.add_argument("--lateral", type=float, default=0.0, help="desplazamiento lateral [m]")
    ap.add_argument("--guinada", type=float, default=0.0, help="ángulo de la trayectoria [°]")
    ap.add_argument("--salida", type=Path)
    ap.add_argument("-j", "--procesos", type=int, default=0)
    a = ap.parse_args()
    if a.lista:
        print("\n".join(nombres()))
        return
    if a.salida is None:
        ap.error("falta --salida")
    spec = dict(nombre=a.caso or PERSONALIZADO, kmh=a.kmh, sentido=a.sentido, lateral=a.lateral, guinada=a.guinada)
    generar(spec, a.salida, a.procesos, progreso=lambda s: print(s, flush=True))


if __name__ == "__main__":
    main()
