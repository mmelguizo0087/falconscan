#!/usr/bin/env python3
"""Matriz de ensayos del reporte (§8.3) ejecutada sobre el simulador.

Uso:
    python3 tools/evaluate_sim.py                 # parámetros por defecto
    python3 tools/evaluate_sim.py --set baseline_scans=1 --set deskew_iterations=0

Corre estimador + PassTracker (``VelocityPipeline``), igual que el nodo: el sentido del
paso fija la búsqueda de la correlación (``search_mode``). Cada caso se corre en los dos
sentidos de circulación (+x y -x).

Reporta por condición: % de escaneos válidos con la volqueta en la ROI, sesgo,
MAE, RMSE (por intervalo y con EMA), pasos detectados y tiempo de cómputo por escaneo.
Son resultados SINTÉTICOS: sirven para comparar versiones del algoritmo
(ablación), no para declarar desempeño de campo.
"""
from __future__ import annotations

import argparse
import sys
import time
from dataclasses import replace
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src" / "falconscan_velocity"))

from falconscan_velocity.core import VelocityParams, VelocityPipeline  # noqa: E402
from falconscan_velocity.sim import LidarSim, LidarSimConfig, TruckMotion  # noqa: E402

TRUCK_LEN = 7.5


def run_case(motion, duration, params, cfg):
    sim, pipe = LidarSim(cfg), VelocityPipeline(params)
    d = cfg.direction
    in_roi, rows, t_proc, n_pass = 0, [], [], 0
    for scan, _ in sim.run(motion, duration):
        t0 = time.perf_counter()
        e, events = pipe.process(scan)
        t_proc.append(time.perf_counter() - t0)
        n_pass += sum(ev.kind == "INICIO" for ev in events)
        front = float(motion.x(np.array([scan.stamp]))[0])
        lo, hi = sorted((front, front - d * TRUCK_LEN))
        inside = hi > params.roi_x_min_m + 0.5 and lo < params.roi_x_max_m - 0.5
        in_roi += inside
        if e.valid:
            vr = motion.velocity(e.mid_stamp)
            rows.append((e.velocity_mps, e.velocity_filtered_mps, vr, inside))
    a = np.array(rows) if rows else np.full((1, 4), np.nan)
    n_ok_in = int(np.nansum(a[:, 3]))
    err = a[:, 0] - a[:, 2]
    errf = a[:, 1] - a[:, 2]
    return dict(valid=n_ok_in / max(1, in_roi), n=len(rows), bias=np.mean(err), mae=np.mean(np.abs(err)),
                rmse=np.sqrt(np.mean(err ** 2)), rmse_ema=np.sqrt(np.mean(errf ** 2)),
                max=np.max(np.abs(err)), ms=1e3 * np.mean(t_proc), passes=n_pass)


def _mirror(motion):
    """El mismo movimiento hacia -x: x(t) -> -x(t)."""
    if motion.position is None:
        return TruckMotion(x0=-motion.x0, v=-motion.v)
    f = motion.position
    return TruckMotion(position=lambda t: -f(t))


def cases():
    base = LidarSimConfig()
    out = []
    for v in (0.3, 0.8, 1.5, 3.0, 5.0):
        out.append((f"constante {v:.1f} m/s", TruckMotion(v=v), 26.5 / v, base))
    out += [
        ("acelera 0.5→3.2 m/s", TruckMotion(position=lambda t: -9.0 + 0.5 * t + 0.3 * t ** 2), 9.0, base),
        ("frena 3.0→0.6 m/s", TruckMotion(position=lambda t: -9.0 + 3.0 * t - 0.3 * t ** 2), 8.0, base),
        ("pérdida 15 % puntos", TruckMotion(v=1.5), 18.0, LidarSimConfig(dropout_prob=0.15)),
        ("escaneos perdidos 10 %", TruckMotion(v=1.5), 18.0, LidarSimConfig(scan_dropout_prob=0.10)),
        ("outliers 1 %", TruckMotion(v=1.5), 18.0, LidarSimConfig(outlier_prob=0.01)),
        ("ruido rango 3 cm", TruckMotion(v=1.5), 18.0, LidarSimConfig(range_noise_m=0.03)),
        ("LiDAR 5 Hz", TruckMotion(v=1.5), 18.0, LidarSimConfig(scan_hz=5.0, samples_per_rev=6400)),
    ]
    for name, motion, dur, cfg in out:
        yield name + " +x", motion, dur, cfg
        yield name + " -x", _mirror(motion), dur, replace(cfg, direction=-1)


def parse_overrides(items):
    out = {}
    names = set(VelocityParams.field_names())
    for it in items:
        k, v = it.split("=", 1)
        if k not in names:
            raise SystemExit(f"parámetro desconocido: {k}")
        default = getattr(VelocityParams(), k)
        out[k] = type(default)(float(v)) if isinstance(default, int) else type(default)(v)
    return out


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--set", action="append", default=[], metavar="param=valor")
    args = ap.parse_args()
    params = VelocityParams(**parse_overrides(args.set))
    print(f"Parámetros modificados: {args.set or 'ninguno'}\n")
    hdr = (f"{'caso':27s} {'válidos':>8s} {'sesgo':>8s} {'MAE':>7s} {'RMSE':>7s} {'RMSE EMA':>9s} "
           f"{'máx':>7s} {'pasos':>6s} {'ms/scan':>8s}")
    print(hdr)
    print("-" * len(hdr))
    for name, motion, dur, cfg in cases():
        r = run_case(motion, dur, params, cfg)
        print(f"{name:27s} {r['valid']*100:7.0f}% {r['bias']:+8.3f} {r['mae']:7.3f} {r['rmse']:7.3f} "
              f"{r['rmse_ema']:9.3f} {r['max']:7.3f} {r['passes']:6d} {r['ms']:8.1f}")
    print("\nErrores en m/s (marco x del sensor). Sesgo/MAE/RMSE por intervalo; 'RMSE EMA' con filtro"
          " temporal; 'pasos' = INICIO emitidos por el PassTracker (debe ser 1).")


if __name__ == "__main__":
    main()
