#!/usr/bin/env python3
"""Matriz de ensayos del reporte (§8.3) ejecutada sobre el simulador.

Uso:
    python3 tools/evaluate_sim.py                 # parámetros por defecto
    python3 tools/evaluate_sim.py --set baseline_scans=1 --set deskew_iterations=0

Reporta por condición: % de escaneos válidos con la volqueta en la ROI, sesgo,
MAE, RMSE (por intervalo y con EMA) y tiempo de cómputo por escaneo.
Son resultados SINTÉTICOS: sirven para comparar versiones del algoritmo
(ablación), no para declarar desempeño de campo.
"""
from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src" / "falconscan_velocity"))

from falconscan_velocity.core import VelocityEstimator, VelocityParams  # noqa: E402
from falconscan_velocity.sim import LidarSim, LidarSimConfig, TruckMotion  # noqa: E402

TRUCK_LEN = 7.5


def run_case(motion, duration, params, cfg):
    sim, est = LidarSim(cfg), VelocityEstimator(params)
    in_roi, rows, t_proc = 0, [], []
    for scan, _ in sim.run(motion, duration):
        t0 = time.perf_counter()
        e = est.process(scan)
        t_proc.append(time.perf_counter() - t0)
        front = float(motion.x(np.array([scan.stamp]))[0])
        inside = front > params.roi_x_min_m + 0.5 and front - TRUCK_LEN < params.roi_x_max_m - 0.5
        in_roi += inside
        if e.valid:
            vr = motion.velocity(e.mid_stamp)
            rows.append((e.velocity_mps, e.velocity_filtered_mps, vr, inside))
    a = np.array(rows) if rows else np.empty((0, 4))
    n_ok_in = int(a[:, 3].sum()) if len(a) else 0
    err = a[:, 0] - a[:, 2]
    errf = a[:, 1] - a[:, 2]
    return dict(valid=n_ok_in / max(1, in_roi), n=len(a), bias=err.mean(), mae=np.abs(err).mean(),
                rmse=np.sqrt((err ** 2).mean()), rmse_ema=np.sqrt((errf ** 2).mean()),
                max=np.abs(err).max(), ms=1e3 * np.mean(t_proc))


def cases():
    base = LidarSimConfig()
    for v in (0.3, 0.8, 1.5, 3.0, 5.0):
        yield f"constante {v:.1f} m/s", TruckMotion(v=v), 26.5 / v, base
    yield "acelera 0.5→3.2 m/s", TruckMotion(position=lambda t: -9.0 + 0.5 * t + 0.3 * t ** 2), 9.0, base
    yield "frena 3.0→0.6 m/s", TruckMotion(position=lambda t: -9.0 + 3.0 * t - 0.3 * t ** 2), 8.0, base
    yield "pérdida 15 % puntos", TruckMotion(v=1.5), 18.0, LidarSimConfig(dropout_prob=0.15)
    yield "escaneos perdidos 10 %", TruckMotion(v=1.5), 18.0, LidarSimConfig(scan_dropout_prob=0.10)
    yield "outliers 1 %", TruckMotion(v=1.5), 18.0, LidarSimConfig(outlier_prob=0.01)
    yield "ruido rango 3 cm", TruckMotion(v=1.5), 18.0, LidarSimConfig(range_noise_m=0.03)
    yield "LiDAR 5 Hz", TruckMotion(v=1.5), 18.0, LidarSimConfig(scan_hz=5.0, samples_per_rev=6400)


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
    hdr = f"{'caso':24s} {'válidos':>8s} {'sesgo':>8s} {'MAE':>7s} {'RMSE':>7s} {'RMSE EMA':>9s} {'máx':>7s} {'ms/scan':>8s}"
    print(hdr)
    print("-" * len(hdr))
    for name, motion, dur, cfg in cases():
        r = run_case(motion, dur, params, cfg)
        print(f"{name:24s} {r['valid']*100:7.0f}% {r['bias']:+8.3f} {r['mae']:7.3f} {r['rmse']:7.3f} "
              f"{r['rmse_ema']:9.3f} {r['max']:7.3f} {r['ms']:8.1f}")
    print("\nErrores en m/s. Sesgo/MAE/RMSE por intervalo; 'RMSE EMA' con filtro temporal.")


if __name__ == "__main__":
    main()
