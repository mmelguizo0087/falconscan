#!/usr/bin/env python3
"""Evaluación de la máquina de paso sobre el simulador: camiones seguidos en ambos sentidos.

Uso:
    python3 tools/evaluate_passes.py                     # VelocityPipeline con el yaml por defecto
    python3 tools/evaluate_passes.py --set empty_reset_scans=1   # reiniciar en cada vacío (política vieja)
    python3 tools/evaluate_passes.py --set pass_exit_scans=3

Por condición reporta: camiones, pasos cerrados (FIN), fragmentados (más de un FIN por camión),
perdidos, válidos (sin banderas), sentido mal asignado, % de escaneos con velocidad OK dentro
de los pasos y error de la mediana de rapidez de avance. Son resultados SINTÉTICOS: sirven para
comparar versiones, no para declarar desempeño de campo.
"""
from __future__ import annotations

import argparse
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src" / "falconscan_velocity"))

from falconscan_velocity.core import VelocityParams, VelocityPipeline  # noqa: E402
from falconscan_velocity.sim import LidarSim, LidarSimConfig, TruckMotion  # noqa: E402

from evaluate_sim import parse_overrides  # noqa: E402

SEPARACION_S = 4.0


def run_case(velocidades, cfg, params, p_ciego=0.0):
    """``p_ciego``: probabilidad por escaneo de que el camión no devuelva nada (carrocería oscura o
    mojada): ese escaneo solo ve el piso. Es lo que produce huecos de escaneos vacíos DENTRO de un
    paso; las pérdidas por rayo del simulador casi nunca vacían un escaneo entero."""
    pipe = VelocityPipeline(params)
    sim = LidarSim(cfg)
    rng = np.random.default_rng(11)
    lejos = TruckMotion(x0=100.0, v=0.0)
    eventos = []
    for s, _ in sim.run(TruckMotion(x0=100.0, v=0.0), 1.0):          # ROI vacía: sale de ARRANQUE
        eventos += pipe.process(s)[1]
    t0, ventanas = 1.0, []
    for v in velocidades:
        dur = 26.5 / abs(v) + SEPARACION_S
        m = TruckMotion(x0=(-10.0 if v > 0 else 16.5) - v * t0, v=v)
        for s, _ in sim.run(m, dur, t0=t0):
            if p_ciego and rng.random() < p_ciego:
                s = sim.scan_at(s.stamp, lejos)
            eventos += pipe.process(s)[1]
        ventanas.append((t0, t0 + dur, v))
        t0 += dur
    fines = [e for e in eventos if e.kind == "FIN"]
    por_camion = [[e for e in fines if a <= e.t_start < b] for a, b, _ in ventanas]
    frag = sum(len(f) > 1 for f in por_camion)
    perdidos = sum(len(f) == 0 for f in por_camion)
    mal_sentido = sum(1 for f, (_, _, v) in zip(por_camion, ventanas) if f and f[0].direction != np.sign(v))
    err = [f[0].v_median_mps - abs(v) for f, (_, _, v) in zip(por_camion, ventanas)   # rapidez de avance
           if f and np.isfinite(f[0].v_median_mps)]
    n_scans = sum(e.n_scans for e in fines)
    return dict(camiones=len(velocidades), fin=len(fines), frag=frag, perdidos=perdidos,
                validos=sum(e.valid for e in fines), sentido=mal_sentido,
                ok=sum(e.n_valid for e in fines) / max(1, n_scans),
                err=float(np.mean(err)) if err else float("nan"),
                err_max=float(np.max(np.abs(err))) if err else float("nan"))


def cases():
    """(nombre, velocidades, config del simulador, prob. de escaneo ciego)."""
    alternos = lambda v: [v, -v, v, -v, v, -v]   # noqa: E731
    for v in (1.39, 2.0, 2.78):                  # 5 / 7.2 / 10 km/h
        yield f"{v * 3.6:4.1f} km/h, ambos sentidos", alternos(v), LidarSimConfig(), 0.0
    for d in (0.15, 0.30):
        yield f"7.2 km/h, pérdida {d:.0%} rayos", alternos(2.0), LidarSimConfig(dropout_prob=d, seed=5), 0.0
    yield "7.2 km/h, ruido 3 cm", alternos(2.0), LidarSimConfig(range_noise_m=0.03, seed=7), 0.0
    for c in (0.05, 0.10, 0.20):
        yield f"7.2 km/h, {c:.0%} escaneos ciegos", alternos(2.0), LidarSimConfig(), c
    yield "10 km/h, 10% escaneos ciegos", alternos(2.78), LidarSimConfig(), 0.10


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--set", action="append", default=[], metavar="param=valor")
    args = ap.parse_args()
    params = VelocityParams(**parse_overrides(args.set))
    print(f"Parámetros modificados: {args.set or 'ninguno'}\n")
    hdr = (f"{'caso':28s} {'camiones':>8s} {'FIN':>4s} {'frag':>5s} {'perd':>5s} {'válidos':>8s} "
           f"{'sentido✗':>9s} {'v OK':>6s} {'err avance':>11s} {'máx':>6s}")
    print(hdr)
    print("-" * len(hdr))
    for nombre, vel, cfg, ciego in cases():
        r = run_case(vel, cfg, params, ciego)
        print(f"{nombre:28s} {r['camiones']:8d} {r['fin']:4d} {r['frag']:5d} {r['perdidos']:5d} "
              f"{r['validos']:8d} {r['sentido']:9d} {r['ok'] * 100:5.0f}% {r['err']:+11.3f} {r['err_max']:6.3f}")
    print("\nescaneos ciegos = el camión no devuelve nada en ese escaneo (solo se ve el piso).\n"
          "frag = camiones con más de un FIN; perd = camiones sin FIN; v OK = escaneos con velocidad OK\n"
          "(ocupados) de los pasos; err avance = mediana de rapidez de avance − |v real| [m/s].")


if __name__ == "__main__":
    main()
