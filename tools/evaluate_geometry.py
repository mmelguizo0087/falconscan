#!/usr/bin/env python3
"""Barrido geométrico en el simulador: ¿la velocidad es igual de buena con objetos grandes y pequeños?

Recorre altura del objeto relativa al sensor (η = h/H), largo del objeto, velocidad, perfil del
techo (plano o con textura) y offset de rango residual b, para dos montajes: el de laboratorio
(H = 2.216 m) y un pórtico (H = 6 m, objetos escalados por 6/2.216). Por caso informa, sobre los
escaneos con el objeto dentro de la ROI: % válidos, sesgo y dispersión del error relativo, p95,
el hueco más largo sin velocidad y el error de la mediana del paso. Junto a cada caso van los
cocientes de diseño: holgura c = H − h, ventana sobre el techo W = min(2c·tan(abanico/2), ROI),
λ = L/W y v·Δt/W.

Uso:
    python3 tools/evaluate_geometry.py                      # barrido completo (varios minutos)
    python3 tools/evaluate_geometry.py --rapido             # subconjunto para probar
    python3 tools/evaluate_geometry.py --semillas 5 --csv salida.csv

Son resultados SINTÉTICOS (ruido de rango 3 mm como el S2E en la prueba de pared, objetos
rectangulares ideales): sirven para elegir qué probar con el sensor, no como desempeño de campo.
"""
from __future__ import annotations

import argparse
import csv
import itertools
import math
import os
import sys
import time
import warnings
from concurrent.futures import ProcessPoolExecutor
from dataclasses import replace
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src" / "falconscan_velocity"))

from falconscan_velocity.core import VelocityEstimator, VelocityParams  # noqa: E402
from falconscan_velocity.sim import LidarSim, LidarSimConfig, TruckMotion  # noqa: E402

H_LAB = 2.216
ABANICO_DEG = 120.0
MONTAJES = {"laboratorio": H_LAB, "pórtico": 6.0}
ETAS = (0.3, 0.45, 0.6, 0.7, 0.8)    # 0.2 en el laboratorio (0.44 m) queda en el umbral de objeto
LARGOS_LAB = (0.8, 1.6, 3.0, 6.0)          # en el pórtico se multiplican por H/H_LAB
VELOCIDADES = (1.39, 2.78)                  # 5 y 10 km/h
PERFILES = ("plano", "textura")
OFFSETS_B = (0.0, 0.004)                    # offset de rango residual sin corregir [m]


def objeto(h: float, largo: float, perfil: str, escala: float, rng: np.random.Generator) -> np.ndarray:
    """Contorno lateral (frente en x = 0, cola en x = −largo, apoyado en el piso).

    «textura»: techo en escalones de 0.5·escala m con alturas entre h − 0.15·escala y h (el punto
    más alto es h, así la holgura c = H − h es la misma que con el techo plano).
    """
    if perfil == "plano":
        return np.array([(0.0, 0.0), (0.0, h), (-largo, h), (-largo, 0.0)])
    paso = 0.5 * escala
    n = max(2, int(round(largo / paso)))
    xs = np.linspace(0.0, -largo, n + 1)
    alturas = h - 0.15 * escala * rng.random(n)
    alturas[rng.integers(n)] = h
    techo = [(0.0, 0.0)]
    for i in range(n):
        techo += [(xs[i], alturas[i]), (xs[i + 1], alturas[i])]
    techo.append((-largo, 0.0))
    return np.array(techo)


def cocientes(H, h, largo, v, p: VelocityParams):
    c = H - h
    ventana = min(2 * c * math.tan(p.fan_angle_rad / 2), p.roi_x_max_m - p.roi_x_min_m)
    dt = p.baseline_scans / 10.0
    return c, ventana, largo / ventana, v * dt / ventana


def correr_caso(caso):
    montaje, eta, largo_lab, v, perfil, b, semillas = caso
    H = MONTAJES[montaje]
    escala = H / H_LAB
    h, largo = eta * H, largo_lab * escala
    p = VelocityParams(mount_height_m=H, fan_angle_rad=math.radians(ABANICO_DEG), roi_z_max_m=H - 0.1)
    errores, huecos, medianas, n_dentro, n_ok, t_proc, ok_paso = [], [], [], 0, 0, [], []
    for s in range(semillas):
        rng = np.random.default_rng(1000 + s)
        cfg = LidarSimConfig(height_m=H, samples_per_rev=3240, range_noise_m=0.003, seed=s + 1)
        sim = LidarSim(cfg, truck=objeto(h, largo, perfil, escala, rng))
        est = VelocityEstimator(p)
        x0 = p.roi_x_min_m - 1.0
        m = TruckMotion(x0=x0, v=v)
        dur = (p.roi_x_max_m - x0 + largo + 1.0) / v
        hueco, hueco_max, vs = 0, 0, []
        for scan, _ in sim.run(m, dur):
            if b:
                scan = replace(scan, ranges=scan.ranges + np.float32(b))
            t0 = time.perf_counter()
            e = est.process(scan)
            t_proc.append(time.perf_counter() - t0)
            frente = float(m.x(np.array([scan.stamp]))[0])
            dentro = frente > p.roi_x_min_m + 0.5 and frente - largo < p.roi_x_max_m - 0.5
            if not dentro:
                continue
            n_dentro += 1
            if e.valid:
                n_ok += 1
                rel = (e.velocity_mps - v) / v
                errores.append(rel)
                vs.append(e.velocity_mps)
                hueco = 0
            else:
                hueco += 1
                hueco_max = max(hueco_max, hueco)
        huecos.append(hueco_max)
        ok_paso.append(len(vs))
        if vs:
            medianas.append((np.median(vs) - v) / v)
    c, ventana, lam, avance = cocientes(H, h, largo, v, p)
    e = np.array(errores) if errores else np.array([np.nan])
    return dict(montaje=montaje, H=H, eta=eta, h=h, largo=largo, v=v, perfil=perfil, b_mm=b * 1e3,
                c=c, W=ventana, lam=lam, avance_W=avance,
                validos=n_ok / max(1, n_dentro), ok_por_paso=float(np.mean(ok_paso)),
                en_roi_por_paso=n_dentro / semillas, sesgo=float(np.nanmean(e)), std=float(np.nanstd(e)),
                p95=float(np.nanpercentile(np.abs(e), 95)), hueco=int(max(huecos)),
                med_paso=float(np.mean(medianas)) if medianas else float("nan"),
                med_paso_max=float(np.max(np.abs(medianas))) if medianas else float("nan"),
                b_predicho=b / c, ms=1e3 * float(np.mean(t_proc)))


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--rapido", action="store_true", help="solo laboratorio, 10 km/h, sin offset b")
    ap.add_argument("--semillas", type=int, default=3, help="repeticiones por caso (ruido y textura)")
    ap.add_argument("--procesos", type=int, default=os.cpu_count())
    ap.add_argument("--csv", type=Path, default=None)
    args = ap.parse_args()
    montajes = ("laboratorio",) if args.rapido else tuple(MONTAJES)
    vel = (2.78,) if args.rapido else VELOCIDADES
    offs = (0.0,) if args.rapido else OFFSETS_B
    casos = [(mo, eta, L, v, pf, b, args.semillas)
             for mo, eta, L, v, pf, b in itertools.product(montajes, ETAS, LARGOS_LAB, vel, PERFILES, offs)]
    print(f"{len(casos)} casos × {args.semillas} semillas, {args.procesos} procesos · abanico {ABANICO_DEG:.0f}° · "
          "ROI ±3 m · ruido 3 mm · SINTÉTICO\n", flush=True)
    t0 = time.time()
    with warnings.catch_warnings():
        warnings.simplefilter("ignore")
        with ProcessPoolExecutor(args.procesos) as ex:
            filas = list(ex.map(correr_caso, casos, chunksize=1))
    hdr = (f"{'montaje':11s} {'η':>4s} {'h':>5s} {'L':>5s} {'km/h':>4s} {'perfil':7s} {'b':>3s} │ {'c':>5s} {'W':>4s} "
           f"{'λ':>4s} {'vΔt/W':>5s} │ {'OK/paso':>7s} {'válid':>5s} {'sesgo':>6s} {'σ':>5s} {'p95':>5s} {'hueco':>5s} {'paso':>6s}")
    print(hdr)
    print("─" * len(hdr))
    for r in filas:
        print(f"{r['montaje']:11s} {r['eta']:4.1f} {r['h']:5.2f} {r['largo']:5.1f} {r['v'] * 3.6:4.0f} {r['perfil']:7s} "
              f"{r['b_mm']:3.0f} │ {r['c']:5.2f} {r['W']:4.1f} {r['lam']:4.1f} {r['avance_W']:5.2f} │ "
              f"{r['ok_por_paso']:4.0f}/{r['en_roi_por_paso']:<3.0f}{r['validos'] * 100:4.0f}% {r['sesgo'] * 100:+5.2f}% {r['std'] * 100:4.1f}% {r['p95'] * 100:4.1f}% "
              f"{r['hueco']:5d} {r['med_paso'] * 100:+5.2f}%")
    print(f"\n{time.time() - t0:.0f} s. Errores RELATIVOS de velocidad por escaneo (sesgo = media, σ, p95 de |error|) "
          "con el objeto en la ROI;\nOK/paso = escaneos con velocidad / escaneos con el objeto en la ROI, por paso;\nhueco = escaneos seguidos sin velocidad (peor semilla); paso = error de la mediana "
          "del paso (media de semillas).\nc = H − h, W = ventana sobre el techo, λ = L/W. SINTÉTICO.")
    if args.csv:
        args.csv.parent.mkdir(parents=True, exist_ok=True)
        with args.csv.open("w", newline="") as f:
            w = csv.DictWriter(f, fieldnames=list(filas[0]))
            w.writeheader()
            w.writerows(filas)
        print(f"CSV: {args.csv}")


if __name__ == "__main__":
    main()
