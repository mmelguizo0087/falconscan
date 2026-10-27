#!/usr/bin/env python3
"""Matriz de ensayos del reconstructor de volumen sobre el simulador 3D del pórtico.

Uso:
    python3 tools/evaluate_volume_sim.py                       # matriz completa
    python3 tools/evaluate_volume_sim.py --reps 5              # repetibilidad (fase y ruido distintos)
    python3 tools/evaluate_volume_sim.py --set vol_edge_from_velocity=false   # ablación
    python3 tools/evaluate_volume_sim.py --only "10 km/h" --plot /tmp/rec.png

Cada caso corre la cadena completa: LiDAR de velocidad (estimador + PassTracker) ->
reconstructor de volumen. Con --true-velocity también se corre con la velocidad verdadera,
para separar el error de área del error de velocidad.

Son resultados SINTÉTICOS: comparan variantes del algoritmo entre sí y verifican que cada
falla se detecte. No son evidencia de desempeño de campo.
"""
from __future__ import annotations

import argparse
import math
import sys
from concurrent.futures import ProcessPoolExecutor
from dataclasses import replace
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT / "src" / "falconscan_volume"), str(ROOT / "src" / "falconscan_velocity")]

from falconscan_volume.core import VolumeParams  # noqa: E402
from falconscan_volume.sim import (Box3D, DumpTruck3D, Motion3D, PorticoSim, PorticoSimConfig,  # noqa: E402
                                   lane_change, run_scenario, stop_and_go)

KMH = 1 / 3.6
TRUCK = DumpTruck3D()


def _accel(d0, v0, v1, a):
    """d(t): parte a v0, cambia con aceleración a hasta v1 y sigue constante."""
    t1 = (v1 - v0) / a

    def d(t):
        t = np.asarray(t, float)
        tc = np.minimum(t, t1)
        return d0 + v0 * tc + 0.5 * a * tc ** 2 + v1 * np.maximum(t - t1, 0)
    return d


def _case(name, expect, motion, cfg=None, obj=None, vol=None):
    return dict(name=name, expect=expect, motion=motion, cfg=cfg or PorticoSimConfig(), obj=obj or TRUCK,
                vol=vol or {})


def cases():
    """(nombre, resultado esperado, movimiento, config). Esperado: V = válida (o con
    advertencias), I:<bandera> = inválida por esa bandera, D = descartada."""
    out = []
    for kmh in (5, 7.5, 10):
        for d in (1, -1):
            out.append(_case(f"{kmh:g} km/h {'+x' if d > 0 else '-x'}", "V", Motion3D(direction=d, v=kmh * KMH, d0=-6.0)))
    v10 = 10 * KMH
    out += [
        # 5 -> 10 km/h con 0.3 m/s² y luego constante (y al revés)
        _case("acelera 5→10 km/h", "V", Motion3D(distance=_accel(-6.0, 1.39, 2.78, 0.3))),
        _case("frena 10→5 km/h", "V", Motion3D(distance=_accel(-6.0, 2.78, 1.39, -0.3))),
        _case("se detiene 3 s bajo el pórtico", "V", Motion3D(distance=stop_and_go(1.39, -6.0, 3.0, 3.0))),
        # recta en diagonal: el centro de la sección en el plano fijo NO se mueve (ver Motion3D)
        _case("diagonal recta 6°", "V", Motion3D(v=v10, d0=-6.0, yaw_rad=math.radians(6))),
        _case("cambio de carril 0.3 m", "V", Motion3D(v=v10, d0=-6.0, lateral=lane_change(0.3, -2.0, 6.0))),
        _case("cambio de carril 0.9 m", "I:CONDUCCION_NO_PARALELA",
              Motion3D(v=v10, d0=-6.0, lateral=lane_change(0.9, -2.0, 6.0))),
        _case("descentrado 0.4 m", "V", Motion3D(v=v10, d0=-6.0, y_offset=0.4)),
        _case("descentrado 0.6 m", "I:DESCENTRADO", Motion3D(v=v10, d0=-6.0, y_offset=0.6)),
        _case("fuera de ROI lateral 1.0 m", "I:TRUNCADO_LATERAL", Motion3D(v=v10, d0=-6.0, y_offset=1.0)),
        _case("escaneos perdidos 5 %", "V", Motion3D(v=v10, d0=-6.0), PorticoSimConfig(scan_dropout_prob=0.05)),
        _case("pérdida 15 % puntos", "V", Motion3D(v=v10, d0=-6.0), PorticoSimConfig(dropout_prob=0.15)),
        _case("ruido rango 3 cm", "V", Motion3D(v=v10, d0=-6.0), PorticoSimConfig(range_noise_m=0.03)),
        _case("reloj desfasado 0.25 s", "I:DESINCRONIZADO", Motion3D(v=v10, d0=-6.0),
              PorticoSimConfig(vol_clock_offset_s=0.25)),
        # un plano mal medido solo desplaza todos los x (no cambia V): se pierde el ajuste de
        # bordes y queda la advertencia; 1 m ya se ve como desincronía
        _case("plano mal medido +0.3 m", "V", Motion3D(v=v10, d0=-6.0), PorticoSimConfig(vol_plane_x=0.3)),
        _case("plano mal medido +1.0 m", "I:DESINCRONIZADO", Motion3D(v=v10, d0=-6.0),
              PorticoSimConfig(vol_plane_x=1.0)),
        _case("persona cruzando", "D", Motion3D(v=1.2, d0=-5.0), obj=Box3D(0.4, 0.6, 1.7)),
        _case("volco de 20 m (22.5 m)", "I:LARGO_FUERA_DE_RANGO", Motion3D(v=v10, d0=-6.0),
              obj=DumpTruck3D(box_len=20.0)),
    ]
    return out


def parse_overrides(items):
    out = {}
    names = set(VolumeParams.field_names())
    for it in items:
        k, v = it.split("=", 1)
        if k not in names:
            raise SystemExit(f"parámetro desconocido: {k}")
        default = getattr(VolumeParams(), k)
        if isinstance(default, bool):
            out[k] = v.lower() in ("1", "true", "si", "sí")
        elif isinstance(default, int):
            out[k] = int(float(v))
        else:
            out[k] = type(default)(v)
    return out


def run_one(args):
    idx, overrides, true_v, rep, reps = args
    case = cases()[idx]          # se reconstruye en el proceso (los movimientos usan lambdas)
    cfg = replace(case["cfg"], seed=1 + rep)
    motion = case["motion"]
    obj = case["obj"]
    if rep and isinstance(obj, DumpTruck3D):
        # Medidas distintas en cada repetición: con un largo múltiplo exacto de Δx (7.5 m a
        # 10 km/h = 27 perfiles) los errores de frente y cola se cancelaban y el punto medio
        # parecía mejor de lo que es.
        rng = np.random.default_rng(100 + rep)
        obj = replace(obj, cab_len=obj.cab_len + rng.uniform(-0.2, 0.2),
                      box_len=obj.box_len + rng.uniform(-0.4, 0.4), seed=rep)
    v_mean = max(0.5, abs(motion.speed(5.0)))
    if rep and motion.distance is None:
        # Fase de los perfiles respecto al camión repartida UNIFORMEMENTE en un espaciado
        # (Δx = v / f): el error de muestreo de los escalones internos depende de esa fase.
        motion = replace(motion, d0=motion.d0 - v_mean / cfg.scan_hz * rep / reps)
    dur = (abs(motion.d0) + obj.length + 7.0) / v_mean if motion.distance is None else 25.0
    sim = PorticoSim(cfg, obj, motion)
    vol = VolumeParams(**{**case["vol"], **overrides})
    r = run_scenario(sim, dur, volparams=vol, true_velocity=true_v)
    return case["name"], true_v, rep, r


def verdict(expect, m):
    if m is None:
        return "sin medición" if expect != "-" else "ok"
    if expect == "V":
        return "ok" if m.result in (0, 1) else "FALLA"
    if expect == "D":
        return "ok" if m.result == 3 else "FALLA"
    flag = expect.split(":", 1)[1]
    return "ok" if (m.result == 2 and flag in m.flags) else "FALLA"


def plot(m, path):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    rec = m.reconstruction
    fig = plt.figure(figsize=(13, 8))
    ax = fig.add_subplot(2, 2, (1, 3), projection="3d")
    p = rec.points[:: max(1, len(rec.points) // 40000)]
    ax.scatter(p[:, 0], p[:, 1], p[:, 2], c=p[:, 3], s=0.5, cmap="viridis")
    ax.set_xlabel("x [m] (frente = 0)")
    ax.set_ylabel("y [m]")
    ax.set_zlabel("z [m]")
    ax.set_box_aspect((np.ptp(p[:, 0]), np.ptp(p[:, 1]), np.ptp(p[:, 2])))
    ax.set_title(f"Nube del paso: V = {m.volume_m3:.2f} m³ ({m.result_text})")
    ax2 = fig.add_subplot(2, 2, 2)
    X, Y = np.meshgrid(rec.surface_x, rec.surface_y, indexing="ij")
    im = ax2.pcolormesh(X, Y, rec.surface_h, shading="nearest", cmap="viridis")
    fig.colorbar(im, ax=ax2, label="altura [m]")
    ax2.set_title("Superficie h(x, y)")
    ax2.set_aspect("equal")
    ax3 = fig.add_subplot(2, 2, 4)
    xa = np.array(m.area_profile)
    ax3.plot(xa[:, 0], xa[:, 1], "o-", ms=3)
    ax3.set_xlabel("x [m]")
    ax3.set_ylabel("área A(x) [m²]")
    ax3.set_title(f"Secciones: {m.n_profiles} perfiles, Δx máx {m.max_dx_m:.2f} m")
    fig.tight_layout()
    fig.savefig(path, dpi=110)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--set", action="append", default=[], metavar="param=valor")
    ap.add_argument("--reps", type=int, default=1,
                    help="repeticiones por caso: fases de los perfiles uniformes en un espaciado y ruido distinto")
    ap.add_argument("--true-velocity", action="store_true", help="también con la velocidad verdadera")
    ap.add_argument("--only", default="", help="solo los casos cuyo nombre contenga este texto")
    ap.add_argument("--plot", default="", help="guarda la reconstrucción del primer caso en este PNG")
    ap.add_argument("-j", "--jobs", type=int, default=0)
    args = ap.parse_args()
    overrides = parse_overrides(args.set)
    sel_idx = [i for i, c in enumerate(cases()) if args.only in c["name"]]
    sel = [cases()[i] for i in sel_idx]
    jobs = [(i, overrides, tv, rep, args.reps) for i in sel_idx for tv in ((False, True) if args.true_velocity else (False,))
            for rep in range(args.reps)]
    with ProcessPoolExecutor(max_workers=args.jobs or None) as ex:
        results = list(ex.map(run_one, jobs))

    print(f"Parámetros modificados: {args.set or 'ninguno'}; repeticiones: {args.reps}\n")
    hdr = (f"{'caso':30s} {'vel':4s} {'V verd':>7s} {'V med':>7s} {'error':>7s} {'σ rep':>6s} {'σ ext':>6s} "
           f"{'bordes':>11s} {'perf':>4s} {'Δt sync':>8s} {'resultado':>16s} {'chequeo':>7s}  banderas")
    print(hdr)
    print("-" * len(hdr))
    by = {}
    for name, tv, rep, r in results:
        by.setdefault((name, tv), []).append(r)
    errs_valid = []
    n_valid_cases = [0, 0]
    for c in sel:
        for tv in ((False, True) if args.true_velocity else (False,)):
            rs = by[(c["name"], tv)]
            r0 = rs[0]
            m = r0.m
            # error: solo repeticiones con volumen medido (en casos válidos, solo las aceptadas)
            use = [r for r in rs if r.m is not None and (c["expect"] != "V" or r.m.result in (0, 1))]
            errs = [r.error_pct for r in use]
            n_ok = sum(verdict(c["expect"], r.m) == "ok" for r in rs)
            if m is None:
                print(f"{c['name']:30s} {'ver' if tv else 'est':4s} {r0.truth_m3:7.2f} {'—':>7s}   sin medición")
                continue
            sd = float(np.std(errs)) if len(errs) > 1 else math.nan
            if c["expect"] == "V" and not tv:
                errs_valid += errs
                n_valid_cases[0] += n_ok
                n_valid_cases[1] += len(rs)
            flags = sorted({f for r in rs if r.m for f in r.m.flags})
            print(f"{c['name']:30s} {'ver' if tv else 'est':4s} {r0.truth_m3:7.2f} {m.volume_m3:7.2f} "
                  f"{np.mean(errs):+6.2f}% {sd:5.2f}% {100 * m.end_uncertainty_m3 / r0.truth_m3:5.2f}% "
                  f"{m.edge_method:>11s} {m.n_profiles:4d} {m.sync_offset_s:+8.3f} {m.result_text:>16s} "
                  f"{n_ok:>3d}/{len(rs):<3d}  {','.join(flags)}")
    if errs_valid:
        e = np.array(errs_valid)
        print(f"\nCasos válidos (velocidad estimada): aceptadas {n_valid_cases[0]}/{n_valid_cases[1]}; "
              f"error medio {e.mean():+.2f} %, |error| máx {np.abs(e).max():.2f} %, σ {e.std():.2f} % (n = {e.size})")
    print("\nerror = (V med − V verd)/V verd, promedio de las repeticiones; σ rep = desviación entre repeticiones;")
    print("σ ext = incertidumbre de extremos informada; chequeo = repeticiones cuyo resultado coincide con lo")
    print("esperado (válida / inválida por la bandera indicada / descartada). Resultado, bordes, perfiles y")
    print("Δt sync son de la primera repetición; banderas = unión de todas.")
    print("Volumen de la ENVOLVENTE (cabina + chasís + volco + carga). Simulación: no es evidencia de campo.")
    if args.plot and results:
        name, tv, rep, r = results[0]
        if r.m is not None:
            plot(r.m, args.plot)
            print(f"\nReconstrucción de «{name}» guardada en {args.plot}")


if __name__ == "__main__":
    main()
