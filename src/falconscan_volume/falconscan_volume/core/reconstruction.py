"""Reconstrucción visual: nube de puntos y superficie del objeto en el MARCO DEL PASO.

Marco del paso: x = 0 en el frente del objeto y negativo hacia la cola (como el contorno del
simulador), y = izquierda del camión (y_camión = sentido · y), z = altura sobre el piso. Así la
reconstrucción sale igual entre por donde entre el camión.
"""
from __future__ import annotations

import json
import math
from dataclasses import dataclass
from pathlib import Path

import numpy as np

from .integration import Profile


@dataclass
class Reconstruction:
    points: np.ndarray          # (N, 4): x, y, z, h  (h = altura, para colorear)
    surface_x: np.ndarray       # (n,)  posición de cada perfil
    surface_y: np.ndarray       # (m,)  centros de la malla lateral (ascendentes)
    surface_h: np.ndarray       # (n, m) altura; NaN = sin dato


def build(profiles: list[Profile], x_front: float, direction: int, centers: np.ndarray,
          occupied_only: bool = True) -> Reconstruction:
    d = 1 if direction >= 0 else -1
    x_ref = x_front if math.isfinite(x_front) else (profiles[0].x if profiles else 0.0)
    pts, rows, xs = [], [], []
    for pr in profiles:
        s = pr.section
        if s.h is not None:
            rows.append(s.h if s.occupied else np.where(np.isfinite(s.h), 0.0, np.nan))
            xs.append(-(pr.x - x_ref))
        if occupied_only and not s.occupied:
            continue
        if s.points.size:
            y, z, t = s.points.T
            x = pr.x + pr.v * (t - s.stamp)          # cada punto en su propio instante
            pts.append(np.column_stack([-(x - x_ref), d * y, z, z]))
    points = np.vstack(pts) if pts else np.empty((0, 4))
    yy = d * np.asarray(centers, dtype=float)
    hh = np.vstack(rows) if rows else np.empty((0, yy.size))
    if d < 0:
        yy, hh = yy[::-1], hh[:, ::-1]
    return Reconstruction(points, np.asarray(xs, dtype=float), yy, hh)


def surface_triangles(rec: Reconstruction, min_h: float = 0.05, step: int = 1) -> np.ndarray:
    """Triángulos (T, 3, 3) de la superficie: dos por celda con sus 4 esquinas válidas y
    alguna por encima de ``min_h`` (no se dibuja el piso). ``step`` diezma columnas."""
    h = rec.surface_h[:, ::step]
    y = rec.surface_y[::step]
    x = rec.surface_x
    if h.shape[0] < 2 or h.shape[1] < 2:
        return np.empty((0, 3, 3))
    X, Y = np.meshgrid(x, y, indexing="ij")
    P = np.stack([X, Y, h], axis=-1)
    a, b, c, e = P[:-1, :-1], P[1:, :-1], P[:-1, 1:], P[1:, 1:]
    z4 = np.stack([a[..., 2], b[..., 2], c[..., 2], e[..., 2]])
    with np.errstate(invalid="ignore"):
        ok = np.all(np.isfinite(z4), axis=0) & (np.nanmax(z4, axis=0) > min_h)
    t1 = np.stack([a[ok], b[ok], e[ok]], axis=1)
    t2 = np.stack([a[ok], e[ok], c[ok]], axis=1)
    return np.concatenate([t1, t2])


def write_ply(path: str | Path, points: np.ndarray) -> None:
    """Nube en PLY ASCII (x, y, z, altura como 'intensity'). Lo abren CloudCompare y MeshLab."""
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w") as f:
        f.write("ply\nformat ascii 1.0\n")
        f.write(f"element vertex {len(points)}\n")
        f.write("property float x\nproperty float y\nproperty float z\nproperty float intensity\nend_header\n")
        np.savetxt(f, points[:, :4], fmt="%.4f")


def write_json(path: str | Path, data: dict) -> None:
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)

    def conv(o):
        if isinstance(o, (np.floating, np.integer)):
            return o.item()
        if isinstance(o, np.ndarray):
            return o.tolist()
        raise TypeError(type(o))

    path.write_text(json.dumps(data, indent=2, ensure_ascii=False, default=conv,
                               allow_nan=True))
