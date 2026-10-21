"""Piezas compartidas por el visor y la pestaña de prueba de paso: paleta, tarjetas y ajuste de recta."""
from __future__ import annotations

import os

import numpy as np
from pyqtgraph.Qt import QtWidgets

# Paleta (tema oscuro)
FONDO, PANEL, BORDE = "#12161c", "#1b2129", "#2a323d"
TEXTO, TEXTO_SUAVE = "#e6edf3", "#8b98a5"
ACENTO, VERDE, AMBAR, ROJO = "#2fb8c4", "#3fb950", "#d29922", "#f85149"


def pantalla_compacta() -> bool:
    """Pantalla chica (p. ej. 7" de 1024×600): pantalla completa, registro y parámetros plegados.

    FALCONSCAN_COMPACTO=1/0 lo fuerza (para probar el diseño en un monitor grande).
    """
    forzado = os.environ.get("FALCONSCAN_COMPACTO")
    if forzado in ("0", "1"):
        return forzado == "1"
    pantalla = QtWidgets.QApplication.primaryScreen()
    if pantalla is None:
        return False
    g = pantalla.availableGeometry()
    return g.width() <= 1100 or g.height() <= 700


def titulo_grafico(texto: str) -> str:
    return f"<span style='color:{TEXTO};font-size:10pt'>{texto}</span>"


def ros_a_sensor(a_ros_deg):
    """Ángulo ROS [°] -> ángulo del sensor en [−180, 180)."""
    return (360.0 - np.asarray(a_ros_deg, dtype=float)) % 360 - 180


def ajuste_pared(x: np.ndarray, y: np.ndarray, tol: float = 0.03, intentos: int = 40):
    """Recta robusta: RANSAC (pares de puntos, inliers a < tol) y luego mínimos cuadrados totales.

    RANSAC evita que unos pocos puntos de otro objeto en el borde del sector arrastren la recta
    (el recorte iterativo de analiza_pared.py parte de una recta ya contaminada).
    Devuelve (distancia perpendicular al origen, σ residual, ángulo de la normal en grados, n puntos)
    o None si hay pocos puntos.
    """
    ok = np.isfinite(x) & np.isfinite(y)
    P = np.stack([x[ok], y[ok]], 1)
    n = len(P)
    if n < 8:
        return None
    rng = np.random.default_rng(0)
    i, j = rng.integers(0, n, (2, intentos))
    d = P[j] - P[i]
    largo = np.hypot(d[:, 0], d[:, 1])
    validos = largo > 1e-6
    nrm = np.stack([-d[validos, 1], d[validos, 0]], 1) / largo[validos, None]
    dist = np.abs((P[None, :, :] - P[i[validos], None, :]) @ nrm[:, :, None])[..., 0]   # intentos × n
    if dist.size == 0:
        return None
    keep = dist[np.argmax((dist < tol).sum(1))] < tol
    if keep.sum() < 8:
        return None
    for _ in range(2):   # refinamiento TLS sobre los inliers
        c = P[keep].mean(0)
        _, _, vt = np.linalg.svd(P[keep] - c, full_matrices=False)
        e = (P - c) @ vt[1]
        nuevo = np.abs(e) < max(0.01, 3 * e[keep].std())
        if nuevo.sum() < 8:
            break
        keep = nuevo
    c = P[keep].mean(0)
    _, _, vt = np.linalg.svd(P[keep] - c, full_matrices=False)
    m = vt[1] * (np.sign(c @ vt[1]) or 1.0)
    e = (P[keep] - c) @ m
    return abs(c @ m), e.std(), np.degrees(np.arctan2(m[1], m[0])) % 360, int(keep.sum())


def tarjeta(titulo: str, valor: str = "—", tip: str = "") -> tuple[QtWidgets.QWidget, QtWidgets.QLabel]:
    w = QtWidgets.QFrame()
    w.setStyleSheet(f"QFrame {{ background: {PANEL}; border: 1px solid {BORDE}; border-radius: 7px; }}")
    lay = QtWidgets.QVBoxLayout(w)
    lay.setContentsMargins(9, 3, 9, 3)
    lay.setSpacing(0)
    t = QtWidgets.QLabel(titulo.upper())
    t.setStyleSheet(f"color: {TEXTO_SUAVE}; font-size: 9px; font-weight: 600; border: none;")
    v = QtWidgets.QLabel(valor)
    v.setStyleSheet(f"font-size: {TARJETA_PX}px; font-weight: 700; border: none;")
    lay.addWidget(t)
    lay.addWidget(v)
    if tip:
        w.setToolTip(tip)
    return w, v


TARJETA_PX = 15      # tamaño del valor en las tarjetas


def chip(titulo: str, tip: str = "") -> tuple[QtWidgets.QWidget, QtWidgets.QLabel]:
    """Métrica de una línea para la barra de estado: «TÍTULO valor»."""
    w = QtWidgets.QWidget()
    lay = QtWidgets.QHBoxLayout(w)
    lay.setContentsMargins(8, 0, 8, 0)
    lay.setSpacing(5)
    t = QtWidgets.QLabel(titulo.upper())
    t.setStyleSheet(f"color: {TEXTO_SUAVE}; font-size: 10px; font-weight: 600;")
    v = QtWidgets.QLabel("—")
    v.setStyleSheet("font-size: 12px; font-weight: 700;")
    lay.addWidget(t)
    lay.addWidget(v)
    if tip:
        w.setToolTip(tip)
    return w, v
