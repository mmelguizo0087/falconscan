"""Pestaña «Calibración» del visor: calibración del montaje de LOS DOS LiDAR del pórtico.

LiDAR de velocidad (longitudinal, plano x–z, 192.168.8.8) y de volumen (transversal, plano y–z,
192.168.11.2), D22. Pasos:
  1. Datos del montaje: altura con cinta de cada sensor, relación entre los dos
     (``vol_plane_offset_m``: distancia sobre x entre el LiDAR de velocidad y el plano transversal;
     hoy 0 cm) y posición lateral.
  2. Piso vacío, los dos a la vez: offset de rango y desvío de la flecha 0° de cada uno
     (core/calibration.py; H = cinta, fijo).
  3. Orientación (opcional): sentido de avance (+x) con un objeto del lado de entrada en el plano
     longitudinal, y lado izquierdo (+y) con un objeto a la izquierda en el plano transversal.
  4. Relación (opcional): un objeto delgado en el cruce de los dos planos. El longitudinal da su x
     (debe ser ``vol_plane_offset_m``), el transversal su y (``vel_plane_y_m``) y los dos su altura.
  5. Aplicar (valores efectivos en «Prueba de paso» y «Volumen 3D») y guardar la ficha.

Los datos llegan de los drivers del pórtico (portico.py) o del simulador del pórtico.
"""
from __future__ import annotations

import datetime as dt
import json
import math
import threading
import time
from dataclasses import dataclass, field, replace
from pathlib import Path

import numpy as np
import pyqtgraph as pg
from pyqtgraph.Qt import QtCore, QtWidgets

from comun import ACENTO, AMBAR, ROJO, TEXTO_SUAVE, VERDE, bordes_abanico, ros_a_sensor, titulo_grafico
from paso import _spin, cargar_params, guardar_params, nadir_ros_deg, offset_desde_nadir
from portico import LIDARS, VEL, VOL, Flujo
from volumen import cargar_params as cargar_ambos

from falconscan_velocity.core import (MountGeometry, calibrate_mount, detect_direction, fan_mask, object_position,
                                      polar_to_xz, validate_ranges)
from falconscan_velocity.core.calibration import CORREGIDO, FALLA, OK

ESPERA_REAL_S = 15.0     # sin completar la captura en este tiempo -> se usa lo que haya o se cancela
ESPERA_SIM_S = 300.0     # el simulador 3D cuesta ~1 s por escaneo en la RPi
N_SIM = 5                # escaneos por LiDAR en una captura simulada
TOL_RELACION_M = 0.05    # diferencia aceptable entre la relación medida y la configurada

EJE = {"vel": "x: avance →  [m]", "vol": "y: lateral, + a la izquierda [m]"}
TITULO = {"vel": "Longitudinal (velocidad) · 192.168.8.8 · plano x–z",
          "vol": "Transversal (volumen) · 192.168.11.2 · plano y–z"}


@dataclass
class EstadoLidar:
    """Captura y resultado de un LiDAR."""
    pila_vacio: np.ndarray | None = None     # rangos corregidos del piso vacío (N × M)
    ang: np.ndarray | None = None
    res: object | None = None                # MountCalibration
    error: str = ""
    dir_detectado: int | None = None         # angle_direction del paso 3
    corr: tuple[float, float] = (1.0, 0.0)   # (range_scale, range_bias_m) con que se corrigió la captura
    cruce: tuple[float, float, int] | None = None   # paso 4: (coordenada, altura, rayos)
    extra: dict = field(default_factory=dict)


# ----------------------------------------------------------------------------- simulador

class SimCalibracion(threading.Thread):
    """Escenas quietas del simulador del pórtico para probar la rutina sin hardware.

    vacio: sin objetos · entrada: caja del lado de entrada en el plano longitudinal (x < 0) ·
    izquierda: caja a la izquierda en el plano transversal (y > 0) · cruce: poste en el cruce.
    """

    def __init__(self, escena: str, que: set[str], flujos: dict[str, Flujo], n: int):
        super().__init__(daemon=True)
        self.escena, self.que, self.flujos, self.n = escena, que, flujos, n
        self.parar = threading.Event()

    def run(self):
        from falconscan_volume.sim import Box3D, DumpTruck3D, Motion3D, PorticoSim, PorticoSimConfig
        obj, mov = {
            "vacio": (DumpTruck3D(), Motion3D(v=0.0, d0=-60.0)),
            "entrada": (Box3D(0.5, 0.6, 0.5), Motion3D(v=0.0, d0=-1.0)),
            "izquierda": (Box3D(0.6, 0.6, 0.6), Motion3D(v=0.0, d0=0.3, y_offset=1.0)),
            "cruce": (Box3D(0.1, 0.1, 1.5), Motion3D(v=0.0, d0=0.05)),
        }[self.escena]
        sim = PorticoSim(PorticoSimConfig(march_step_m=0.02), obj, mov)
        t = time.monotonic()
        for k in range(self.n):
            for clave in ("vel", "vol"):
                if self.parar.is_set():
                    return
                if clave in self.que:
                    s = sim.velocity_scan(t + 0.1 * k) if clave == "vel" else sim.volume_scan(t + 0.1 * k)
                    self.flujos[clave].agregar(s)


# ----------------------------------------------------------------------------- panel

class PanelCalibracion(QtWidgets.QWidget):
    """Contenido de la pestaña «Calibración». Aplica en «Prueba de paso» y «Volumen 3D»."""

    def __init__(self, ventana, carpeta_datos: Path):
        super().__init__()
        self.v = ventana
        self.portico = ventana.portico
        self.carpeta = carpeta_datos / "calibraciones"
        self.est = {l.clave: EstadoLidar() for l in LIDARS}
        self.flujos_sim = {l.clave: Flujo() for l in LIDARS}
        self._sim_hilo: SimCalibracion | None = None
        self._vol_cache = (None, None)       # (mtime del yaml, VolumeParams)
        self._captura = None                 # (n, inicio por LiDAR, t0, que, al_terminar, espera)
        self._iniciado = False
        self._t = QtCore.QTimer(self, interval=100, timeout=self._sondear)
        self._t_vivo = QtCore.QTimer(self, interval=250, timeout=self._vivo)
        self._construir()
        self.portico.cambio.connect(self._estado_fuente)

    # ------------------------------------------------------------------ parámetros
    def _nominales(self, clave: str):
        """Parámetros del yaml (nominales) de ese LiDAR: VelocityParams o VolumeParams."""
        if clave == "vel":
            return self.v.paso.params_yaml
        ruta = self.v.paso.yaml_ruta
        mtime = ruta.stat().st_mtime
        if self._vol_cache[0] != mtime:
            self._vol_cache = (mtime, cargar_ambos(ruta)[1])
        return self._vol_cache[1]

    def _sim(self) -> bool:
        return self.c_fuente.currentIndex() == 1

    def _flujos(self) -> dict[str, Flujo]:
        return self.flujos_sim if self._sim() else self.portico.flujos

    def _corr(self, clave: str) -> tuple[float, float]:
        """(range_scale, range_bias_m) de la unidad (yaml); el simulador publica rangos exactos."""
        if self._sim():
            return 1.0, 0.0
        p = self._nominales(clave)
        return p.range_scale, p.range_bias_m

    # ------------------------------------------------------------------ construcción
    def _construir(self):
        div = QtWidgets.QSplitter(QtCore.Qt.Orientation.Horizontal)
        izq = QtWidgets.QTabWidget()
        izq.setDocumentMode(True)
        planos = QtWidgets.QSplitter(QtCore.Qt.Orientation.Vertical)
        planitud = QtWidgets.QSplitter(QtCore.Qt.Orientation.Vertical)
        self.g, self.items, self.g_res, self.res_items = {}, {}, {}, {}
        for l in LIDARS:
            c = l.clave
            g = pg.PlotWidget()
            g.setTitle(titulo_grafico(TITULO[c]))
            g.setAspectLocked(True)
            g.showGrid(x=True, y=True, alpha=0.12)
            g.setLabel("bottom", EJE[c])
            g.setLabel("left", "z [m]")
            it = {}
            it["roi"] = QtWidgets.QGraphicsRectItem()
            it["roi"].setPen(pg.mkPen(ACENTO, width=1, style=QtCore.Qt.PenStyle.DashLine))
            it["roi"].setBrush(pg.mkBrush(47, 184, 196, 18))
            g.addItem(it["roi"])
            g.addItem(pg.InfiniteLine(angle=0, pen=pg.mkPen("#8b98a5", width=2)))
            it["otros"] = pg.ScatterPlotItem(size=3, pen=None, brush=pg.mkBrush("#c9d1d9"))
            it["piso"] = pg.ScatterPlotItem(size=3, pen=None, brush=pg.mkBrush(VERDE))
            it["alto"] = pg.ScatterPlotItem(size=4, pen=None, brush=pg.mkBrush(ROJO))
            it["sensor"] = pg.ScatterPlotItem(size=14, brush=pg.mkBrush(ACENTO), pen=pg.mkPen("w"))
            it["plomada"] = pg.PlotDataItem(pen=pg.mkPen(AMBAR, width=1, style=QtCore.Qt.PenStyle.DashLine))
            it["abanico"] = pg.PlotDataItem(pen=pg.mkPen(ACENTO, width=1, style=QtCore.Qt.PenStyle.DotLine),
                                            connect="finite")
            it["cruce"] = pg.InfiniteLine(angle=90, pen=pg.mkPen("#a371f7", width=1, style=QtCore.Qt.PenStyle.DashLine))
            for k in ("abanico", "otros", "piso", "alto", "plomada", "sensor", "cruce"):
                g.addItem(it[k])
            it["cruce"].hide()
            it["leyenda"] = pg.TextItem("", color=TEXTO_SUAVE, anchor=(0, 0))      # bajo el piso
            g.addItem(it["leyenda"])
            self.g[c], self.items[c] = g, it
            planos.addWidget(g)

            gr = pg.PlotWidget()
            gr.setTitle(titulo_grafico(f"Planitud del piso · {l.nombre}"))
            gr.showGrid(x=True, y=True, alpha=0.15)
            gr.setLabel("left", "z [mm]")
            gr.setLabel("bottom", EJE[c])
            ri = {"puntos": pg.ScatterPlotItem(size=3, pen=None, brush=pg.mkBrush(VERDE)),
                  "mediana": pg.PlotDataItem(pen=pg.mkPen(ACENTO, width=2), connect="finite")}
            gr.addItem(ri["puntos"])
            gr.addItem(ri["mediana"])
            gr.addItem(pg.InfiniteLine(angle=0, pen=pg.mkPen("#8b98a5")))
            self.g_res[c], self.res_items[c] = gr, ri
            planitud.addWidget(gr)
        izq.addTab(planos, "Planos de escaneo")
        izq.addTab(planitud, "Planitud del piso")
        div.addWidget(izq)

        panel = QtWidgets.QWidget()
        lp = QtWidgets.QVBoxLayout(panel)
        lp.setContentsMargins(4, 0, 0, 0)

        # 0 · fuente
        g = QtWidgets.QGroupBox("Fuente: los dos LiDAR")
        f = QtWidgets.QGridLayout(g)
        self.c_fuente = QtWidgets.QComboBox()
        self.c_fuente.addItems(["LiDAR del pórtico", "Simulador del pórtico"])
        self.c_fuente.setToolTip("LiDAR del pórtico: velocidad en 192.168.8.8 y volumen en 192.168.11.2 (drivers del\n"
                                 "visor o portico.launch.py). Simulador: escenas quietas del simulador 3D para probar\n"
                                 "la rutina (no calibra los sensores reales; ~1 s por escaneo en la RPi).")
        self.c_fuente.currentIndexChanged.connect(self._fuente_cambiada)
        f.addWidget(self.c_fuente, 0, 0)
        self.b_encender = QtWidgets.QPushButton("▶ Encender LiDAR", objectName="iniciar")
        self.b_encender.setToolTip("Lanza los dos drivers (gira el motor) y escucha /lidar_velocity/scan y\n"
                                   "/lidar_volume/scan. Lo mismo usa la fuente ROS de «Volumen 3D».")
        self.b_encender.clicked.connect(self._encender)
        f.addWidget(self.b_encender, 0, 1)
        self.lbl_fuente = QtWidgets.QLabel("")
        self.lbl_fuente.setWordWrap(True)
        f.addWidget(self.lbl_fuente, 1, 0, 1, 2)
        lp.addWidget(g)

        # 1 · datos del montaje (dos columnas)
        g = QtWidgets.QGroupBox("1 · Datos del montaje")
        f = QtWidgets.QGridLayout(g)
        f.setHorizontalSpacing(6)
        for j, t in enumerate(("Velocidad", "Volumen"), start=1):
            lb = QtWidgets.QLabel(f"<b>{t}</b>")
            lb.setAlignment(QtCore.Qt.AlignmentFlag.AlignCenter)
            f.addWidget(lb, 0, j)
        self.chk_usar, self.c_h, self.c_abajo, self.lbl_abajo, self.c_zmax = {}, {}, {}, {}, {}
        self.chk_pend, self.c_pend = {}, {}
        fila = 1

        def fila_par(etiqueta, crear, tip=""):
            nonlocal fila
            lb = QtWidgets.QLabel(etiqueta)
            lb.setToolTip(tip)
            f.addWidget(lb, fila, 0)
            out = {}
            for j, l in enumerate(LIDARS, start=1):
                w = crear(l.clave)
                if w is not None:
                    w.setToolTip(tip)
                    f.addWidget(w, fila, j)
                out[l.clave] = w
            fila += 1
            return out

        def usar(c):
            w = QtWidgets.QCheckBox("calibrar")
            w.setChecked(True)
            w.toggled.connect(self._habilitar)
            return w
        self.chk_usar = fila_par("", usar, "Desmarca un LiDAR para calibrar solo el otro (p. ej. si no está conectado)")

        def altura(c):
            w = _spin(0.1, 30, 3, 0.01, "m")
            w.setValue(3.0)
            return w
        self.c_h = fila_par("Altura con cinta", altura,
                            "mount_height_m: altura VERTICAL (plomada) del piso al centro óptico (eje de giro del S2E).\n"
                            "Se toma como verdadera: la diferencia con lo que ve el sensor es el offset de rango.")

        def abajo(c):
            w = _spin(0, 359.99, 2, 1.0, "°")
            w.setWrapping(True)
            w.valueChanged.connect(self._abajo_txt)
            return w
        self.c_abajo = fila_par("Rayo abajo (ROS)", abajo,
                                "Ángulo ROS aproximado del rayo que mira al piso (±25°), solo para buscar el piso.\n"
                                "Nominal: flecha 0° del S2E al piso = ROS 180°. El desvío se mide contra el nominal.")

        def abajo_txt(c):
            w = QtWidgets.QLabel()
            w.setObjectName("suave")
            return w
        self.lbl_abajo = fila_par("", abajo_txt)

        def zmax(c):
            w = _spin(0.5, 30, 2, 0.1, "m")
            w.setValue(4.5)
            return w
        self.c_zmax = fila_par("Altura máx.", zmax,
                               "Puntos más altos sobre el piso se descartan (soporte, estructura). Ponlo por debajo\n"
                               "del sensor y por encima del vehículo más alto. Velocidad: roi_z_max_m (compartido con\n"
                               "«Prueba de paso»). Volumen: vol_roi_z_max_m.")
        self.c_zmax["vel"].editingFinished.connect(lambda: self._compartido_cambiado("roi_z_max_m"))

        def fan(c):
            w = _spin(1, 360, 1, 5, "°")
            w.setValue(360)
            if c == "vel":
                w.editingFinished.connect(lambda: self._compartido_cambiado("fan_angle_rad"))
            else:
                w.editingFinished.connect(self._recalcular)
            return w
        self.c_fan = fila_par("Abanico (apertura)", fan,
                              "fan_angle_rad: apertura del abanico de interés centrado en la vertical (360° = sin\n"
                              "filtro); quita el soporte y lo que está por encima del sensor. Para cubrir la ROI hasta\n"
                              "una distancia D de la vertical: apertura > 2·atan(D / H). Velocidad: compartido con\n"
                              "«Prueba de paso». Volumen: fan_angle_rad de volume_node (centro: fan_center_rad del yaml).")

        def pend_chk(c):
            w = QtWidgets.QCheckBox("la conozco")
            w.toggled.connect(self._recalcular)
            return w
        self.chk_pend = fila_par("", pend_chk,
                                 "Medida con nivel digital sobre el piso EN EL PLANO de ese LiDAR (velocidad: a lo largo\n"
                                 "del avance; volumen: de lado a lado). Solo separa la inclinación del sensor de la del piso.")

        def pend(c):
            w = _spin(-30, 30, 2, 0.1, "°")
            w.setEnabled(False)
            self.chk_pend[c].toggled.connect(w.setEnabled)
            w.valueChanged.connect(self._recalcular)
            return w
        self.c_pend = fila_par("Pendiente del piso", pend,
                               "Velocidad: + si el piso SUBE hacia +x (salida). Volumen: + si sube hacia +y (izquierda).")

        sep = QtWidgets.QLabel("<b>Relación entre los LiDAR</b> (sección volume_node)")
        f.addWidget(sep, fila, 0, 1, 3)
        fila += 1

        def simple(etiqueta, w, tip):
            nonlocal fila
            lb = QtWidgets.QLabel(etiqueta)
            lb.setToolTip(tip)
            w.setToolTip(tip)
            f.addWidget(lb, fila, 0, 1, 2)
            f.addWidget(w, fila, 2)
            fila += 1
            return w
        self.c_plano = simple("Plano transversal sobre x", _spin(-5, 5, 3, 0.01, "m"),
                              "vol_plane_offset_m: distancia, sobre el eje de avance x, del centro de rotación del LiDAR\n"
                              "de velocidad al plano de corte del transversal (+ hacia +x). Instalación actual: 0 cm\n"
                              "(los dos en el mismo x). El paso 4 la verifica.")
        self.c_yoff = simple("Sensor transversal: lateral", _spin(-5, 5, 3, 0.01, "m"),
                             "y_offset_m: posición lateral del LiDAR de volumen respecto al eje del carril\n"
                             "(+ a la izquierda de un vehículo que avanza hacia +x). Medir con cinta.")
        self.c_vely = simple("Plano de velocidad: lateral", _spin(-5, 5, 3, 0.01, "m"),
                             "vel_plane_y_m: posición lateral del plano del LiDAR de velocidad en el marco del carril.\n"
                             "El paso 4 la mide.")
        self.c_dur = simple("Duración de la captura", _spin(1, 9, 0, 1, "s"),
                            "Se ajusta la mediana de los escaneos de este tiempo (10 por segundo y por LiDAR)")
        self.c_dur.setValue(3)
        for w in (self.c_plano, self.c_yoff, self.c_vely):
            w.valueChanged.connect(self._recalcular)
        lp.addWidget(g)

        # 2 · piso vacío
        g = QtWidgets.QGroupBox("2 · Piso vacío (los dos a la vez)")
        f = QtWidgets.QVBoxLayout(g)
        t = QtWidgets.QLabel("Retira todo de los DOS planos de escaneo (personas incluidas) y no toques los soportes.")
        t.setWordWrap(True)
        t.setObjectName("suave")
        f.addWidget(t)
        self.b_piso = QtWidgets.QPushButton("Capturar piso vacío")
        self.b_piso.clicked.connect(lambda: self._capturar("vacio", self._marcados(), self._piso_capturado))
        f.addWidget(self.b_piso)
        self.barra = QtWidgets.QProgressBar()
        self.barra.setTextVisible(False)
        self.barra.setFixedHeight(6)
        f.addWidget(self.barra)
        self.lbl_res, self.lbl_avisos = {}, {}
        for l in LIDARS:
            self.lbl_res[l.clave] = QtWidgets.QLabel(f"<b>{l.nombre.capitalize()}:</b> sin captura.")
            self.lbl_res[l.clave].setWordWrap(True)
            self.lbl_res[l.clave].setTextInteractionFlags(QtCore.Qt.TextInteractionFlag.TextSelectableByMouse)
            self.lbl_avisos[l.clave] = QtWidgets.QLabel("")
            self.lbl_avisos[l.clave].setWordWrap(True)
            f.addWidget(self.lbl_res[l.clave])
            f.addWidget(self.lbl_avisos[l.clave])
        lp.addWidget(g)

        # 3 · orientación
        g = QtWidgets.QGroupBox("3 · Orientación (opcional)")
        f = QtWidgets.QVBoxLayout(g)
        t = QtWidgets.QLabel(
            "<b>a) Sentido de avance (velocidad).</b> Pon un objeto (caja ≥ 30 cm o una persona) a 1–2 m de la vertical "
            "del LiDAR de velocidad, sobre su plano, del lado desde donde LLEGAN los vehículos que se miden como "
            "positivos (+x). Para solo cambiar el signo mostrado basta «Invertir referencia» de Prueba de paso.")
        t.setWordWrap(True)
        t.setObjectName("suave")
        f.addWidget(t)
        self.b_dir = QtWidgets.QPushButton("Detectar sentido de avance")
        self.b_dir.clicked.connect(lambda: self._capturar("entrada", {"vel"}, self._sentido_capturado))
        f.addWidget(self.b_dir)
        self.lbl_dir = QtWidgets.QLabel("")
        self.lbl_dir.setWordWrap(True)
        f.addWidget(self.lbl_dir)
        t = QtWidgets.QLabel(
            "<b>b) Lado izquierdo (volumen).</b> Parado mirando hacia +x (hacia donde van los vehículos del punto a), pon "
            "el objeto a tu IZQUIERDA, a ~1 m de la vertical del LiDAR transversal y sobre su plano. Así +y queda a la "
            "izquierda del camión, como espera la reconstrucción.")
        t.setWordWrap(True)
        t.setObjectName("suave")
        f.addWidget(t)
        self.b_izq = QtWidgets.QPushButton("Detectar lado izquierdo")
        self.b_izq.clicked.connect(lambda: self._capturar("izquierda", {"vol"}, self._izquierda_capturada))
        f.addWidget(self.b_izq)
        self.lbl_izq = QtWidgets.QLabel("")
        self.lbl_izq.setWordWrap(True)
        f.addWidget(self.lbl_izq)
        lp.addWidget(g)

        # 4 · relación
        g = QtWidgets.QGroupBox("4 · Relación entre los LiDAR (opcional)")
        f = QtWidgets.QVBoxLayout(g)
        t = QtWidgets.QLabel("Pon un objeto delgado y alto (tubo, poste, persona de pie) justo en el CRUCE de los dos "
                             "planos: bajo el LiDAR transversal y sobre la línea del longitudinal. El de velocidad mide su "
                             "x (debe dar «Plano transversal sobre x»), el de volumen su y y los dos su altura.")
        t.setWordWrap(True)
        t.setObjectName("suave")
        f.addWidget(t)
        fila_b = QtWidgets.QHBoxLayout()
        self.b_cruce = QtWidgets.QPushButton("Medir el cruce")
        self.b_cruce.clicked.connect(lambda: self._capturar("cruce", {"vel", "vol"}, self._cruce_capturado))
        self.b_usar_cruce = QtWidgets.QPushButton("Usar lo medido")
        self.b_usar_cruce.setToolTip("Copia la x y la y medidas a «Plano transversal sobre x» y «Plano de velocidad: lateral»")
        self.b_usar_cruce.clicked.connect(self._usar_cruce)
        fila_b.addWidget(self.b_cruce)
        fila_b.addWidget(self.b_usar_cruce)
        f.addLayout(fila_b)
        self.lbl_cruce = QtWidgets.QLabel("")
        self.lbl_cruce.setWordWrap(True)
        f.addWidget(self.lbl_cruce)
        lp.addWidget(g)

        # 5 · aplicar
        g = QtWidgets.QGroupBox("5 · Aplicar")
        f = QtWidgets.QVBoxLayout(g)
        fila_b = QtWidgets.QHBoxLayout()
        self.b_aplicar = QtWidgets.QPushButton("Aplicar")
        self.b_aplicar.setToolTip("Usa los valores EFECTIVOS (H de la cinta, offset de rango, rayo hacia abajo, sentido y\n"
                                  "relación) en «Prueba de paso» (velocidad) y en «Volumen 3D» (los dos). El yaml no cambia.")
        self.b_aplicar.clicked.connect(lambda: self._aplicar())
        self.b_guardar = QtWidgets.QPushButton("Aplicar y guardar ficha")
        self.b_guardar.setToolTip(f"Aplica y guarda la ficha de calibración de los dos LiDAR en {self.carpeta}.\n"
                                  "Del yaml solo ofrece actualizar lo que mide el instalador (alturas con cinta y\n"
                                  "relación entre los LiDAR): offset y rotación son efectivos.")
        self.b_guardar.clicked.connect(self._guardar)
        fila_b.addWidget(self.b_aplicar)
        fila_b.addWidget(self.b_guardar)
        f.addLayout(fila_b)
        lp.addWidget(g)
        lp.addStretch(1)

        area = QtWidgets.QScrollArea()
        area.setWidgetResizable(True)
        area.setWidget(panel)
        area.setMinimumWidth(340)
        area.setHorizontalScrollBarPolicy(QtCore.Qt.ScrollBarPolicy.ScrollBarAlwaysOff)
        div.addWidget(area)
        div.setStretchFactor(0, 1)
        div.setSizes([640, 380])
        panel.setMinimumWidth(0)
        panel.setSizePolicy(QtWidgets.QSizePolicy.Policy.Ignored, QtWidgets.QSizePolicy.Policy.Preferred)
        raiz = QtWidgets.QVBoxLayout(self)
        raiz.setContentsMargins(0, 6, 0, 0)
        raiz.addWidget(div)
        self._habilitar()

    # ------------------------------------------------------------------ fuente
    def showEvent(self, ev):
        """Al entrar: valores nominales del yaml (la primera vez) y los compartidos con «Prueba de paso»."""
        super().showEvent(ev)
        if not self._iniciado:
            self._iniciado = True
            self._cargar_nominales()
        for n, (w, factor) in (("roi_z_max_m", (self.c_zmax["vel"], 1.0)),
                               ("fan_angle_rad", (self.c_fan["vel"], 180 / math.pi))):
            v = getattr(self.v.paso.params_form, n) * factor
            if not math.isclose(w.value(), round(v, w.decimals()), abs_tol=1e-9):
                w.blockSignals(True)
                w.setValue(v)
                w.blockSignals(False)
        self._abajo_txt()
        self._estado_fuente()
        self._t_vivo.start()

    def hideEvent(self, ev):
        super().hideEvent(ev)
        self._t_vivo.stop()

    def _cargar_nominales(self):
        for l in LIDARS:
            p = self._nominales(l.clave)
            self.c_abajo[l.clave].setValue(nadir_ros_deg(p))
            self.c_h[l.clave].setValue(p.mount_height_m - p.floor_z_m)
        wp = self._nominales("vol")
        self.c_zmax["vol"].setValue(wp.vol_roi_z_max_m)
        self.c_fan["vol"].setValue(math.degrees(wp.fan_angle_rad))
        for w, v in ((self.c_plano, wp.vol_plane_offset_m), (self.c_yoff, wp.y_offset_m), (self.c_vely, wp.vel_plane_y_m)):
            w.blockSignals(True)
            w.setValue(v)
            w.blockSignals(False)

    def _fuente_cambiada(self, i: int):
        if i == 1:
            if self.portico.encendido:
                self.portico.apagar()
            from falconscan_volume.sim import PorticoSimConfig
            c = PorticoSimConfig()
            self.c_h["vel"].setValue(c.vel_height)
            self.c_h["vol"].setValue(c.vol_height)
            self.v._log("Calibración con el SIMULADOR del pórtico: alturas = las del simulador (5 m). Sirve para "
                        "probar la rutina, no calibra los sensores.", "aviso")
        else:
            self._cargar_nominales()
        for e in self.est.values():
            e.__init__()
        self._limpiar_graficos()
        self._estado_fuente()

    def _encender(self):
        if self.portico.encendido:
            self.portico.apagar()
            return
        if self.v.leyendo and self.v.fuente.currentIndex() in (0, 3):
            QtWidgets.QMessageBox.information(self, "LiDAR en uso", "La barra superior está leyendo un LiDAR del pórtico. "
                                              "Detenla (F6) antes de encender los dos desde aquí.")
            return
        self.portico.encender()

    def _estado_fuente(self):
        sim = self._sim()
        self.b_encender.setVisible(not sim)
        if sim:
            self.lbl_fuente.setText(f"<span style='color:{AMBAR}'>Simulador: cada captura genera {N_SIM} escaneos "
                                    "por LiDAR con el simulador 3D (~1 s cada uno en la RPi).</span>")
            return
        on = self.portico.encendido
        self.b_encender.setText("■ Apagar LiDAR" if on else "▶ Encender LiDAR")
        self.b_encender.setObjectName("detener" if on else "iniciar")
        self.b_encender.style().polish(self.b_encender)
        partes = []
        for l in LIDARS:
            hz = self.portico.hz(l.clave)
            color = VERDE if hz > 5 else (AMBAR if on else TEXTO_SUAVE)
            partes.append(f"<span style='color:{color}'>● {l.clave} {l.ip}: "
                          f"{f'{hz:.1f} Hz' if hz else ('sin datos' if on else 'apagado')}</span>")
        self.lbl_fuente.setText(" &nbsp; ".join(partes))

    # ------------------------------------------------------------------ captura
    def _marcados(self) -> set[str]:
        return {c for c, w in self.chk_usar.items() if w.isChecked()}

    def _capturar(self, escena: str, que: set[str], al_terminar):
        if not que:
            QtWidgets.QMessageBox.information(self, "Calibración", "Marca al menos un LiDAR para calibrar.")
            return
        flujos = self._flujos()
        if self._sim():
            n = N_SIM
            for c in que:
                flujos[c].limpiar()
            self._sim_hilo = SimCalibracion(escena, que, flujos, n)
            self._sim_hilo.start()
            espera = ESPERA_SIM_S
        else:
            if not self.portico.encendido:
                QtWidgets.QMessageBox.information(self, "Calibración", "Enciende los LiDAR primero («▶ Encender LiDAR»).")
                return
            sin = [c for c in que if self.portico.hz(c) == 0]
            if len(sin) == len(que):
                QtWidgets.QMessageBox.warning(self, "Calibración", "No llegan escaneos de ningún LiDAR marcado.")
                return
            n = int(self.c_dur.value() * 10)
            espera = ESPERA_REAL_S
        inicio = {c: flujos[c].estado()[0] for c in que}
        self._captura = (n, inicio, time.monotonic(), que, al_terminar, espera)
        self.barra.setRange(0, n)
        self.barra.setValue(0)
        self._habilitar(capturando=True)
        self._t.start()

    def _sondear(self):
        n, inicio, t0, que, al_terminar, espera = self._captura
        flujos = self._flujos()
        nuevos = {c: flujos[c].estado()[0] - inicio[c] for c in que}
        self.barra.setValue(min(n, min(nuevos.values())))
        listos = {c for c in que if nuevos[c] >= n}
        if listos != que and time.monotonic() - t0 < espera:
            return
        self._t.stop()
        self._captura = None
        if not listos:
            self._habilitar()
            QtWidgets.QMessageBox.warning(self, "Calibración", "Dejaron de llegar escaneos; captura cancelada.")
            return
        for c in que - listos:
            self.v._log(f"Calibración: el LiDAR de {('velocidad' if c == 'vel' else 'volumen')} no completó la "
                        "captura; se usa solo el otro.", "aviso")
        capturas = {}
        for c in listos:
            escaneos = flujos[c].ultimos(n)
            escaneos = [s for s in escaneos if s.ranges.size == escaneos[-1].ranges.size]
            p = self._nominales(c)
            s, b = self._corr(c)
            pila = np.stack([(validate_ranges(e.ranges, max(p.range_min_m, e.range_min), min(p.range_max_m, e.range_max))
                              - b) * s for e in escaneos])
            capturas[c] = (pila, escaneos[-1].angles, (s, b))
        try:
            al_terminar(capturas)
        finally:
            self._habilitar()

    def _habilitar(self, *_, capturando: bool = False):
        cal = {c: e.res is not None and not e.res.failed for c, e in self.est.items()}
        for b in (self.b_piso, self.b_dir, self.b_izq, self.b_cruce, self.b_aplicar, self.b_guardar, self.b_usar_cruce):
            b.setEnabled(False)
        if capturando:
            return
        self.b_piso.setEnabled(True)
        self.b_dir.setEnabled(self.est["vel"].res is not None)
        self.b_izq.setEnabled(self.est["vol"].res is not None)
        self.b_cruce.setEnabled(cal["vel"] and cal["vol"])
        self.b_usar_cruce.setEnabled(self.est["vel"].cruce is not None or self.est["vol"].cruce is not None)
        algo = any(cal.values()) and not any(e.res is not None and e.res.failed for e in self.est.values())
        self.b_aplicar.setEnabled(algo)    # FALLA en alguno: solo volver a capturar
        self.b_guardar.setEnabled(algo)

    # ------------------------------------------------------------------ paso 2: piso
    def _piso_capturado(self, capturas):
        for c, (pila, ang, corr) in capturas.items():
            e = self.est[c]
            e.__init__(pila_vacio=pila, ang=ang, corr=corr)
            e.extra["params_antes"] = self.v.paso.params_form if c == "vel" else self._nominales("vol")
        self.lbl_dir.setText("")
        self.lbl_izq.setText("")
        self.lbl_cruce.setText("")
        self._recalcular()
        for c in capturas:
            r = self.est[c].res
            nombre = "velocidad" if c == "vel" else "volumen"
            if r is None:
                self.v._log(f"Calibración {nombre}: {self.est[c].error}", "error")
                continue
            self.v._log(f"Calibración {nombre}: offset {r.range_offset_m * 100:+.1f} cm ({r.offset_status}), desvío de "
                        f"la flecha {math.degrees(-r.arrow_deviation_rad):+.2f}° ({r.angle_status})"
                        + (f" · FALLA: {' '.join(r.failures)}" if r.failed else f" · {len(r.warnings)} avisos."),
                        "error" if r.failed else ("aviso" if r.warnings else "ok"))

    def _calcular(self, c: str):
        """Calibra la captura de ese LiDAR con los datos del formulario."""
        e = self.est[c]
        q = self._nominales(c)
        tol = self.v.paso.params_yaml           # tolerancias cal_*: mismas para los dos S2E
        d = e.dir_detectado or e.extra["params_antes"].angle_direction
        pend = math.radians(self.c_pend[c].value()) if self.chk_pend[c].isChecked() else None
        comunes = dict(nominal_down_rad=math.radians(nadir_ros_deg(q)), angle_direction=d, floor_slope_rad=pend,
                       offset_tol_ok_m=tol.cal_offset_tol_ok_m, offset_tol_max_m=tol.cal_offset_tol_max_m,
                       angle_tol_ok_rad=tol.cal_angle_tol_ok_rad, angle_tol_max_rad=tol.cal_angle_tol_max_rad)
        if c == "vel":
            return calibrate_mount(e.pila_vacio, e.ang, self.c_h[c].value(), math.radians(self.c_abajo[c].value()),
                                   roi_x_m=(q.roi_x_min_m - q.x_offset_m, q.roi_x_max_m - q.x_offset_m),
                                   roi_z_max_m=self.c_zmax[c].value(), object_h_threshold_m=q.object_h_threshold_m,
                                   fan_angle_rad=math.radians(self.c_fan[c].value()), fan_center_rad=self._fan(c)[1],
                                   coord_offset_m=q.x_offset_m, **comunes)
        y0 = self.c_yoff.value()                # la ROI lateral es del carril; la rutina trabaja desde el sensor
        return calibrate_mount(e.pila_vacio, e.ang, self.c_h[c].value(), math.radians(self.c_abajo[c].value()),
                               roi_x_m=(q.vol_roi_y_min_m - y0, q.vol_roi_y_max_m - y0),
                               roi_z_max_m=self.c_zmax[c].value(), object_h_threshold_m=q.vol_h_threshold_m,
                               fan_angle_rad=math.radians(self.c_fan[c].value()), fan_center_rad=self._fan(c)[1],
                               axis_name="y", coord_offset_m=y0, **comunes)

    def _fan(self, c: str) -> tuple[float, float]:
        """(apertura, centro) del abanico de ese LiDAR: apertura del formulario; centro de «Prueba de paso» o del yaml."""
        centro = self.v.paso.params_form.fan_center_rad if c == "vel" else self._nominales("vol").fan_center_rad
        return math.radians(self.c_fan[c].value()), centro

    def _desplazamiento(self, c: str) -> float:
        """Posición horizontal del sensor en el marco del carril: x_offset_m (velocidad) o y_offset_m (volumen)."""
        return self._nominales("vel").x_offset_m if c == "vel" else self.c_yoff.value()

    def _geometria(self, c: str) -> MountGeometry:
        r = self.est[c].res
        return MountGeometry(r.perp_height_m, r.angle_offset_rad, r.angle_direction, self._desplazamiento(c))

    def _recalcular(self, *_):
        for l in LIDARS:
            c = l.clave
            e = self.est[c]
            if e.pila_vacio is None:
                continue
            try:
                e.res, e.error = self._calcular(c), ""
            except ValueError as err:
                e.res, e.error = None, str(err)
            self._mostrar(c)
        self._habilitar()

    def _mostrar(self, c: str):
        e, l = self.est[c], (VEL if c == "vel" else VOL)
        titulo = f"<b>{l.nombre.capitalize()}</b> ({l.ip})"
        if e.res is None:
            self.lbl_res[c].setText(f"{titulo}: <span style='color:{ROJO}'>{e.error}</span>")
            self.lbl_avisos[c].setText("")
            return
        r = e.res
        color = {OK: VERDE, CORREGIDO: ACENTO, FALLA: ROJO}
        txt = {OK: "OK", CORREGIDO: "corregido", FALLA: "FALLA"}

        def marca(estado):
            return f"<b style='color:{color[estado]}'>{txt[estado]}</b>"
        abajo, nominal = math.degrees(r.down_angle_rad), math.degrees(r.nominal_down_rad)
        eje = "x" if c == "vel" else "y"
        off = self._desplazamiento(c)
        lineas = [
            titulo,
            f"Altura: cinta {r.height_m:.3f} m"
            + (f" → {r.perp_height_m:.3f} m perpendicular" if r.floor_slope_rad is not None else "")
            + f" · ve el piso a {r.seen_height_m:.3f} m · offset <b>{r.range_offset_m * 100:+.1f} cm</b> "
            f"<span style='color:{TEXTO_SUAVE}'>± {r.range_offset_sigma_m * 1e3:.1f} mm</span> {marca(r.offset_status)}",
            f"Rotación: flecha 0° desviada <b>{math.degrees(-r.arrow_deviation_rad):+.2f}°</b> "
            f"<span style='color:{TEXTO_SUAVE}'>± {math.degrees(r.down_angle_sigma_rad):.3f}°</span> {marca(r.angle_status)} "
            f"· rayo al piso {abajo:.2f}° ROS (nominal {nominal:.2f}°)",
            f"Piso: {eje} = {r.floor_x_range_m[0] + off:.2f} a {r.floor_x_range_m[1] + off:.2f} m · cobertura ROI "
            f"{r.roi_coverage * 100:.0f} % · planitud ±{r.roi_floor_dev_m * 1e3:.0f} mm · RMS {r.model_rms_m * 1e3:.1f} mm · "
            f"{r.fit.n_points} haces, {r.n_scans} escaneos",
        ]
        if r.plumb_angle_rad is not None:
            pl = math.degrees(r.plumb_angle_rad)
            lineas.append(f"Plomada: flecha a {(nominal - pl + 180) % 360 - 180:+.2f}° de la vertical; el piso aporta "
                          f"{math.degrees(r.floor_slope_rad):+.2f}°.")
        sentido = "+1 (normal)" if r.angle_direction > 0 else "−1 (espejado)"
        lineas.append(f"Sentido del ángulo: {sentido}" + (" (detectado)" if e.dir_detectado else " (el configurado)"))
        if self._sim():
            lineas.append(f"<span style='color:{AMBAR}'>Datos del SIMULADOR: no calibra el sensor real.</span>")
        self.lbl_res[c].setText("<br>".join(lineas))
        avisos = [f"<span style='color:{ROJO}'>✖ {w}</span>" for w in r.failures]
        avisos += [f"<span style='color:{AMBAR}'>⚠ {w}</span>" for w in r.warnings]
        if r.failed:
            avisos.append(f"<span style='color:{ROJO}'>Calibración fallida: corrige el montaje y vuelve a capturar.</span>")
        self.lbl_avisos[c].setText("<br>".join(avisos) or f"<span style='color:{VERDE}'>✔ Sin avisos.</span>")
        self._dibujar(c)

    @staticmethod
    def _mediana(pila):
        return np.nanmedian(pila, axis=0) if pila.shape[0] > 1 else pila[0]

    def _roi(self, c: str) -> tuple[float, float, float]:
        q = self._nominales(c)
        if c == "vel":
            return q.roi_x_min_m, q.roi_x_max_m, self.c_zmax[c].value()
        return q.vol_roi_y_min_m, q.vol_roi_y_max_m, self.c_zmax[c].value()

    def _dibujar(self, c: str):
        e, it = self.est[c], self.items[c]
        r = e.res
        g = self._geometria(c)
        x, z = polar_to_xz(self._mediana(e.pila_vacio) - r.range_offset_m, e.ang, g)
        ok = np.isfinite(x) & (z > -1.0) & (z < r.perp_height_m + 1.0)
        x0, x1, zmax = self._roi(c)
        fan = self._fan(c)
        dentro = fan_mask(e.ang, g, *fan)
        it["abanico"].setData(*bordes_abanico(g.x_offset_m, r.perp_height_m, *fan, 1.5 * max(r.perp_height_m, x1 - x0)))
        piso = ok & r.fit.inliers
        alto = ok & dentro & ~piso & (x >= x0) & (x < x1) & (z > 0.10) & (z < zmax)
        otros = ok & ~piso & ~alto
        it["piso"].setData(x[piso], z[piso])
        it["alto"].setData(x[alto], z[alto])
        it["otros"].setData(x[otros], z[otros])
        it["sensor"].setData([g.x_offset_m], [r.perp_height_m])
        it["roi"].setRect(x0, 0.0, x1 - x0, zmax)
        if r.plumb_angle_rad is not None:
            xp, zp = polar_to_xz(np.array([0.0, r.perp_height_m * 1.05]), np.full(2, r.plumb_angle_rad), g)
            it["plomada"].setData(xp, zp)
        else:
            it["plomada"].setData([], [])
        it["leyenda"].setText("verde: piso · rojo: algo en la ROI · gris: resto"
                              + (" · punteado: abanico" if self.c_fan[c].value() < 360 else ""))
        it["leyenda"].setPos(x0, -0.15)
        cruce = self.c_plano.value() if c == "vel" else self.c_vely.value()
        it["cruce"].setPos(cruce)
        it["cruce"].show()
        ri = self.res_items[c]
        ri["puntos"].setData(x[piso], z[piso] * 1e3)
        if piso.any():
            celdas = np.arange(np.floor(x[piso].min() * 4) / 4, x[piso].max() + 0.25, 0.25)
            med = [np.median(z[piso][(x[piso] >= a) & (x[piso] < a + 0.25)]) * 1e3
                   if ((x[piso] >= a) & (x[piso] < a + 0.25)).any() else np.nan for a in celdas]
            ri["mediana"].setData(celdas + 0.125, np.asarray(med, dtype=float))

    def _limpiar_graficos(self):
        for c in self.items:
            for k in ("piso", "alto", "otros", "sensor"):
                self.items[c][k].setData([], [])
            self.items[c]["cruce"].hide()
            for k in self.res_items[c]:
                self.res_items[c][k].setData([], [])
            self.lbl_res[c].setText(f"<b>{(VEL if c == 'vel' else VOL).nombre.capitalize()}:</b> sin captura.")
            self.lbl_avisos[c].setText("")
        self._habilitar()

    def _vivo(self):
        """Antes de capturar: el último escaneo de cada LiDAR con la configuración del formulario."""
        if not self._sim():
            self._estado_fuente()
        flujos = self._flujos()
        for l in LIDARS:
            c = l.clave
            if self.est[c].res is not None:
                continue
            _, s = flujos[c].estado()
            if s is None:
                continue
            p = self._nominales(c)
            sc, b = self._corr(c)
            r = (validate_ranges(s.ranges, max(p.range_min_m, s.range_min), min(p.range_max_m, s.range_max)) - b) * sc
            d = p.angle_direction
            g = MountGeometry(self.c_h[c].value(), offset_desde_nadir(self.c_abajo[c].value(), d), d,
                              self._desplazamiento(c))
            x, z = polar_to_xz(r, s.angles, g)
            ok = np.isfinite(x) & (z > -2.0) & (z < g.height_m + 1.0)
            it = self.items[c]
            it["otros"].setData(x[ok], z[ok])
            it["sensor"].setData([g.x_offset_m], [g.height_m])
            x0, x1, zmax = self._roi(c)
            it["roi"].setRect(x0, 0.0, x1 - x0, zmax)
            it["abanico"].setData(*bordes_abanico(g.x_offset_m, g.height_m, *self._fan(c), 1.5 * max(g.height_m, x1 - x0)))
            it["leyenda"].setText("En vivo, configuración actual: el piso debería verse en z ≈ 0")
            it["leyenda"].setPos(x0, -0.15)

    def _abajo_txt(self, *_):
        for c, w in self.lbl_abajo.items():
            w.setText(f"= {ros_a_sensor(self.c_abajo[c].value()):+.1f}° en la marca")

    def _compartido_cambiado(self, nombre: str):
        """roi_z_max_m y fan_angle_rad del LiDAR de velocidad: mismo campo que «Prueba de paso»."""
        w = self.c_zmax["vel"] if nombre == "roi_z_max_m" else self.c_fan["vel"]
        w_paso = self.v.paso.campos[nombre][0]
        if math.isclose(w_paso.value(), w.value(), abs_tol=1e-9):
            return
        w_paso.setValue(w.value())
        self.v.paso.aplicar(silencioso=True)
        self.v._log(f"{nombre} = {w.value():g} {w.suffix().strip()} (aplicado en Prueba de paso; sin guardar).", "aviso")
        self._recalcular()

    # ------------------------------------------------------------------ paso 3: orientación
    def _sentido_capturado(self, capturas):
        self._orientacion(capturas, "vel", entry_side=-1, etiqueta=self.lbl_dir,
                          que="sentido de avance", lado="del lado de entrada (x &lt; 0)")

    def _izquierda_capturada(self, capturas):
        self._orientacion(capturas, "vol", entry_side=+1, etiqueta=self.lbl_izq,
                          que="lado izquierdo", lado="a la izquierda (y &gt; 0)")

    def _orientacion(self, capturas, c, entry_side, etiqueta, que, lado):
        e = self.est[c]
        if c not in capturas or e.res is None or capturas[c][0].shape[1] != e.pila_vacio.shape[1]:
            return
        d = detect_direction(e.pila_vacio, capturas[c][0], e.ang, e.res.down_angle_rad, entry_side=entry_side)
        if d is None:
            etiqueta.setText(f"<span style='color:{ROJO}'>No veo ningún objeto nuevo respecto al piso vacío. "
                             "Acércalo a la vertical del sensor, ponlo sobre su plano o usa uno más alto.</span>")
            return
        sentido, xm, n = d
        e.dir_detectado = sentido
        cambio = sentido != e.extra["params_antes"].angle_direction
        etiqueta.setText(f"Objeto detectado ({n} rayos, a {abs(xm):.2f} m de la vertical): queda {lado} con "
                         f"angle_direction = <b>{'+1 (normal)' if sentido > 0 else '−1 (espejado)'}</b>"
                         + (f" <span style='color:{AMBAR}'>(cambia respecto al configurado)</span>" if cambio else
                            f" <span style='color:{VERDE}'>(coincide con el configurado)</span>")
                         + ". Retira el objeto.")
        self.v._log(f"Calibración: {que} → angle_direction = {sentido:+d}.", "aviso" if cambio else "ok")
        self._recalcular()

    # ------------------------------------------------------------------ paso 4: relación
    def _cruce_capturado(self, capturas):
        for c in ("vel", "vol"):
            e = self.est[c]
            e.cruce = None
            if c in capturas and e.res is not None and capturas[c][0].shape[1] == e.pila_vacio.shape[1]:
                e.cruce = object_position(e.pila_vacio, capturas[c][0], e.ang, self._geometria(c),
                                          range_offset_m=e.res.range_offset_m)
        cv, cw = self.est["vel"].cruce, self.est["vol"].cruce
        lineas = []

        def comparar(medido, config, nombre):
            dif = medido - config
            color = VERDE if abs(dif) <= TOL_RELACION_M else AMBAR
            return (f"{nombre}: medido <b>{medido:+.3f} m</b>, configurado {config:+.3f} m "
                    f"<span style='color:{color}'>({dif * 100:+.1f} cm)</span>")
        if cv is None:
            lineas.append(f"<span style='color:{ROJO}'>El LiDAR de velocidad no ve el objeto: ponlo sobre su plano.</span>")
        else:
            lineas.append(comparar(cv[0], self.c_plano.value(), "Plano transversal sobre x (vol_plane_offset_m)"))
        if cw is None:
            lineas.append(f"<span style='color:{ROJO}'>El LiDAR transversal no ve el objeto: ponlo bajo su plano.</span>")
        else:
            lineas.append(comparar(cw[0], self.c_vely.value(), "Plano de velocidad lateral (vel_plane_y_m)"))
        if cv is not None and cw is not None:
            dh = cv[1] - cw[1]
            color = VERDE if abs(dh) <= TOL_RELACION_M else AMBAR
            lineas.append(f"Altura del objeto: velocidad {cv[1]:.3f} m · volumen {cw[1]:.3f} m "
                          f"<span style='color:{color}'>({dh * 100:+.1f} cm)</span>: control cruzado de las dos "
                          "calibraciones de altura (un poste inclinado o ancho también la mueve).")
        self.lbl_cruce.setText("<br>".join(lineas))
        self.v._log("Calibración: cruce de los planos medido "
                    + (f"(x = {cv[0]:+.3f} m" if cv else "(x = —") + (f", y = {cw[0]:+.3f} m)." if cw else ", y = —)."),
                    "ok" if cv and cw else "aviso")

    def _usar_cruce(self):
        cv, cw = self.est["vel"].cruce, self.est["vol"].cruce
        if cv is not None:
            self.c_plano.setValue(round(cv[0], 3))
        if cw is not None:
            self.c_vely.setValue(round(cw[0], 3))
        self.v._log("Relación entre los LiDAR: se usan los valores medidos en el cruce (sin guardar).", "aviso")

    # ------------------------------------------------------------------ paso 5: aplicar / guardar
    def _efectivos(self, c: str) -> dict | None:
        """Valores EFECTIVOS de ese LiDAR (H de la cinta, offset b sumado a su corrección, rayo calibrado)."""
        e = self.est[c]
        r = e.res
        if r is None or r.failed:
            return None
        s, b = e.corr
        q = self._nominales(c)
        out = {"mount_height_m": r.height_m + q.floor_z_m, "angle_offset_rad": r.angle_offset_rad,
               "angle_direction": r.angle_direction, "range_bias_m": r.range_bias_for(b, s)}
        if c == "vol":
            out.update(y_offset_m=self.c_yoff.value(), vol_plane_offset_m=self.c_plano.value(),
                       vel_plane_y_m=self.c_vely.value(), fan_angle_rad=math.radians(self.c_fan[c].value()))
        return out

    def _aplicar(self, silencioso=False) -> bool:
        aplicados = []
        ev = self._efectivos("vel")
        if ev is not None:
            paso = self.v.paso
            r = self.est["vel"].res
            paso.campos["mount_height_m"][0].setValue(ev["mount_height_m"])
            paso.c_nadir.setValue(math.degrees(r.down_angle_rad))
            paso.c_dir.setCurrentIndex(0 if r.angle_direction > 0 else 1)
            paso.fijar_calibracion({"range_bias_m": ev["range_bias_m"]})
            self.v.vol.fijar_calibracion("vel", ev)
            aplicados.append(f"velocidad (H {r.height_m:.3f} m, offset {r.range_offset_m * 100:+.1f} cm, "
                             f"abajo {math.degrees(r.down_angle_rad):.2f}°, sentido {r.angle_direction:+d})")
        ew = self._efectivos("vol")
        if ew is not None:
            r = self.est["vol"].res
            self.v.vol.fijar_calibracion("vol", ew)
            aplicados.append(f"volumen (H {r.height_m:.3f} m, offset {r.range_offset_m * 100:+.1f} cm, "
                             f"abajo {math.degrees(r.down_angle_rad):.2f}°, sentido {r.angle_direction:+d}, "
                             f"plano en x = {ew['vol_plane_offset_m']:+.3f} m)")
        if aplicados and not silencioso:
            self.v._log("Calibración aplicada (efectiva, el yaml no cambia): " + "; ".join(aplicados) + ".", "ok")
        return bool(aplicados)

    def _guardar(self):
        """Ficha de los dos LiDAR. Del yaml solo se ofrece guardar lo que mide el instalador."""
        if not self._aplicar(silencioso=True):
            return
        ruta_yaml = self.v.paso.yaml_ruta
        cambios = []       # (nodo, params nuevos, nombres)
        pv = self.v.paso.params_yaml
        ev = self._efectivos("vel")
        if ev is not None and not math.isclose(ev["mount_height_m"], pv.mount_height_m, abs_tol=5e-4):
            cambios.append(("velocity_node", replace(pv, mount_height_m=ev["mount_height_m"]), ["mount_height_m"]))
        ew = self._efectivos("vol")
        if ew is not None:
            pw = self._nominales("vol")
            nuevos = {k: ew[k] for k in ("mount_height_m", "y_offset_m", "vol_plane_offset_m", "vel_plane_y_m",
                                         "fan_angle_rad")
                      if not math.isclose(ew[k], getattr(pw, k), abs_tol=5e-4)}
            if nuevos:
                cambios.append(("volume_node", replace(pw, **nuevos), list(nuevos)))
        if cambios:
            lista = "\n".join(f"  {nodo}: " + ", ".join(f"{n} = {getattr(p, n):.3f}" for n in nombres)
                              for nodo, p, nombres in cambios)
            r = QtWidgets.QMessageBox.question(
                self, "Guardar en el yaml",
                f"Valores del instalador distintos del yaml:\n{lista}\n\n¿Guardarlos en {ruta_yaml}?\n\n"
                "El offset y la rotación calibrados NO se guardan: son efectivos y se recalculan en cada calibración.")
            if r == QtWidgets.QMessageBox.StandardButton.Yes:
                for nodo, p, nombres in cambios:
                    escritos = guardar_params(ruta_yaml, p, nombres, nodo)
                    self.v._log(f"falconscan.yaml ({nodo}): {', '.join(escritos) or 'sin cambios'}", "ok")
                self.v.paso.params_yaml = cargar_params(ruta_yaml)
        self.carpeta.mkdir(parents=True, exist_ok=True)
        ruta = self.carpeta / f"calibracion_{dt.datetime.now():%Y%m%d_%H%M%S}.json"
        ficha = {"fecha": dt.datetime.now().isoformat(timespec="seconds"),
                 "fuente": "simulador" if self._sim() else "LiDAR reales",
                 "relacion": {"vol_plane_offset_m": self.c_plano.value(), "y_offset_m": self.c_yoff.value(),
                              "vel_plane_y_m": self.c_vely.value(),
                              "cruce_x_medido_m": None if self.est["vel"].cruce is None else self.est["vel"].cruce[0],
                              "cruce_y_medido_m": None if self.est["vol"].cruce is None else self.est["vol"].cruce[0],
                              "cruce_altura_m": [None if self.est[c].cruce is None else self.est[c].cruce[1]
                                                 for c in ("vel", "vol")]},
                 "lidars": {}}
        deg = math.degrees
        for l in LIDARS:
            e, r = self.est[l.clave], self.est[l.clave].res
            if r is None:
                continue
            s, b = e.corr
            ficha["lidars"][l.clave] = {
                "ip": l.ip, "nombre": l.nombre,
                "entradas": {"mount_height_m_cinta": r.height_m, "rayo_abajo_aprox_ros_deg": self.c_abajo[l.clave].value(),
                             "rayo_abajo_nominal_ros_deg": deg(r.nominal_down_rad), "escaneos": r.n_scans,
                             "pendiente_piso_deg": None if r.floor_slope_rad is None else deg(r.floor_slope_rad),
                             "range_bias_m": b, "range_scale": s, "altura_max_m": self.c_zmax[l.clave].value(),
                             "abanico_deg": self.c_fan[l.clave].value(),
                             "abanico_centro_deg": math.degrees(self._fan(l.clave)[1])},
                "resultado": {"offset_rango_m": r.range_offset_m, "offset_sigma_m": r.range_offset_sigma_m,
                              "offset_estado": r.offset_status, "altura_vista_m": r.seen_height_m,
                              "range_bias_efectivo_m": r.range_bias_for(b, s),
                              "rayo_abajo_ros_deg": deg(r.down_angle_rad), "rayo_sigma_deg": deg(r.down_angle_sigma_rad),
                              "desvio_flecha_marca_deg": deg(-r.arrow_deviation_rad), "rotacion_estado": r.angle_status,
                              "angle_offset_efectivo_rad": r.angle_offset_rad, "angle_direction": r.angle_direction,
                              "sentido_detectado": e.dir_detectado is not None, "rms_modelo_m": r.model_rms_m,
                              "puntos_piso": r.fit.n_points, "cobertura_roi": r.roi_coverage,
                              "planitud_roi_m": r.roi_floor_dev_m, "fraccion_movil": r.moving_fraction},
                "avisos": list(r.warnings),
                "escaneo_mediana": {"angulos_rad": [round(float(a), 6) for a in e.ang],
                                    "rangos_m": [None if not np.isfinite(v) else round(float(v), 5)
                                                 for v in self._mediana(e.pila_vacio)]},
            }
        ruta.write_text(json.dumps(ficha, ensure_ascii=False, indent=1))
        self.v._log(f"Ficha de calibración (los dos LiDAR) guardada en {ruta}", "ok")

    def refrescar(self, _ultimo=None):
        """Compatibilidad: la vista en vivo ahora la lleva el temporizador propio de la pestaña."""
