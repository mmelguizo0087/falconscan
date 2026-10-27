"""Pestaña «Calibración» del visor: rutina de calibración del montaje con el piso vacío.

Pasos: (1) datos del montaje (altura con cinta y, si se conoce, pendiente del piso),
(2) captura del piso vacío -> altura perpendicular, rayo hacia abajo e inclinación compensada,
(3) opcional: sentido de avance con un objeto del lado de entrada, (4) aplicar / guardar en
falconscan.yaml con una ficha de calibración. El cálculo está en core/calibration.py.
"""
from __future__ import annotations

import datetime as dt
import json
import math
import time
from dataclasses import replace
from pathlib import Path

import numpy as np
import pyqtgraph as pg
from pyqtgraph.Qt import QtCore, QtWidgets

from comun import ACENTO, AMBAR, ROJO, TEXTO_SUAVE, VERDE, bordes_abanico, ros_a_sensor, titulo_grafico
from paso import _spin, cargar_params, guardar_params, nadir_ros_deg, offset_desde_nadir

from falconscan_velocity.core import (MountGeometry, calibrate_mount, detect_direction, fan_mask, polar_to_xz,
                                     validate_ranges)
from falconscan_velocity.core.calibration import CORREGIDO, FALLA, OK

ESPERA_MAX_S = 15.0     # sin completar la captura en este tiempo -> se cancela


class PanelCalibracion(QtWidgets.QWidget):
    """Contenido de la pestaña «Calibración». Escribe sus resultados en el formulario de PanelPaso."""

    def __init__(self, ventana, carpeta_datos: Path):
        super().__init__()
        self.v = ventana
        self.carpeta = carpeta_datos / "calibraciones"
        self.res = None                     # MountCalibration de la última captura
        self.pila_vacio = None              # rangos corregidos del piso vacío (N × M)
        self.ang = None
        self.dir_detectado = None           # angle_direction del paso 3 (None = sin detectar)
        self.params_antes = None            # VelocityParams con que se capturó (para comparar)
        self.p_captura = None               # parámetros efectivos con que se corrigieron los rangos
        self._captura = None                # (n, recibidos al empezar, t0, al_terminar)
        self._iniciado = False
        self._t = QtCore.QTimer(self, interval=100, timeout=self._sondear)
        self._construir()

    # ------------------------------------------------------------------ construcción
    def _construir(self):
        div = QtWidgets.QSplitter(QtCore.Qt.Orientation.Horizontal)
        izq = QtWidgets.QSplitter(QtCore.Qt.Orientation.Vertical)

        self.g_xz = pg.PlotWidget()
        self.g_xz.setTitle(titulo_grafico("Plano de escaneo"))
        self.g_xz.setAspectLocked(True)
        self.g_xz.showGrid(x=True, y=True, alpha=0.12)
        self.g_xz.setLabel("bottom", "x: avance →  [m]")
        self.g_xz.setLabel("left", "z: altura sobre el piso [m]")
        self.roi_rect = QtWidgets.QGraphicsRectItem()
        self.roi_rect.setPen(pg.mkPen(ACENTO, width=1, style=QtCore.Qt.PenStyle.DashLine))
        self.roi_rect.setBrush(pg.mkBrush(47, 184, 196, 18))
        self.g_xz.addItem(self.roi_rect)
        self.g_xz.addItem(pg.InfiniteLine(angle=0, pen=pg.mkPen("#8b98a5", width=2)))
        self.p_otros = pg.ScatterPlotItem(size=3, pen=None, brush=pg.mkBrush("#c9d1d9"))
        self.p_piso = pg.ScatterPlotItem(size=3, pen=None, brush=pg.mkBrush(VERDE))
        self.p_alto = pg.ScatterPlotItem(size=4, pen=None, brush=pg.mkBrush(ROJO))
        self.sensor = pg.ScatterPlotItem(size=14, brush=pg.mkBrush(ACENTO), pen=pg.mkPen("w"))
        self.plomada = pg.PlotDataItem(pen=pg.mkPen(AMBAR, width=1, style=QtCore.Qt.PenStyle.DashLine))
        self.abanico = pg.PlotDataItem(pen=pg.mkPen(ACENTO, width=1, style=QtCore.Qt.PenStyle.DotLine), connect="finite")
        for it in (self.abanico, self.p_otros, self.p_piso, self.p_alto, self.plomada, self.sensor):
            self.g_xz.addItem(it)
        self.leyenda = pg.TextItem("", color=TEXTO_SUAVE, anchor=(0, 0))
        self.g_xz.addItem(self.leyenda)
        izq.addWidget(self.g_xz)

        self.g_res = pg.PlotWidget()
        self.g_res.setTitle(titulo_grafico("Planitud del piso (residuo de la recta)"))
        self.g_res.showGrid(x=True, y=True, alpha=0.15)
        self.g_res.setLabel("left", "z [mm]")
        self.g_res.setLabel("bottom", "x [m]")
        self.g_res.setXLink(self.g_xz)
        self.r_puntos = pg.ScatterPlotItem(size=3, pen=None, brush=pg.mkBrush(VERDE))
        self.r_mediana = pg.PlotDataItem(pen=pg.mkPen(ACENTO, width=2), connect="finite")
        self.g_res.addItem(self.r_puntos)
        self.g_res.addItem(self.r_mediana)
        self.g_res.addItem(pg.InfiniteLine(angle=0, pen=pg.mkPen("#8b98a5")))
        izq.addWidget(self.g_res)
        izq.setSizes([320, 150])
        div.addWidget(izq)

        panel = QtWidgets.QWidget()
        lp = QtWidgets.QVBoxLayout(panel)
        lp.setContentsMargins(4, 0, 0, 0)

        # 1 · datos del montaje
        g = QtWidgets.QGroupBox("1 · Datos del montaje")
        f = QtWidgets.QFormLayout(g)
        self.c_h = _spin(0.1, 30, 3, 0.01, "m")
        self.c_h.setValue(3.0)
        self.c_h.setToolTip("mount_height_m: altura VERTICAL (con plomada) desde el piso hasta el centro óptico\n"
                            "del LiDAR (el eje de giro del S2E, no el borde de la carcasa).\n"
                            "Se toma como verdadera: la calibración no la cambia y atribuye la diferencia\n"
                            "con lo que ve el sensor al offset de rango.")
        f.addRow("Altura medida con cinta", self.c_h)
        self.c_zmax = _spin(0.5, 30, 2, 0.1, "m")
        self.c_zmax.setValue(4.5)
        self.c_zmax.setToolTip("roi_z_max_m: los puntos más altos que esto sobre el piso se descartan (soporte,\n"
                               "estructura, polvo o insectos junto al sensor). Ponlo por debajo del sensor y por\n"
                               "encima del vehículo más alto. Es el mismo campo de «Prueba de paso» (se aplica\n"
                               "allí al cambiarlo); para dejarlo en falconscan.yaml usa «Guardar en yaml» allí.")
        f.addRow("Altura máx. considerada", self.c_zmax)
        self.c_fan = _spin(1, 360, 1, 5, "°")
        self.c_fan.setValue(360)
        self.c_fan.setToolTip("fan_angle_rad: apertura total del abanico de interés, centrado en la vertical hacia\n"
                              "abajo. Los rayos de fuera (soporte, techo, paredes) no se usan. 360° = sin filtro.\n"
                              "Para ver el piso hasta |x| = X hace falta más de 2·atan(X / H). Es el mismo campo\n"
                              "de «Prueba de paso» (el centro se ajusta allí); para dejarlo en falconscan.yaml usa\n"
                              "«Guardar en yaml» allí.")
        f.addRow("Abanico (apertura)", self.c_fan)
        # Campos compartidos con «Prueba de paso»: nombre -> (widget, factor de visualización, etiqueta)
        self.compartidos = {"roi_z_max_m": (self.c_zmax, 1.0, "Altura máx. considerada"),
                            "fan_angle_rad": (self.c_fan, 180 / math.pi, "Abanico (apertura)")}
        for n in self.compartidos:
            self.compartidos[n][0].editingFinished.connect(lambda n=n: self._compartido_cambiado(n))
        self.chk_pend = QtWidgets.QCheckBox("Conozco la pendiente del piso")
        self.chk_pend.setToolTip("Medida con nivel digital o inclinómetro sobre el piso, a lo largo del avance.\n"
                                 "Solo hace falta para separar la inclinación del sensor de la del piso:\n"
                                 "la compensación para medir no la necesita.")
        f.addRow(self.chk_pend)
        self.c_pend = _spin(-30, 30, 2, 0.1, "°")
        self.c_pend.setToolTip("Positivo: el piso SUBE hacia la salida (sentido de avance, +x)")
        self.c_pend.setEnabled(False)
        self.chk_pend.toggled.connect(self.c_pend.setEnabled)
        self.chk_pend.toggled.connect(self._mostrar)
        self.c_pend.valueChanged.connect(self._mostrar)
        f.addRow("Pendiente del piso", self.c_pend)
        self.c_abajo = _spin(0, 359.99, 2, 1.0, "°")
        self.c_abajo.setWrapping(True)
        self.c_abajo.setToolTip("Ángulo ROS aproximado del rayo que mira al piso (±25°), solo para buscar el piso.\n"
                                "Por defecto el nominal del yaml: flecha 0° del S2E al piso = ROS 180°.\n"
                                "El desvío de la flecha se mide siempre contra ese nominal.")
        self.c_abajo.valueChanged.connect(self._abajo_txt)
        f.addRow("Rayo hacia abajo aprox. (ROS)", self.c_abajo)
        self.lbl_abajo = QtWidgets.QLabel()
        self.lbl_abajo.setObjectName("suave")
        self.lbl_abajo.setWordWrap(True)
        f.addRow("", self.lbl_abajo)
        self.c_dur = _spin(1, 9, 0, 1, "s")
        self.c_dur.setValue(3)
        self.c_dur.setToolTip("Se ajusta la mediana de los escaneos de este tiempo (10 por segundo)")
        f.addRow("Duración de la captura", self.c_dur)
        lp.addWidget(g)

        # 2 · piso vacío
        g = QtWidgets.QGroupBox("2 · Piso vacío")
        f = QtWidgets.QVBoxLayout(g)
        t = QtWidgets.QLabel("Retira todo del plano de escaneo (personas incluidas) y no toques el soporte.")
        t.setWordWrap(True)
        t.setObjectName("suave")
        f.addWidget(t)
        self.b_piso = QtWidgets.QPushButton("Capturar piso vacío")
        self.b_piso.clicked.connect(lambda: self._capturar(self._piso_capturado))
        f.addWidget(self.b_piso)
        self.barra = QtWidgets.QProgressBar()
        self.barra.setTextVisible(False)
        self.barra.setFixedHeight(6)
        f.addWidget(self.barra)
        self.lbl_res = QtWidgets.QLabel("Sin captura.")
        self.lbl_res.setWordWrap(True)
        self.lbl_res.setTextInteractionFlags(QtCore.Qt.TextInteractionFlag.TextSelectableByMouse)
        f.addWidget(self.lbl_res)
        self.lbl_avisos = QtWidgets.QLabel("")
        self.lbl_avisos.setWordWrap(True)
        f.addWidget(self.lbl_avisos)
        lp.addWidget(g)

        # 3 · sentido de avance
        g = QtWidgets.QGroupBox("3 · Sentido de avance (opcional)")
        f = QtWidgets.QVBoxLayout(g)
        t = QtWidgets.QLabel("Orienta el eje x: después de capturar el piso, pon un objeto (caja de ≥30 cm o una "
                             "persona) a 1–2 m de la vertical del sensor, del lado desde donde llegan los vehículos "
                             "que quieres medir como POSITIVOS. Para solo cambiar el signo basta la casilla "
                             "«Invertir referencia» de Prueba de paso.")
        t.setWordWrap(True)
        t.setObjectName("suave")
        f.addWidget(t)
        self.b_dir = QtWidgets.QPushButton("Detectar sentido")
        self.b_dir.setToolTip("Fija angle_direction para que ese lado quede en x negativo (los vehículos que\n"
                              "salen de él van hacia +x). Con la referencia sin invertir, son los positivos.")
        self.b_dir.setEnabled(False)
        self.b_dir.clicked.connect(lambda: self._capturar(self._objeto_capturado))
        f.addWidget(self.b_dir)
        self.lbl_dir = QtWidgets.QLabel("")
        self.lbl_dir.setWordWrap(True)
        f.addWidget(self.lbl_dir)
        lp.addWidget(g)

        # 4 · aplicar
        g = QtWidgets.QGroupBox("4 · Aplicar")
        f = QtWidgets.QVBoxLayout(g)
        fila = QtWidgets.QHBoxLayout()
        self.b_aplicar = QtWidgets.QPushButton("Aplicar")
        self.b_aplicar.setToolTip("Usa en «Prueba de paso» los valores EFECTIVOS: H de la cinta, offset de rango\n"
                                  "y rayo hacia abajo calibrados, y el sentido. El yaml no cambia.")
        self.b_aplicar.clicked.connect(lambda: self._aplicar())
        self.b_guardar = QtWidgets.QPushButton("Aplicar y guardar ficha")
        self.b_guardar.setToolTip(f"Aplica y guarda la ficha de calibración en {self.carpeta}\n"
                                  "(entradas, resultados, avisos y la mediana del piso). Del yaml solo puede\n"
                                  "actualizar mount_height_m si la cinta cambió: offset y rotación son efectivos.")
        self.b_guardar.clicked.connect(self._guardar)
        for b in (self.b_aplicar, self.b_guardar):
            b.setEnabled(False)
            fila.addWidget(b)
        f.addLayout(fila)
        lp.addWidget(g)
        lp.addStretch(1)

        area = QtWidgets.QScrollArea()
        area.setWidgetResizable(True)
        area.setWidget(panel)
        area.setMinimumWidth(330)
        area.setHorizontalScrollBarPolicy(QtCore.Qt.ScrollBarPolicy.ScrollBarAlwaysOff)
        div.addWidget(area)
        div.setStretchFactor(0, 1)
        div.setSizes([680, 340])
        # Sin barra horizontal: el panel nunca debe ser más ancho que el área (el texto se ajusta)
        panel.setMinimumWidth(0)
        panel.setSizePolicy(QtWidgets.QSizePolicy.Policy.Ignored, QtWidgets.QSizePolicy.Policy.Preferred)
        raiz = QtWidgets.QVBoxLayout(self)
        raiz.setContentsMargins(0, 6, 0, 0)
        raiz.addWidget(div)

    def showEvent(self, ev):
        """Al entrar a la pestaña, partir de la configuración vigente de «Prueba de paso»."""
        super().showEvent(ev)
        if not self._iniciado:
            self._iniciado = True
            p = self.v.paso.params_yaml                    # nominales
            self.c_abajo.setValue(nadir_ros_deg(p))
            self.c_h.setValue(p.mount_height_m - p.floor_z_m)
        cambio = False
        for n, (w, factor, _) in self.compartidos.items():      # pudieron cambiar en «Prueba de paso»
            v = getattr(self.v.paso.params_form, n) * factor
            if not math.isclose(w.value(), round(v, w.decimals()), abs_tol=1e-9):
                w.blockSignals(True)
                w.setValue(v)
                w.blockSignals(False)
                cambio = True
        if cambio:
            self._mostrar()
        self._abajo_txt()

    def _compartido_cambiado(self, nombre):
        """Mismo parámetro que «Prueba de paso»: se aplica allí (reinicia la máquina) y se recalcula."""
        paso = self.v.paso
        w, _, etiqueta = self.compartidos[nombre]
        w_paso = paso.campos[nombre][0]
        if math.isclose(w_paso.value(), w.value(), abs_tol=1e-9):
            return
        w_paso.setValue(w.value())
        paso.aplicar(silencioso=True)
        self.v._log(f"{etiqueta} = {w.value():g} {w.suffix().strip()} (aplicado en Prueba de paso; "
                    "sin guardar en falconscan.yaml).", "aviso")
        self._mostrar()

    def _abanico(self) -> tuple[float, float]:
        """(apertura, centro) en rad: la apertura del campo de esta pestaña, el centro de «Prueba de paso»."""
        return math.radians(self.c_fan.value()), self.v.paso.params_form.fan_center_rad

    def _abajo_txt(self):
        self.lbl_abajo.setText(f"= {ros_a_sensor(self.c_abajo.value()):+.1f}° en la marca del S2E")

    # ------------------------------------------------------------------ captura
    def _capturar(self, al_terminar):
        ultimo, _, recibidos = self.v.ros.instantanea()
        if not self.v.leyendo or ultimo is None:
            QtWidgets.QMessageBox.information(self, "Calibración", "Inicia la lectura del LiDAR primero (F5).")
            return
        if self.v.reproductor.activo():
            self.v._log("Calibrando sobre un rosbag: los resultados valen para el montaje de esa grabación.", "aviso")
        n = int(self.c_dur.value() * 10)
        self._captura = (n, recibidos, time.monotonic(), al_terminar)
        self.barra.setRange(0, n)
        self.barra.setValue(0)
        for b in (self.b_piso, self.b_dir, self.b_aplicar, self.b_guardar):
            b.setEnabled(False)
        self._t.start()

    def _sondear(self):
        n, n0, t0, al_terminar = self._captura
        _, buffer, recibidos = self.v.ros.instantanea()
        self.barra.setValue(min(n, recibidos - n0))
        if recibidos - n0 < n:
            if time.monotonic() - t0 > ESPERA_MAX_S or not self.v.leyendo:
                self._t.stop()
                self._captura = None
                self._habilitar()
                QtWidgets.QMessageBox.warning(self, "Calibración", "Dejaron de llegar escaneos; captura cancelada.")
            return
        self._t.stop()
        self._captura = None
        escaneos = [e for e in buffer[-n:] if e.ranges.size == buffer[-1].ranges.size]
        p = self.p_captura = self.v.paso.proc.p   # corrección de rango efectiva (el simulador no la lleva)
        pila = np.stack([(validate_ranges(e.scan.ranges, max(p.range_min_m, e.scan.range_min),
                                          min(p.range_max_m, e.scan.range_max)) - p.range_bias_m) * p.range_scale
                         for e in escaneos])
        try:
            al_terminar(pila, escaneos[-1].ang)
        finally:
            self._habilitar()

    def _habilitar(self):
        self.b_piso.setEnabled(True)
        hay = self.res is not None
        self.b_dir.setEnabled(hay)
        self.b_aplicar.setEnabled(hay and not self.res.failed)    # FALLA: solo volver a capturar
        self.b_guardar.setEnabled(hay and not self.res.failed)

    # ------------------------------------------------------------------ paso 2: piso
    def _piso_capturado(self, pila, ang):
        self.params_antes = self.v.paso.params_form
        self.ang = ang
        self.pila_vacio = pila
        self.dir_detectado = None
        self.lbl_dir.setText("")
        self.res = None
        self._mostrar()
        if self.res is not None:
            c = self.res
            self.v._log(f"Calibración: offset {c.range_offset_m * 100:+.1f} cm ({c.offset_status}), desvío de la "
                        f"flecha {math.degrees(-c.arrow_deviation_rad):+.2f}° ({c.angle_status})"
                        + (f" · FALLA: {' '.join(c.failures)}" if c.failed else f" · {len(c.warnings)} avisos."),
                        "error" if c.failed else ("aviso" if c.warnings else "ok"))

    def _calcular(self):
        """Calibra la última captura con los datos del formulario (pendiente y sentido vigentes)."""
        p = self.v.paso
        q = p.params_yaml                                   # nominales: el yaml no cambia
        d = self.dir_detectado or self.params_antes.angle_direction
        return calibrate_mount(
            self.pila_vacio, self.ang, self.c_h.value(), math.radians(self.c_abajo.value()),
            nominal_down_rad=math.radians(nadir_ros_deg(q)), angle_direction=d,
            floor_slope_rad=math.radians(self.c_pend.value()) if self.chk_pend.isChecked() else None,
            roi_x_m=(q.roi_x_min_m, q.roi_x_max_m), roi_z_max_m=self.c_zmax.value(),
            object_h_threshold_m=q.object_h_threshold_m,
            offset_tol_ok_m=q.cal_offset_tol_ok_m, offset_tol_max_m=q.cal_offset_tol_max_m,
            angle_tol_ok_rad=q.cal_angle_tol_ok_rad, angle_tol_max_rad=q.cal_angle_tol_max_rad,
            fan_angle_rad=self._abanico()[0], fan_center_rad=self._abanico()[1])

    def _mostrar(self, *_):
        if self.pila_vacio is None:
            return
        try:
            c = self.res = self._calcular()
        except ValueError as e:
            self.res = None
            self.lbl_res.setText(f"<span style='color:{ROJO}'>{e}</span>")
            self.lbl_avisos.setText("")
            self._habilitar()
            return
        q = self.v.paso.params_yaml
        color = {OK: VERDE, CORREGIDO: ACENTO, FALLA: ROJO}
        txt = {OK: "OK", CORREGIDO: "corregido", FALLA: "FALLA"}
        def marca(estado):
            return f"<b style='color:{color[estado]}'>{txt[estado]}</b>"
        abajo, nominal = math.degrees(c.down_angle_rad), math.degrees(c.nominal_down_rad)
        desvio_marca = math.degrees(-c.arrow_deviation_rad)       # en la marca del S2E el signo se invierte
        # Cómo se vería el piso en los bordes de la ROI con lo nominal (lo que corrige la calibración)
        g0 = MountGeometry(c.perp_height_m, offset_desde_nadir(nominal, c.angle_direction), c.angle_direction)
        r = np.where(c.fit.inliers, self._mediana(), np.nan)
        x0, z0 = polar_to_xz(r, self.ang, g0)
        bordes = []
        for xb in (q.roi_x_min_m, q.roi_x_max_m):
            sel = np.isfinite(x0) & (np.abs(x0 - xb) < 0.25)
            bordes.append(f"{np.median(z0[sel]) * 100:+.1f}" if sel.any() else "—")
        lineas = [
            f"<b>1 · Altura:</b> parámetro (cinta) {c.height_m:.3f} m"
            + (f" → {c.perp_height_m:.3f} m perpendicular con la pendiente" if c.floor_slope_rad is not None else "")
            + f" · el sensor ve el piso a {c.seen_height_m:.3f} m",
            f"&nbsp;&nbsp;&nbsp;offset del sensor <b>{c.range_offset_m * 100:+.1f} cm</b> "
            f"<span style='color:{TEXTO_SUAVE}'>± {c.range_offset_sigma_m * 1e3:.1f} mm</span> {marca(c.offset_status)}",
            f"<b>2 · Piso:</b> visto de x = {c.floor_x_range_m[0]:.2f} a {c.floor_x_range_m[1]:.2f} m · cobertura ROI "
            f"{c.roi_coverage * 100:.0f} % · planitud ±{c.roi_floor_dev_m * 1e3:.0f} mm · RMS del modelo "
            f"{c.model_rms_m * 1e3:.1f} mm · {c.fit.n_points} haces, mediana de {c.n_scans} escaneos",
            f"<b>3 · Rotación:</b> desvío de la flecha 0° respecto a la perpendicular al piso "
            f"<b>{desvio_marca:+.2f}°</b> <span style='color:{TEXTO_SUAVE}'>± "
            f"{math.degrees(c.down_angle_sigma_rad):.3f}°</span> {marca(c.angle_status)}",
            f"&nbsp;&nbsp;&nbsp;rayo hacia el piso {abajo:.2f}° ROS (nominal {nominal:.2f}°); sin corregir, el piso "
            f"en los bordes de la ROI se vería a z = {bordes[0]} / {bordes[1]} cm",
        ]
        if c.plumb_angle_rad is not None:
            pl = math.degrees(c.plumb_angle_rad)
            lineas.append(f"Plomada (vertical real): {pl:.2f}° ROS ({ros_a_sensor(pl):+.2f}° en la marca): la flecha "
                          f"está a {(nominal - pl + 180) % 360 - 180:+.2f}° de la vertical; el piso aporta "
                          f"{math.degrees(c.floor_slope_rad):+.2f}° a la rotación compensada.")
        lineas.append(f"Sentido: {'+1 (normal)' if c.angle_direction > 0 else '−1 (espejado)'}"
                      + (" (detectado)" if self.dir_detectado else " (el configurado; el paso 3 es opcional)"))
        if self.v.paso.fuente_sim:
            lineas.append(f"<span style='color:{AMBAR}'>Datos del SIMULADOR: sirve para probar la rutina, "
                          "no calibra el sensor real.</span>")
        self.lbl_res.setText("<br>".join(lineas))
        avisos = [f"<span style='color:{ROJO}'>✖ {w}</span>" for w in c.failures]
        avisos += [f"<span style='color:{AMBAR}'>⚠ {w}</span>" for w in c.warnings]
        if c.failed:
            avisos.append(f"<span style='color:{ROJO}'>Calibración fallida: corrige el montaje y vuelve a capturar "
                          "(no se puede aplicar).</span>")
        self.lbl_avisos.setText("<br>".join(avisos) or f"<span style='color:{VERDE}'>✔ Sin avisos.</span>")
        self._habilitar()
        self._dibujar(c)

    def _mediana(self):
        return np.nanmedian(self.pila_vacio, axis=0) if self.pila_vacio.shape[0] > 1 else self.pila_vacio[0]

    def _dibujar(self, c):
        r = self._mediana() - c.range_offset_m
        p = self.v.paso.params_yaml
        x, z = polar_to_xz(r, self.ang, c.geometry)
        ok = np.isfinite(x) & (z > -1.0) & (z < c.perp_height_m + 1.0)
        fan = self._abanico()
        dentro = fan_mask(self.ang, c.geometry, *fan)
        piso = ok & c.fit.inliers
        alto = ok & dentro & ~piso & (x >= p.roi_x_min_m) & (x < p.roi_x_max_m) & (z > 0.10) & (z < self.c_zmax.value())
        otros = ok & ~piso & ~alto
        self.p_piso.setData(x[piso], z[piso])
        self.p_alto.setData(x[alto], z[alto])
        self.p_otros.setData(x[otros], z[otros])
        self.sensor.setData([0.0], [c.perp_height_m])
        self.abanico.setData(*bordes_abanico(0.0, c.perp_height_m, *fan,
                                             1.5 * max(c.perp_height_m, p.roi_x_max_m - p.roi_x_min_m)))
        self.roi_rect.setRect(p.roi_x_min_m, 0.0, p.roi_x_max_m - p.roi_x_min_m, self.c_zmax.value())
        if c.plumb_angle_rad is not None:       # vertical real dibujada en el marco del piso
            xp, zp = polar_to_xz(np.array([0.0, c.perp_height_m * 1.05]), np.full(2, c.plumb_angle_rad), c.geometry)
            self.plomada.setData(xp, zp)
        else:
            self.plomada.setData([], [])
        self.leyenda.setText("verde: piso · rojo: algo en la ROI\ngris: resto · punteado: abanico"
                             + (" · ámbar: plomada" if c.plumb_angle_rad is not None else ""))
        self.leyenda.setPos(p.roi_x_min_m, c.perp_height_m + 1.0)
        self.r_puntos.setData(x[piso], z[piso] * 1e3)
        celdas = np.arange(np.floor(x[piso].min() * 4) / 4, x[piso].max() + 0.25, 0.25) if piso.any() else []
        med = [np.median(z[piso][(x[piso] >= a) & (x[piso] < a + 0.25)]) * 1e3
               if ((x[piso] >= a) & (x[piso] < a + 0.25)).any() else np.nan for a in celdas]
        self.r_mediana.setData(np.asarray(celdas) + 0.125, np.asarray(med, dtype=float))

    def refrescar(self, ultimo):
        """Vista en vivo antes de capturar: el escaneo actual con la configuración vigente."""
        if self.res is not None or ultimo is None:
            return
        p = self.v.paso.proc.p
        r = (validate_ranges(ultimo.scan.ranges, max(p.range_min_m, ultimo.scan.range_min),
                             min(p.range_max_m, ultimo.scan.range_max)) - p.range_bias_m) * p.range_scale
        g = MountGeometry(self.c_h.value(), offset_desde_nadir(self.c_abajo.value(), p.angle_direction),
                          p.angle_direction)
        x, z = polar_to_xz(r, ultimo.ang, g)
        ok = np.isfinite(x) & (z > -2.0) & (z < g.height_m + 1.0)
        self.p_otros.setData(x[ok], z[ok])
        self.sensor.setData([0.0], [g.height_m])
        self.abanico.setData(*bordes_abanico(0.0, g.height_m, *self._abanico(),
                                             1.5 * max(g.height_m, p.roi_x_max_m - p.roi_x_min_m)))
        self.roi_rect.setRect(p.roi_x_min_m, 0.0, p.roi_x_max_m - p.roi_x_min_m, self.c_zmax.value())
        self.leyenda.setText("En vivo, configuración actual:\nel piso debería verse en z ≈ 0")
        self.leyenda.setPos(p.roi_x_min_m, g.height_m + 1.0)

    # ------------------------------------------------------------------ paso 3: sentido
    def _objeto_capturado(self, pila, ang):
        if self.res is None or pila.shape[1] != self.pila_vacio.shape[1]:
            return
        d = detect_direction(self.pila_vacio, pila, ang, self.res.down_angle_rad)
        if d is None:
            self.lbl_dir.setText(f"<span style='color:{ROJO}'>No veo ningún objeto nuevo respecto al piso vacío. "
                                 "Acércalo a la vertical del sensor o usa uno más alto.</span>")
            return
        sentido, xm, n = d
        self.dir_detectado = sentido
        cambio = sentido != self.params_antes.angle_direction
        self.lbl_dir.setText(
            f"Objeto detectado ({n} rayos, a {abs(xm):.2f} m de la vertical). "
            f"Sentido = <b>{'+1 (normal)' if sentido > 0 else '−1 (espejado)'}</b>"
            + (f" <span style='color:{AMBAR}'>(cambia respecto al configurado)</span>" if cambio else
               f" <span style='color:{VERDE}'>(coincide con el configurado)</span>")
            + ". Retira el objeto antes de probar pasos.")
        self.v._log(f"Calibración: sentido de avance = {sentido:+d}.", "aviso" if cambio else "ok")
        self._mostrar()

    # ------------------------------------------------------------------ paso 4: aplicar / guardar
    def _aplicar(self, silencioso=False):
        """Valores EFECTIVOS a «Prueba de paso»: H = cinta, offset b y rayo calibrado. El yaml no cambia."""
        c = self.res
        if c is None or c.failed:
            return None
        paso = self.v.paso
        p0 = self.p_captura                     # corrección de rango con que se capturó
        paso.campos["mount_height_m"][0].setValue(c.height_m + paso.campos["floor_z_m"][0].value())
        paso.c_nadir.setValue(math.degrees(c.down_angle_rad))
        paso.c_dir.setCurrentIndex(0 if c.angle_direction > 0 else 1)
        paso.fijar_calibracion({"range_bias_m": c.range_bias_for(p0.range_bias_m, p0.range_scale)})
        if not silencioso:
            self.v._log(f"Calibración aplicada (efectiva, el yaml no cambia): H = {c.height_m:.3f} m, offset "
                        f"{c.range_offset_m * 100:+.1f} cm, abajo = {math.degrees(c.down_angle_rad):.2f}° ROS, "
                        f"sentido {c.angle_direction:+d}.", "ok")
        return c

    def _guardar(self):
        """Ficha de calibración. Del yaml solo se actualiza mount_height_m si la cinta cambió (es el
        valor del instalador); offset y rotación son efectivos y se recalculan en cada calibración."""
        c = self._aplicar(silencioso=True)
        if c is None:
            return
        paso = self.v.paso
        antes = paso.params_yaml
        if not math.isclose(c.height_m, antes.mount_height_m - antes.floor_z_m, abs_tol=5e-4):
            r = QtWidgets.QMessageBox.question(
                self, "Altura del sensor",
                f"La altura medida con cinta ({c.height_m:.3f} m) es distinta de mount_height_m del yaml "
                f"({antes.mount_height_m:.3f} m).\n¿Guardarla en {paso.yaml_ruta}?\n\n"
                "El offset y la rotación calibrados NO se guardan: son efectivos y se recalculan en cada calibración.")
            if r == QtWidgets.QMessageBox.StandardButton.Yes:
                guardar_params(paso.yaml_ruta, replace(antes, mount_height_m=c.height_m + antes.floor_z_m),
                               ["mount_height_m"])
                paso.params_yaml = cargar_params(paso.yaml_ruta)
                self.v._log(f"falconscan.yaml: mount_height_m = {c.height_m + antes.floor_z_m:.3f} m", "ok")
        self.carpeta.mkdir(parents=True, exist_ok=True)
        ruta = self.carpeta / f"calibracion_{dt.datetime.now():%Y%m%d_%H%M%S}.json"
        deg = math.degrees
        p0 = self.p_captura
        ficha = {
            "fecha": dt.datetime.now().isoformat(timespec="seconds"),
            "fuente": "simulador" if paso.fuente_sim else ("rosbag" if self.v.reproductor.activo() else "LiDAR real"),
            "entradas": {
                "mount_height_m_cinta": c.height_m,
                "pendiente_piso_deg": None if c.floor_slope_rad is None else deg(c.floor_slope_rad),
                "rayo_abajo_aprox_ros_deg": self.c_abajo.value(),
                "rayo_abajo_nominal_ros_deg": deg(c.nominal_down_rad),
                "escaneos": c.n_scans,
                "range_bias_m": p0.range_bias_m, "range_scale": p0.range_scale,
                "roi_z_max_m": self.c_zmax.value(),
                "abanico_apertura_deg": self.c_fan.value(), "abanico_centro_deg": deg(self._abanico()[1]),
            },
            "resultado": {
                "offset_rango_m": c.range_offset_m, "offset_sigma_m": c.range_offset_sigma_m,
                "offset_estado": c.offset_status, "altura_vista_m": c.seen_height_m,
                "range_bias_efectivo_m": c.range_bias_for(p0.range_bias_m, p0.range_scale),
                "rayo_abajo_ros_deg": deg(c.down_angle_rad), "rayo_sigma_deg": deg(c.down_angle_sigma_rad),
                "desvio_flecha_marca_deg": deg(-c.arrow_deviation_rad), "rotacion_estado": c.angle_status,
                "angle_offset_efectivo_rad": c.angle_offset_rad, "angle_direction": c.angle_direction,
                "sentido_detectado": self.dir_detectado is not None,
                "plomada_ros_deg": None if c.plumb_angle_rad is None else deg(c.plumb_angle_rad),
                "rms_modelo_m": c.model_rms_m, "puntos_piso": c.fit.n_points,
                "piso_x_m": list(c.floor_x_range_m), "cobertura_roi": c.roi_coverage,
                "planitud_roi_m": c.roi_floor_dev_m, "objeto_roi_x_m": c.roi_object_x_m,
                "fraccion_movil": c.moving_fraction,
            },
            "avisos": list(c.warnings),
            "escaneo_mediana": {"angulos_rad": [round(float(a), 6) for a in self.ang],
                                "rangos_m": [None if not np.isfinite(v) else round(float(v), 5)
                                             for v in self._mediana()]},
        }
        ruta.write_text(json.dumps(ficha, ensure_ascii=False, indent=1))
        self.v._log(f"Ficha de calibración guardada en {ruta}", "ok")
