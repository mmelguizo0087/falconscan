"""Visor LiDAR FalconScan: lecturas en vivo de /scan, control del driver y análisis de sector.

Pensado para la etapa 0 (caracterización del sensor): ver el escaneo, medir una pared,
grabar rosbags y exportar capturas sin usar la terminal.

    tools/visor_lidar/visor_lidar.sh          # entorno ROS + este script

Convenciones de ángulo (sllidar_ros2, inverted:=false):
    ROS     : ángulo del mensaje LaserScan, antihorario, 0° = eje x del frame `lidar`.
    Sensor  : marca física del S2E. ángulo_sensor = 180° − ángulo_ROS, mostrado en [−180°, 180°)
              para que el frente del sensor (0°) quede en el centro del eje.
La vista superior es el frame ROS, opcionalmente girado −90° para dejar el frente del sensor
hacia arriba (rotación, nunca espejo).
"""
from __future__ import annotations

import csv
import datetime as dt
import math
import os
import re
import signal
import sys
import threading
import time
import warnings
from collections import deque
from pathlib import Path

import numpy as np
import pyqtgraph as pg
from pyqtgraph.Qt import QtCore, QtGui, QtWidgets

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan

from paso import PanelPaso, ReproductorBag, Scan, inspeccionar_bag
from calibracion import PanelCalibracion
from comun import (ACENTO, AMBAR, BORDE, FONDO, PANEL, ROJO, TEXTO, TEXTO_SUAVE, VERDE,
                   ajuste_pared, chip, pantalla_compacta, ros_a_sensor, titulo_grafico)

REPO = Path(__file__).resolve().parents[2]
DATOS = Path(os.environ.get("FALCONSCAN_DATOS", REPO.parent))
CARPETA_BAGS = DATOS / "rosbags"
CARPETA_CAPTURAS = DATOS / "capturas"
CALIBRACION_YAML = REPO / "src" / "falconscan_velocity" / "config" / "velocity.yaml"

LIDAR_IP = "192.168.11.2"
CMD_DRIVER = ["ros2", "launch", "sllidar_ros2", "sllidar_s2e_launch.py", "frame_id:=lidar"]
CMD_SIM = ["ros2", "run", "falconscan_velocity", "sim_publisher",
           "--ros-args", "-p", "alternate_direction:=true"]   # camiones en ambos sentidos

BUFFER_ESCANEOS = 100      # 10 s a 10 Hz: base del análisis de sector
SIN_DATOS_S = 2.0          # sin escaneos durante este tiempo -> alerta


ESTILO = f"""
QWidget {{ background: {FONDO}; color: {TEXTO}; font-size: 12px; }}
QGroupBox {{ background: {PANEL}; border: 1px solid {BORDE}; border-radius: 7px;
            margin-top: 12px; padding: 8px 6px 6px 6px; font-weight: 600; }}
QGroupBox::title {{ subcontrol-origin: margin; left: 10px; padding: 0 4px; color: {ACENTO}; }}
QLabel {{ background: transparent; }}
QLabel#valor {{ font-size: 15px; font-weight: 600; }}
QLabel#grande {{ font-size: 20px; font-weight: 700; color: {ACENTO}; }}
QLabel#suave {{ color: {TEXTO_SUAVE}; }}
QPushButton {{ background: {BORDE}; border: 1px solid #3a4452; border-radius: 6px;
              padding: 6px 10px; min-height: 18px; font-weight: 600; }}
QPushButton:hover {{ background: #344050; }}
QPushButton:disabled {{ color: #5a6672; background: #20262e; border-color: #262d36; }}
QPushButton#iniciar {{ background: #1f6f3a; border-color: #2a8a4a; }}
QPushButton#iniciar:hover {{ background: #268546; }}
QPushButton#detener {{ background: #7a2a2a; border-color: #973636; }}
QPushButton#detener:hover {{ background: #903333; }}
QPushButton#grabar:checked {{ background: {ROJO}; border-color: {ROJO}; }}
QPushButton#congelar:checked {{ background: {AMBAR}; color: #111; border-color: {AMBAR}; }}
QComboBox, QSpinBox, QDoubleSpinBox, QLineEdit {{ background: #0f1318; border: 1px solid {BORDE};
              border-radius: 5px; padding: 3px 5px; min-height: 20px; }}
QCheckBox {{ background: transparent; spacing: 8px; }}
QCheckBox::indicator {{ width: 16px; height: 16px; border: 1px solid #4a5666; border-radius: 4px; background: #0f1318; }}
QCheckBox::indicator:checked {{ background: {ACENTO}; border-color: {ACENTO}; }}
QPlainTextEdit {{ background: #0b0e12; border: 1px solid {BORDE}; border-radius: 6px;
                 font-family: monospace; font-size: 11px; color: #b8c4cf; }}
QSplitter::handle {{ background: {BORDE}; }}
QToolTip {{ background: {PANEL}; color: {TEXTO}; border: 1px solid {BORDE}; }}
QScrollArea {{ border: none; }}
QTabWidget::pane {{ border: none; }}
QTabBar::tab {{ background: {PANEL}; color: {TEXTO_SUAVE}; border: 1px solid {BORDE}; border-bottom: none;
               border-top-left-radius: 7px; border-top-right-radius: 7px; padding: 5px 14px; margin-right: 3px;
               font-weight: 600; }}
QTabBar::tab:selected {{ background: #243040; color: {TEXTO}; border-color: {ACENTO}; }}
QTableWidget QLineEdit {{ padding: 0; }}
QStatusBar {{ background: {PANEL}; border-top: 1px solid {BORDE}; }}
QStatusBar::item {{ border: none; }}
QStatusBar QPushButton {{ padding: 2px 8px; min-height: 14px; font-size: 11px; }}
QPushButton#registro[alerta="true"] {{ color: {ROJO}; border-color: {ROJO}; }}
"""


# ----------------------------------------------------------------------------- utilidades

def leer_calibracion() -> tuple[float, float]:
    """(range_scale, range_bias_m) de velocity.yaml; (1, 0) si no se encuentran."""
    try:
        txt = CALIBRACION_YAML.read_text()
    except OSError:
        return 1.0, 0.0
    def valor(nombre, defecto):
        m = re.search(rf"^\s*{nombre}:\s*([-+0-9.eE]+)", txt, re.M)
        return float(m.group(1)) if m else defecto
    return valor("range_scale", 1.0), valor("range_bias_m", 0.0)


class Escaneo:
    """LaserScan convertido a numpy (solo lo que usa el visor)."""
    __slots__ = ("stamp", "rx", "ranges", "intens", "ang", "range_max", "scan")

    def __init__(self, msg: LaserScan, rx: float):
        self.stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self.rx = rx
        crudo = np.asarray(msg.ranges, dtype=np.float32)
        # Para el estimador: el mensaje tal cual (él valida rangos y aplica su calibración)
        self.scan = Scan(stamp=self.stamp, angle_min=msg.angle_min, angle_increment=msg.angle_increment,
                         ranges=crudo, time_increment=msg.time_increment,
                         range_min=msg.range_min, range_max=msg.range_max)
        r = crudo.copy()
        r[~np.isfinite(r) | (r <= 0)] = np.nan
        self.ranges = r
        self.intens = np.asarray(msg.intensities, dtype=np.float32) if len(msg.intensities) == r.size else None
        self.ang = msg.angle_min + np.arange(r.size) * msg.angle_increment   # rad, ROS
        self.range_max = msg.range_max


# ----------------------------------------------------------------------------- ROS

class PuenteRos:
    """Nodo rclpy girando en un hilo; el GUI lee el último escaneo con un temporizador."""

    def __init__(self):
        rclpy.init()
        self.node = Node("visor_lidar")
        self._lock = threading.Lock()
        self._sub = None
        self.ultimo: Escaneo | None = None
        self.buffer: deque[Escaneo] = deque(maxlen=BUFFER_ESCANEOS)
        self.recibidos = 0
        self.procesar = None          # callable(scan, ahora): estimador + máquina de estados
        self._hilo = threading.Thread(target=self._spin, daemon=True)
        self._hilo.start()

    def _spin(self):
        try:
            rclpy.spin(self.node)
        except Exception:  # al cerrar rclpy desde otro hilo
            pass

    def limpiar(self):
        with self._lock:
            self.buffer.clear()
            self.ultimo = None
            self.recibidos = 0

    def suscribir(self, topico: str):
        self.desuscribir()
        self.limpiar()
        self._sub = self.node.create_subscription(LaserScan, topico, self._cb, qos_profile_sensor_data)

    def desuscribir(self):
        if self._sub is not None:
            self.node.destroy_subscription(self._sub)
            self._sub = None

    def _cb(self, msg: LaserScan):
        rx = time.monotonic()
        self.recibir(msg, rx, ahora=rx)

    def inyectar(self, msg: LaserScan):
        """Escaneo de un rosbag: la máquina de estados usa el tiempo del propio bag."""
        self.recibir(msg, time.monotonic(), ahora=None)

    def recibir(self, msg: LaserScan, rx: float, ahora: float | None):
        e = Escaneo(msg, rx)
        if self.procesar is not None:
            self.procesar(e.scan, ahora)
        with self._lock:
            self.ultimo = e
            self.buffer.append(e)
            self.recibidos += 1

    def instantanea(self):
        with self._lock:
            return self.ultimo, list(self.buffer), self.recibidos

    def publicadores(self, topico: str) -> int:
        return self.node.count_publishers(topico)

    def cerrar(self):
        self.desuscribir()
        self.node.destroy_node()
        rclpy.try_shutdown()


# ----------------------------------------------------------------------------- procesos

class Proceso(QtCore.QObject):
    """Proceso externo (driver, simulador, ros2 bag) con salida al registro y parada por SIGINT."""
    linea = QtCore.Signal(str)
    termino = QtCore.Signal(int)

    def __init__(self, nombre: str, parent=None):
        super().__init__(parent)
        self.nombre = nombre
        self.p = QtCore.QProcess(self)
        self.p.setProcessChannelMode(QtCore.QProcess.ProcessChannelMode.MergedChannels)
        self.p.readyReadStandardOutput.connect(self._leer)
        self.p.finished.connect(self._fin)
        # Plan B si el SIGINT no basta; se cancela al terminar para no matar un proceso posterior
        self._t_forzar = QtCore.QTimer(self, singleShot=True, interval=6000, timeout=self._forzar)

    def _fin(self, code, _estado):
        self._t_forzar.stop()
        self.termino.emit(code)

    def iniciar(self, cmd: list[str], cwd: Path | None = None):
        if cwd:
            self.p.setWorkingDirectory(str(cwd))
        self.p.start(cmd[0], cmd[1:])

    def activo(self) -> bool:
        return self.p.state() != QtCore.QProcess.ProcessState.NotRunning

    def detener(self, esperar_ms: int = 0):
        """SIGINT (parada limpia: el driver apaga el motor, ros2 bag cierra el archivo)."""
        if not self.activo():
            return
        os.kill(int(self.p.processId()), signal.SIGINT)
        self._t_forzar.start()
        if esperar_ms:
            if not self.p.waitForFinished(esperar_ms):
                self._forzar()
                self.p.waitForFinished(2000)

    def _forzar(self):
        if self.activo():
            self.p.terminate()
            if not self.p.waitForFinished(2000):
                self.p.kill()

    def _leer(self):
        txt = bytes(self.p.readAllStandardOutput()).decode(errors="replace")
        for l in txt.splitlines():
            if l.strip():
                self.linea.emit(l)


# ----------------------------------------------------------------------------- widgets

class Insignia(QtWidgets.QLabel):
    def fijar(self, texto: str, color: str):
        self.setText(f"●  {texto}")
        self.setStyleSheet(f"color: {color}; font-weight: 700; font-size: 12px; padding: 3px 8px;"
                           f"border: 1px solid {color}; border-radius: 12px; background: transparent;")


class VentanaPrincipal(QtWidgets.QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("FalconScan · Visor LiDAR")
        self.resize(1280, 800)
        self.setMinimumSize(800, 480)
        self.ros = PuenteRos()
        self.driver = Proceso("driver", self)
        self.driver.linea.connect(self._linea_driver)
        self.driver.termino.connect(self._driver_termino)
        self.bag = Proceso("ros2 bag", self)
        self.bag.linea.connect(lambda l: self._log(l, "bag"))
        self.bag.termino.connect(self._bag_termino)
        self.ping = QtCore.QProcess(self)
        self.ping.finished.connect(self._ping_resultado)

        self.leyendo = False
        self.driver_externo = False
        self.t_inicio = 0.0
        self.t_bag = 0.0
        self.ruta_bag: Path | None = None
        self.calib = leer_calibracion()
        self._ultimo_dibujado = -1
        self._escaneo_mostrado: Escaneo | None = None
        self._r_mostrado: np.ndarray | None = None

        self.reproductor = ReproductorBag(self.ros.inyectar, self)
        self.reproductor.termino.connect(self._reproduccion_termino)
        self.reproductor.progreso.connect(lambda f: setattr(self, "_progreso_rep", f))
        self._progreso_rep = 0.0

        self._construir()
        self.ros.procesar = self.paso.proc.procesar
        self._conv_cambiada(inicial=True)
        self._orientar_vista()
        self._estado_detenido()

        self.t_dibujo = QtCore.QTimer(self, interval=66, timeout=self._refrescar)     # ~15 fps
        self.t_dibujo.start()
        self.t_analisis = QtCore.QTimer(self, interval=400, timeout=self._analizar_sector)
        self.t_analisis.start()
        # SIN_DATOS de la máquina de estados (solo en vivo; un rosbag usa su propio tiempo)
        self.t_paso = QtCore.QTimer(self, interval=100, timeout=lambda: self.leyendo and not self.reproductor.activo()
                                    and self.paso.proc.tick(time.monotonic()))
        self.t_paso.start()

    # ------------------------------------------------------------------ construcción
    def _construir(self):
        central = QtWidgets.QWidget()
        self.setCentralWidget(central)
        raiz = QtWidgets.QVBoxLayout(central)
        raiz.setContentsMargins(8, 6, 8, 4)
        raiz.setSpacing(5)

        # --- barra superior: control + estado (una fila; cabe en 1024 px)
        barra = QtWidgets.QHBoxLayout()
        barra.setSpacing(5)
        titulo = QtWidgets.QLabel("FalconScan")
        titulo.setStyleSheet("font-size: 16px; font-weight: 700;")
        titulo.setToolTip("Visor LiDAR")
        barra.addWidget(titulo)
        barra.addSpacing(6)
        self.fuente = QtWidgets.QComboBox()
        self.fuente.addItems(["LiDAR S2E", "Simulador", "Rosbag…"])
        self.fuente.setToolTip(f"De dónde vienen los datos: LiDAR S2E ({LIDAR_IP}), simulador de volquetas\n"
                               "(probar sin hardware) o un rosbag grabado (repetir un paso con otros parámetros).")
        barra.addWidget(self.fuente)
        self.vel_rep = QtWidgets.QComboBox()
        self.vel_rep.addItems(["1×", "2×", "5×", "máx."])
        self.vel_rep.setToolTip("Velocidad de reproducción del rosbag")
        self.vel_rep.currentIndexChanged.connect(self._vel_rep)
        barra.addWidget(self.vel_rep)
        self.fuente.currentIndexChanged.connect(lambda i: self.vel_rep.setVisible(i == 2))
        self.vel_rep.setVisible(False)
        self.b_iniciar = QtWidgets.QPushButton("▶ Iniciar", objectName="iniciar")
        self.b_iniciar.setToolTip("Arranca el driver (gira el motor) y empieza a mostrar /scan  [F5]")
        self.b_iniciar.setShortcut("F5")
        self.b_detener = QtWidgets.QPushButton("■ Detener", objectName="detener")
        self.b_detener.setToolTip("Detiene el driver y el motor del LiDAR  [F6]")
        self.b_detener.setShortcut("F6")
        self.b_congelar = QtWidgets.QPushButton("❚❚ Congelar", objectName="congelar", checkable=True)
        self.b_congelar.setToolTip("Congela la imagen para inspeccionarla; el LiDAR sigue leyendo.\n"
                                   "Con un rosbag, además pausa la reproducción.  [Espacio]")
        self.b_congelar.setShortcut("Space")
        self.b_grabar = QtWidgets.QPushButton("● Grabar", objectName="grabar", checkable=True)
        self.b_grabar.setToolTip(f"Graba /scan como rosbag en {CARPETA_BAGS}")
        self.b_captura = QtWidgets.QPushButton("⤓ Captura")
        self.b_captura.setToolTip(f"Guarda el escaneo visible (CSV) y una imagen (PNG) en {CARPETA_CAPTURAS}  [Ctrl+S]")
        self.b_captura.setShortcut("Ctrl+S")
        self.b_ayuda = QtWidgets.QPushButton("?")
        self.b_ayuda.setFixedWidth(34)
        for b in (self.b_iniciar, self.b_detener, self.b_congelar, self.b_grabar, self.b_captura, self.b_ayuda):
            barra.addWidget(b)
        barra.addStretch(1)
        self.insignia = Insignia()
        barra.addWidget(self.insignia)
        self.b_pantalla = QtWidgets.QPushButton("⛶")
        self.b_pantalla.setFixedWidth(34)
        self.b_pantalla.setToolTip("Pantalla completa  [F11]")
        self.b_pantalla.setShortcut("F11")
        self.b_pantalla.clicked.connect(self._pantalla_completa)
        barra.addWidget(self.b_pantalla)
        self.b_salir = QtWidgets.QPushButton("✕", objectName="detener")
        self.b_salir.setFixedWidth(34)
        self.b_salir.setToolTip("Cerrar el visor (detiene el driver)")
        self.b_salir.clicked.connect(self.close)
        barra.addWidget(self.b_salir)
        raiz.addLayout(barra)

        self.b_iniciar.clicked.connect(self.iniciar)
        self.b_detener.clicked.connect(self.detener)
        self.b_grabar.toggled.connect(self._grabar)
        self.b_congelar.toggled.connect(lambda on: setattr(self.reproductor, "pausado", on))
        self.b_captura.clicked.connect(self._captura)
        self.b_ayuda.clicked.connect(self._ayuda)

        # --- zona central: pestañas
        div = QtWidgets.QSplitter(QtCore.Qt.Orientation.Horizontal)
        div.addWidget(self._construir_graficos())
        div.addWidget(self._construir_panel())
        div.setStretchFactor(0, 1)
        div.setSizes([720, 300])
        self.pestanas = QtWidgets.QTabWidget()
        self.pestanas.addTab(div, "Sensor")
        self.paso = PanelPaso(self, CALIBRACION_YAML, DATOS)
        self.pestanas.addTab(self.paso, "Prueba de paso")
        self.calib = PanelCalibracion(self, DATOS)
        self.pestanas.addTab(self.calib, "Calibración")
        self.pestanas.currentChanged.connect(lambda _: setattr(self, "_ultimo_dibujado", -1))
        raiz.addWidget(self.pestanas, 1)

        # --- registro (plegable: en pantalla chica arranca oculto; la última línea va en la barra de estado)
        self.registro = QtWidgets.QPlainTextEdit(readOnly=True)
        self.registro.setMaximumBlockCount(2000)
        self.registro.setFixedHeight(96)
        raiz.addWidget(self.registro)

        # --- barra de estado: último mensaje + métricas del sensor
        est = self.statusBar()
        est.setSizeGripEnabled(False)
        self.b_registro = QtWidgets.QPushButton("☰ Registro", checkable=True, objectName="registro")
        self.b_registro.setToolTip("Muestra u oculta el registro de mensajes  [F2]")
        self.b_registro.setShortcut("F2")
        self.b_registro.toggled.connect(self._mostrar_registro)
        est.addWidget(self.b_registro)
        self.ultimo_log = QtWidgets.QLabel("")
        self.ultimo_log.setStyleSheet("font-size: 11px;")
        self.ultimo_log.setMinimumWidth(50)
        self.ultimo_log.setSizePolicy(QtWidgets.QSizePolicy.Policy.Ignored, QtWidgets.QSizePolicy.Policy.Preferred)
        est.addWidget(self.ultimo_log, 1)
        for attr, titulo, tip in (
                ("m_frec", "Hz", "Escaneos por segundo según header.stamp (últimos 2 s)"),
                ("m_jitter", "σΔt", "Variación del intervalo entre escaneos. Afecta directamente al error de velocidad."),
                ("m_validos", "Válidos", "Porcentaje de rayos con distancia válida en el último escaneo (giro completo)"),
                ("m_rango", "Rango", "Distancia mínima y máxima del último escaneo"),
                ("m_cuenta", "Escaneos", "Escaneos recibidos"),
                ("m_tiempo", "Tiempo", "Tiempo de lectura")):
            w, lbl = chip(titulo, tip)
            setattr(self, attr, lbl)
            est.addPermanentWidget(w)
        self.compacto = pantalla_compacta()
        self.b_registro.setChecked(not self.compacto)
        self._mostrar_registro(not self.compacto)

    def _mostrar_registro(self, on):
        self.registro.setVisible(on)
        self.ultimo_log.setVisible(not on)
        self.b_registro.setText("☰ Registro")
        self.b_registro.setProperty("alerta", False)
        self.b_registro.style().polish(self.b_registro)

    def _pantalla_completa(self):
        if self.isFullScreen():
            self.showMaximized()
        else:
            self.showFullScreen()

    def _construir_graficos(self) -> QtWidgets.QWidget:
        pg.setConfigOptions(antialias=True, background=PANEL, foreground=TEXTO_SUAVE)
        cont = QtWidgets.QSplitter(QtCore.Qt.Orientation.Vertical)

        # Vista superior
        self.vista = pg.PlotWidget()
        self.vista.setAspectLocked(True)
        self.vista.showGrid(x=True, y=True, alpha=0.12)
        self.anillos = []
        self.sector_poli = QtWidgets.QGraphicsPolygonItem()
        self.sector_poli.setBrush(pg.mkBrush(47, 184, 196, 40))
        self.sector_poli.setPen(pg.mkPen(ACENTO, width=1, style=QtCore.Qt.PenStyle.DashLine))
        self.vista.addItem(self.sector_poli)
        self.puntos = pg.ScatterPlotItem(size=3, pen=None, pxMode=True)
        self.vista.addItem(self.puntos)
        self.recta_pared = pg.PlotDataItem(pen=pg.mkPen(AMBAR, width=2))
        self.vista.addItem(self.recta_pared)
        sensor = pg.ScatterPlotItem([0], [0], size=14, symbol="o", brush=pg.mkBrush(ACENTO), pen=pg.mkPen("w", width=1))
        self.vista.addItem(sensor)
        # Flecha: dirección física 0° del sensor (= ROS 180°); se coloca en _orientar_vista
        self.flecha = pg.ArrowItem(headLen=16, tailLen=26, tailWidth=5, brush=pg.mkBrush(AMBAR), pen=None)
        self.flecha.setZValue(30)
        self.vista.addItem(self.flecha)
        self.lbl_frente = pg.TextItem("frente (0°)", color=AMBAR)
        self.lbl_frente.setZValue(30)
        self.vista.addItem(self.lbl_frente)
        self.lbl_cursor_vista = pg.TextItem("", color=TEXTO, anchor=(0, 1), fill=pg.mkBrush(18, 22, 28, 200))
        self.lbl_cursor_vista.setZValue(50)
        self.vista.addItem(self.lbl_cursor_vista, ignoreBounds=True)
        self.vista.scene().sigMouseMoved.connect(self._cursor_vista)
        cont.addWidget(self.vista)

        # Ángulo vs distancia
        self.perfil = pg.PlotWidget(title=titulo_grafico("Distancia vs ángulo"))
        self.perfil.showGrid(x=True, y=True, alpha=0.15)
        self.perfil.setLabel("left", "distancia [m]")
        self.curva = pg.PlotDataItem(pen=pg.mkPen(ACENTO, width=1.4), connect="finite")
        self.perfil.addItem(self.curva)
        self.region = pg.LinearRegionItem(values=(170, 190),
                                          brush=pg.mkBrush(47, 184, 196, 35),
                                          hoverBrush=pg.mkBrush(47, 184, 196, 60))
        self.region.setZValue(-10)
        self.region.sigRegionChanged.connect(self._sector_movido)
        self.perfil.addItem(self.region)
        self.cruz = pg.InfiniteLine(angle=90, pen=pg.mkPen(TEXTO_SUAVE, width=1, style=QtCore.Qt.PenStyle.DotLine))
        self.perfil.addItem(self.cruz, ignoreBounds=True)
        self.lbl_cursor_perfil = pg.TextItem("", color=TEXTO, anchor=(0, 1), fill=pg.mkBrush(18, 22, 28, 200))
        self.lbl_cursor_perfil.setZValue(50)
        self.perfil.addItem(self.lbl_cursor_perfil, ignoreBounds=True)
        self.perfil.scene().sigMouseMoved.connect(self._cursor_perfil)
        cont.addWidget(self.perfil)
        cont.setSizes([330, 170])
        return cont

    def _construir_panel(self) -> QtWidgets.QWidget:
        panel = QtWidgets.QWidget()
        lay = QtWidgets.QVBoxLayout(panel)
        lay.setContentsMargins(4, 0, 0, 0)

        # Visualización
        g = QtWidgets.QGroupBox("Visualización")
        f = QtWidgets.QFormLayout(g)
        self.conv = QtWidgets.QComboBox()
        self.conv.addItems(["ROS (mensaje /scan)", "Sensor (marca física 0°)"])
        self.conv.setToolTip("Eje del gráfico distancia-ángulo.\nSensor = 180° − ROS (sllidar_ros2 con inverted:=false).")
        self.conv.currentIndexChanged.connect(self._conv_cambiada)
        f.addRow("Ángulo", self.conv)
        self.chk_arriba = QtWidgets.QCheckBox("Frente del sensor hacia arriba")
        self.chk_arriba.setToolTip("Gira la vista superior para que la marca 0° del sensor apunte hacia arriba.\n"
                                   "Desmarcado: frame ROS tal cual (x a la derecha).")
        self.chk_arriba.setChecked(True)
        self.chk_arriba.toggled.connect(self._orientar_vista)
        f.addRow(self.chk_arriba)
        self.color = QtWidgets.QComboBox()
        self.color.addItems(["Distancia", "Intensidad", "Uniforme"])
        f.addRow("Color puntos", self.color)
        self.promedio = QtWidgets.QSpinBox(minimum=1, maximum=50, value=1, suffix=" escaneos")
        self.promedio.setToolTip("Mediana temporal de los últimos N escaneos: reduce ruido (solo para escenas quietas)")
        f.addRow("Mediana de", self.promedio)
        self.alcance = QtWidgets.QDoubleSpinBox(minimum=0.5, maximum=30, value=6.0, singleStep=0.5, suffix=" m")
        self.alcance.setToolTip("Radio de la vista superior y límite del eje de distancia")
        self.alcance.valueChanged.connect(self._ajustar_vista)
        f.addRow("Alcance vista", self.alcance)
        s, b = self.calib
        self.chk_calib = QtWidgets.QCheckBox(f"Corregir rango (×{s:.6f}, −{b*1e3:.1f} mm)")
        self.chk_calib.setToolTip(f"r_corr = (r − range_bias_m) · range_scale\nValores leídos de {CALIBRACION_YAML}\n"
                                  "Calibración de la prueba de pared (unidad 1). Desactiva para ver el dato crudo.")
        self.chk_calib.setChecked(True)
        f.addRow(self.chk_calib)
        b_vista = QtWidgets.QPushButton("Restablecer vista")
        b_vista.clicked.connect(self._ajustar_vista)
        f.addRow(b_vista)
        lay.addWidget(g)

        # Análisis de sector
        g = QtWidgets.QGroupBox("Análisis del sector seleccionado")
        v = QtWidgets.QVBoxLayout(g)
        ayuda = QtWidgets.QLabel("Arrastra la franja del gráfico inferior sobre la pared o el objeto a medir.")
        ayuda.setObjectName("suave")
        ayuda.setWordWrap(True)
        v.addWidget(ayuda)
        f = QtWidgets.QFormLayout()
        h = QtWidgets.QHBoxLayout()
        self.sec_a1 = QtWidgets.QDoubleSpinBox(minimum=-180, maximum=360, decimals=1, suffix="°", value=170)
        self.sec_a2 = QtWidgets.QDoubleSpinBox(minimum=-180, maximum=360, decimals=1, suffix="°", value=190)
        for w in (self.sec_a1, self.sec_a2):
            w.valueChanged.connect(self._sector_editado)
            h.addWidget(w)
        f.addRow("Sector", h)
        self.ref = QtWidgets.QDoubleSpinBox(minimum=0, maximum=40, decimals=3, singleStep=0.01, suffix=" m")
        self.ref.setSpecialValueText("— (sin referencia)")
        self.ref.setToolTip("Distancia medida con cinta/láser, para calcular el sesgo")
        f.addRow("Referencia", self.ref)
        v.addLayout(f)
        self.a_dist = QtWidgets.QLabel("—", objectName="grande")
        self.a_dist.setToolTip("Distancia perpendicular a la recta ajustada (pared), media sobre el buffer")
        v.addWidget(QtWidgets.QLabel("Distancia perpendicular (ajuste de recta)", objectName="suave"))
        v.addWidget(self.a_dist)
        self.a_tabla = QtWidgets.QLabel("")
        self.a_tabla.setTextFormat(QtCore.Qt.TextFormat.RichText)
        v.addWidget(self.a_tabla)
        lay.addWidget(g)

        # Sensor
        g = QtWidgets.QGroupBox("Sensor")
        f = QtWidgets.QFormLayout(g)
        self.s_sn = QtWidgets.QLabel("—")
        self.s_fw = QtWidgets.QLabel("—")
        self.s_salud = QtWidgets.QLabel("—")
        self.s_modo = QtWidgets.QLabel("—")
        self.s_modo.setWordWrap(True)
        for nombre, w in (("S/N", self.s_sn), ("Firmware / HW", self.s_fw), ("Salud", self.s_salud), ("Modo", self.s_modo)):
            w.setTextInteractionFlags(QtCore.Qt.TextInteractionFlag.TextSelectableByMouse)
            f.addRow(nombre, w)
        self.s_sn.setStyleSheet("font-size: 10px;")
        lay.addWidget(g)

        # Grabación
        g = QtWidgets.QGroupBox("Grabación")
        f = QtWidgets.QFormLayout(g)
        self.nombre_bag = QtWidgets.QLineEdit(placeholderText="p. ej. pared_5m")
        self.nombre_bag.setToolTip("Prefijo del rosbag; se le añade fecha y hora")
        f.addRow("Nombre", self.nombre_bag)
        self.lbl_bag = QtWidgets.QLabel("No grabando", objectName="suave")
        self.lbl_bag.setWordWrap(True)
        f.addRow(self.lbl_bag)
        lay.addWidget(g)
        lay.addStretch(1)

        area = QtWidgets.QScrollArea()
        area.setWidgetResizable(True)
        area.setWidget(panel)
        area.setMinimumWidth(270)
        area.setHorizontalScrollBarPolicy(QtCore.Qt.ScrollBarPolicy.ScrollBarAlwaysOff)
        return area

    # ------------------------------------------------------------------ ciclo de vida
    def iniciar(self):
        if self.leyendo:
            return
        self.b_iniciar.setEnabled(False)
        self.fuente.setEnabled(False)
        if self.fuente.currentIndex() != 2 and self.ros.publicadores("/scan") > 0:
            # Puede ser un driver lanzado desde una terminal, o uno recién detenido que el grafo de
            # ROS aún no olvidó: solo cuenta como externo si de verdad llegan datos.
            self.insignia.fijar("Buscando datos en /scan…", AMBAR)
            self.ros.suscribir("/scan")
            QtCore.QTimer.singleShot(1500, self._sondeo_externo)
            return
        self._arrancar_fuente()

    def _sondeo_externo(self):
        if self.ros.instantanea()[2] > 0:
            self.driver_externo = True
            self._log("Ya hay un nodo publicando /scan fuera del visor: me conecto sin lanzar otro driver. "
                      "«Detener» solo dejará de mostrar datos; ese driver se detiene desde su terminal.", "aviso")
            self._comenzar_lectura("/scan")
        else:
            self.ros.desuscribir()
            self._arrancar_fuente()

    def _arrancar_fuente(self):
        self.driver_externo = False
        if self.fuente.currentIndex() == 2:
            self._reproducir()
            return
        # La calibración es del S2E físico; el simulador ya publica rangos exactos
        real = self.fuente.currentIndex() == 0
        self._fijar_calibracion(real)
        if self.fuente.currentIndex() == 0:
            self.insignia.fijar("Comprobando red…", AMBAR)
            self._log(f"Comprobando que el LiDAR responde en {LIDAR_IP}…")
            self.ping.start("ping", ["-c", "1", "-W", "1", LIDAR_IP])
        else:
            self._lanzar(CMD_SIM, "simulador")

    def _ping_resultado(self, codigo, _estado):
        if codigo == 0:
            self._log("El LiDAR responde. Lanzando driver…", "ok")
            self._lanzar(CMD_DRIVER, "driver")
            return
        self._estado_detenido()
        r = QtWidgets.QMessageBox.warning(
            self, "El LiDAR no responde",
            f"No hay respuesta de {LIDAR_IP}.\n\nRevisa:\n"
            "  • Alimentación de 12 V del LiDAR\n"
            "  • Cable Ethernet entre el LiDAR y el puerto eth0 de la Raspberry\n"
            "  • Que eth0 tenga la IP 192.168.11.100 (perfil de red «lidar»):\n"
            "      nmcli con up lidar\n\n"
            "¿Lanzar el driver de todos modos?",
            QtWidgets.QMessageBox.StandardButton.Yes | QtWidgets.QMessageBox.StandardButton.No,
            QtWidgets.QMessageBox.StandardButton.No)
        if r == QtWidgets.QMessageBox.StandardButton.Yes:
            self._lanzar(CMD_DRIVER, "driver")
        else:
            self._log(f"Sin respuesta de {LIDAR_IP}; lectura cancelada.", "error")

    def _lanzar(self, cmd, nombre):
        self._log("$ " + " ".join(cmd))
        self.driver.nombre = nombre
        self.driver.iniciar(cmd)
        self._comenzar_lectura("/scan")

    def _comenzar_lectura(self, topico):
        self.ros.suscribir(topico)
        self._preparar_lectura(sim=self.fuente.currentIndex() == 1 and not self.driver_externo)

    def _fijar_calibracion(self, real: bool):
        if self.chk_calib.isChecked() != real:
            self.chk_calib.setChecked(real)
            self._log("Corrección de rango " + ("activada (LiDAR real)." if real else
                      "desactivada: datos del simulador, sin el error del sensor."), "aviso")

    def _reproducir(self):
        ruta = QtWidgets.QFileDialog.getExistingDirectory(self, "Elegir rosbag (carpeta)", str(CARPETA_BAGS))
        if not ruta:
            self._estado_detenido()
            return
        try:
            tema, es_sim, n = inspeccionar_bag(ruta)
        except Exception as e:   # noqa: BLE001
            QtWidgets.QMessageBox.warning(self, "Rosbag", f"No pude abrir {ruta}:\n{e}")
            self._estado_detenido()
            return
        if tema is None:
            QtWidgets.QMessageBox.warning(self, "Rosbag", "Ese rosbag no tiene ningún tópico LaserScan.")
            self._estado_detenido()
            return
        self._fijar_calibracion(not es_sim)
        self._log(f"Reproduciendo {ruta} ({tema}, {n} escaneos, "
                  f"{'simulador' if es_sim else 'LiDAR real'}).", "info")
        self.ros.limpiar()
        self._preparar_lectura(sim=es_sim)
        self._vel_rep()
        self.reproductor.pausado = self.b_congelar.isChecked()
        self.reproductor.iniciar(ruta)
        self.b_grabar.setEnabled(False)

    def _vel_rep(self, *_):
        self.reproductor.velocidad = [1.0, 2.0, 5.0, math.inf][self.vel_rep.currentIndex()]

    def _reproduccion_termino(self, msg):
        if msg and msg != "detenido":
            self._log(msg, "error")
        elif not msg:
            self._log("Fin del rosbag.", "ok")
        self.leyendo = False
        self.paso.proc.cortar()
        self._estado_detenido()

    def _preparar_lectura(self, sim: bool):
        """Común a todas las fuentes: la máquina de estados arranca en ARRANQUE con los parámetros
        de la fuente (el simulador no lleva orden invertido ni calibración de rango)."""
        self.paso.fijar_fuente(sim)
        self.paso.proc.activo = True
        self.leyendo = True
        self.t_inicio = time.monotonic()
        self._ultimo_dibujado = -1
        self._progreso_rep = 0.0
        self.b_iniciar.setEnabled(False)
        self.b_detener.setEnabled(True)
        self.b_grabar.setEnabled(True)
        self.fuente.setEnabled(False)
        self.insignia.fijar("Esperando datos…", AMBAR)

    def detener(self):
        if self.b_grabar.isChecked():
            self.b_grabar.setChecked(False)
        self.ros.desuscribir()
        self.leyendo = False
        self.paso.proc.cortar()
        if self.reproductor.activo():
            self.reproductor.detener()   # _reproduccion_termino completa el cambio de estado
            return
        if self.driver.activo():
            self._log(f"Deteniendo {self.driver.nombre}…")
            self.insignia.fijar("Deteniendo…", AMBAR)
            self.b_detener.setEnabled(False)
            self.driver.detener()      # _driver_termino completa el cambio de estado
        else:
            self._estado_detenido()

    def _driver_termino(self, codigo):
        self._log(f"{self.driver.nombre} terminó (código {codigo}).", "ok" if not self.leyendo else "error")
        if self.leyendo:   # murió solo, no lo detuvimos nosotros
            self.ros.desuscribir()
            self.leyendo = False
            self.paso.proc.cortar()
            QtWidgets.QMessageBox.critical(self, "Driver detenido",
                                           f"El {self.driver.nombre} terminó inesperadamente (código {codigo}).\n"
                                           "Revisa el registro de la parte inferior.")
        self._estado_detenido()

    def _estado_detenido(self):
        self.b_iniciar.setEnabled(True)
        self.b_detener.setEnabled(False)
        self.b_grabar.setEnabled(False)
        self.fuente.setEnabled(True)
        self.insignia.fijar("Detenido", TEXTO_SUAVE)

    def closeEvent(self, ev):
        self.reproductor.detener()
        if self.bag.activo():
            self.bag.detener(esperar_ms=5000)
        if self.driver.activo():
            self._log("Cerrando: deteniendo driver…")
            self.driver.detener(esperar_ms=6000)
        self.ros.cerrar()
        ev.accept()

    # ------------------------------------------------------------------ registro
    def _log(self, texto, tipo="info"):
        color = {"info": "#b8c4cf", "ok": VERDE, "aviso": AMBAR, "error": ROJO, "bag": "#a5a0ff"}.get(tipo, "#b8c4cf")
        hora = dt.datetime.now().strftime("%H:%M:%S")
        texto = texto.replace("&", "&amp;").replace("<", "&lt;").replace(">", "&gt;")
        self.registro.appendHtml(f"<span style='color:{TEXTO_SUAVE}'>{hora}</span> <span style='color:{color}'>{texto}</span>")
        self.ultimo_log.setText(f"<span style='color:{TEXTO_SUAVE}'>{hora}</span> <span style='color:{color}'>{texto}</span>")
        self.ultimo_log.setToolTip(texto)
        if tipo == "error" and not self.registro.isVisible():
            self.b_registro.setText("☰ Registro •")
            self.b_registro.setProperty("alerta", True)
            self.b_registro.style().polish(self.b_registro)

    def _linea_driver(self, l):
        msg = l.split("]: ", 1)[-1].strip()
        tipo = "error" if ("[ERROR]" in l or "Error" in msg) else ("aviso" if "[WARN]" in l else "info")
        self._log(msg if "]: " in l else l, tipo)
        if m := re.search(r"S/N: (\w+)", msg):
            self.s_sn.setText(m.group(1))
        elif m := re.search(r"Firmware Ver: ([\d.]+)", msg):
            self.s_fw.setText(m.group(1))
        elif m := re.search(r"Hardware Rev: (\d+)", msg):
            self.s_fw.setText(f"{self.s_fw.text()} / rev {m.group(1)}")
        elif "health status : OK" in msg:
            self.s_salud.setText(f"<span style='color:{VERDE}'>OK</span>")
        elif "health status : Warning" in msg:
            self.s_salud.setText(f"<span style='color:{AMBAR}'>Advertencia</span>")
        elif "internal error" in msg:
            self.s_salud.setText(f"<span style='color:{ROJO}'>Error interno — reiniciar LiDAR</span>")
        elif m := re.search(r"current scan mode: (.*)", msg):
            self.s_modo.setText(m.group(1).rstrip(", "))
        elif "operation time out" in msg or "cannot connect" in msg:
            self.insignia.fijar("Sin conexión con el LiDAR", ROJO)

    # ------------------------------------------------------------------ dibujo
    def _sensor(self) -> bool:
        return self.conv.currentIndex() == 1

    def _eje(self) -> tuple[float, float]:
        return (-180.0, 180.0) if self._sensor() else (0.0, 360.0)

    def _angulo_vis(self, ang_ros_rad: np.ndarray) -> np.ndarray:
        d = np.degrees(ang_ros_rad) % 360
        return ros_a_sensor(d) if self._sensor() else d

    def _a_ros_deg(self, a_vis: float) -> float:
        return (180.0 - a_vis) % 360 if self._sensor() else a_vis

    def _xy(self, x, y):
        """Frame ROS -> coordenadas de la vista (giro −90° si el frente va arriba)."""
        return (y, -x) if self.chk_arriba.isChecked() else (x, y)

    def _orientar_vista(self, *_):
        arriba = self.chk_arriba.isChecked()
        titulo = "Vista superior — frente del sensor arriba" if arriba else "Vista superior — frame ROS"
        self.vista.setTitle(f"<span style='color:{TEXTO};font-size:13pt'>{titulo}</span>")
        self.vista.setLabel("bottom", "lateral [m]" if arriba else "x ROS [m]")
        self.vista.setLabel("left", "hacia el frente [m]" if arriba else "y ROS [m]")
        # Flecha justo delante del sensor; ArrowItem: angle 0 apunta a −x, −90 apunta a +y
        self.flecha.setPos(*self._xy(-0.45, 0.0))
        self.flecha.setStyle(angle=-90 if arriba else 0)
        self.lbl_frente.setAnchor((0.0, 0.5) if arriba else (1.0, 0.5))
        self.lbl_frente.setPos(*((0.15, 0.6) if arriba else (-0.75, 0.0)))
        self._ultimo_dibujado = -1
        self._ajustar_vista()

    def _corregir(self, r: np.ndarray) -> np.ndarray:
        if not self.chk_calib.isChecked():
            return r
        s, b = self.calib
        return (r - b) * s

    def _rangos(self, buffer):
        """Rangos a mostrar: último escaneo o mediana temporal de N, ya corregidos."""
        n = min(self.promedio.value(), len(buffer))
        if n <= 1:
            r = buffer[-1].ranges
        else:
            pila = [e.ranges for e in buffer[-n:] if e.ranges.size == buffer[-1].ranges.size]
            with warnings.catch_warnings():
                warnings.simplefilter("ignore", RuntimeWarning)   # columnas todo-NaN
                r = np.nanmedian(np.stack(pila), axis=0)
        return self._corregir(r)

    def _refrescar(self):
        ultimo, buffer, recibidos = self.ros.instantanea()
        ahora = time.monotonic()
        if self.leyendo:
            self.m_tiempo.setText(self._fmt_t(ahora - self.t_inicio))
            if ultimo is None:
                if ahora - self.t_inicio > 15:   # el driver tarda ~3 s y el simulador ~8 s en la Pi
                    self.insignia.fijar("Sin datos en /scan", ROJO)
            elif self.reproductor.activo():
                self.insignia.fijar(("En pausa" if self.reproductor.pausado else "Reproduciendo")
                                    + f" rosbag · {100 * self._progreso_rep:.0f} %", VERDE)
            elif ahora - ultimo.rx > SIN_DATOS_S:
                self.insignia.fijar(f"Sin datos hace {ahora - ultimo.rx:.0f} s", ROJO)
            else:
                self.insignia.fijar("Recibiendo" + (" (driver externo)" if self.driver_externo else ""), VERDE)
        if self.bag.activo():
            self.lbl_bag.setText(f"<span style='color:{ROJO}'>● Grabando</span> {self._fmt_t(ahora - self.t_bag)}<br>"
                                 f"<span style='font-size:11px'>{self.ruta_bag}</span>")

        en_paso = self.pestanas.currentIndex() == 1
        en_calib = self.pestanas.currentWidget() is self.calib
        self.paso.refrescar(congelado=self.b_congelar.isChecked(), dibujar=en_paso)   # eventos siempre
        if ultimo is None or recibidos == self._ultimo_dibujado or self.b_congelar.isChecked():
            return
        self._ultimo_dibujado = recibidos
        self._metricas(buffer, recibidos)
        if en_calib:
            self.calib.refrescar(ultimo)
        if en_paso or en_calib:
            return

        r = self._rangos(buffer)
        ang = ultimo.ang
        self._escaneo_mostrado, self._r_mostrado = ultimo, r
        ok = np.isfinite(r)
        x, y = self._xy(r * np.cos(ang), r * np.sin(ang))
        self.puntos.setData(x[ok], y[ok], brush=self._brochas(r[ok], None if ultimo.intens is None else ultimo.intens[ok]))

        a = self._angulo_vis(ang)
        orden = np.argsort(a)
        self.curva.setData(a[orden], r[orden])

    def _brochas(self, r, intens):
        modo = self.color.currentIndex()
        if modo == 2 or (modo == 1 and intens is None):
            return pg.mkBrush(ACENTO)
        if not hasattr(self, "_lut"):
            lut = pg.colormap.get("viridis").getLookupTable(nPts=64, alpha=False)
            self._lut = np.array([pg.mkBrush(*map(int, c)) for c in lut], dtype=object)
        v = r if modo == 0 else intens
        if v.size == 0:
            return pg.mkBrush(ACENTO)
        lo, hi = (0.0, self.alcance.value()) if modo == 0 else np.percentile(v, [2, 98])
        idx = np.clip(((v - lo) / max(hi - lo, 1e-6) * 63), 0, 63).astype(int)
        return self._lut[idx]

    def _metricas(self, buffer, recibidos):
        e = buffer[-1]
        self.m_cuenta.setText(f"{recibidos}")
        val = np.isfinite(e.ranges)
        self.m_validos.setText(f"{100 * val.mean():.1f} %")
        self._color_metrica(self.m_validos, val.mean() > 0.9, val.mean() > 0.6)
        if val.any():
            self.m_rango.setText(f"{np.nanmin(e.ranges):.2f} / {np.nanmax(e.ranges):.2f} m")
        st = np.array([b.stamp for b in buffer[-21:]])
        if st.size >= 3:
            d = np.diff(st)
            self.m_frec.setText(f"{1 / d.mean():.2f} Hz")
            self.m_jitter.setText(f"{d.std() * 1e3:.2f} ms")
            self._color_metrica(self.m_jitter, d.std() < 0.003, d.std() < 0.01)

    @staticmethod
    def _color_metrica(lbl, bien, regular):
        c = VERDE if bien else (AMBAR if regular else ROJO)
        lbl.setStyleSheet(f"font-size: 12px; font-weight: 700; color: {c};")

    @staticmethod
    def _fmt_t(s):
        s = int(s)
        return f"{s // 3600:02d}:{s % 3600 // 60:02d}:{s % 60:02d}"

    def _dibujar_anillos(self, alcance):
        for it in self.anillos:
            self.vista.removeItem(it)
        self.anillos = []
        paso = next(p for p in (0.25, 0.5, 1, 2, 5, 10) if alcance / p <= 7)
        pluma = pg.mkPen(TEXTO_SUAVE, width=1, style=QtCore.Qt.PenStyle.DotLine)
        r = paso
        while r <= alcance + 1e-9:
            c = QtWidgets.QGraphicsEllipseItem(-r, -r, 2 * r, 2 * r)
            c.setPen(pluma)
            c.setZValue(-20)
            self.vista.addItem(c)
            t = pg.TextItem(f"{r:g} m", color=TEXTO_SUAVE, anchor=(0, 1))
            t.setPos(r * 0.707, r * 0.707)
            self.vista.addItem(t)
            self.anillos += [c, t]
            r += paso

    def _ajustar_vista(self, *_):
        a = self.alcance.value()
        self._dibujar_anillos(a)
        self.vista.setRange(xRange=(-a, a), yRange=(-a, a))
        self.perfil.setYRange(0, a)
        self.perfil.setXRange(*self._eje(), padding=0.01)
        self._dibujar_sector()

    # ------------------------------------------------------------------ cursor
    def _cursor_vista(self, pos):
        vb = self.vista.getPlotItem().vb
        if not self.vista.sceneBoundingRect().contains(pos):
            return
        p = vb.mapSceneToView(pos)
        x, y = (-p.y(), p.x()) if self.chk_arriba.isChecked() else (p.x(), p.y())
        r = np.hypot(x, y)
        a_ros = np.degrees(np.arctan2(y, x)) % 360
        self.lbl_cursor_vista.setText(f"r = {r:.3f} m\nROS {a_ros:.1f}° · sensor {ros_a_sensor(a_ros):+.1f}°")
        self.lbl_cursor_vista.setPos(p.x(), p.y())

    def _cursor_perfil(self, pos):
        vb = self.perfil.getPlotItem().vb
        if not self.perfil.sceneBoundingRect().contains(pos):
            return
        p = vb.mapSceneToView(pos)
        self.cruz.setPos(p.x())
        txt = f"{p.x():.1f}°"
        if self._r_mostrado is not None:
            a = self._angulo_vis(self._escaneo_mostrado.ang)
            i = int(np.argmin(np.abs((a - p.x() + 180) % 360 - 180)))   # distancia angular circular
            r = self._r_mostrado[i]
            txt += f"\n{r:.3f} m" if np.isfinite(r) else "\nsin dato"
        self.lbl_cursor_perfil.setText(txt)
        self.lbl_cursor_perfil.setPos(p.x(), p.y())

    # ------------------------------------------------------------------ sector
    def _sector_movido(self):
        a1, a2 = self.region.getRegion()
        for w, v in ((self.sec_a1, a1), (self.sec_a2, a2)):
            w.blockSignals(True)
            w.setValue(v)
            w.blockSignals(False)
        self._dibujar_sector()

    def _sector_editado(self):
        a1, a2 = sorted((self.sec_a1.value(), self.sec_a2.value()))
        self.region.blockSignals(True)
        self.region.setRegion((a1, a2))
        self.region.blockSignals(False)
        self._dibujar_sector()

    def _conv_cambiada(self, *_, inicial=False):
        lo, hi = self._eje()
        a1, a2 = self.region.getRegion()
        if not inicial:
            # Mismo sector físico en la nueva convención: ROS ↔ sensor es a ↦ 180 − a (involución)
            def norm(a):
                return (a - lo) % 360 + lo
            n1, n2 = norm(180 - a2), norm(180 - a1)
            if n1 > n2:   # cruzaría el borde del eje: se recentra sin partirlo
                c, w = norm(180 - (a1 + a2) / 2), a2 - a1
                n1, n2 = max(lo, c - w / 2), min(hi, c + w / 2)
            a1, a2 = n1, n2
        self.region.setBounds((lo, hi))
        self.region.setRegion((a1, a2))
        self.perfil.getAxis("bottom").setTicks([[(a, f"{a:g}°") for a in range(int(lo), int(hi) + 1, 30)]])
        self.perfil.setLabel("bottom", "ángulo del sensor [°] (0° = frente)" if self._sensor() else "ángulo ROS [°]")
        self.perfil.setXRange(lo, hi, padding=0.01)
        self._ultimo_dibujado = -1

    def _dibujar_sector(self):
        a1, a2 = self.region.getRegion()
        R = self.alcance.value() * 1.5
        vis = np.linspace(a1, a2, 40)
        ros = np.radians([self._a_ros_deg(v) for v in vis])
        poli = QtGui.QPolygonF([QtCore.QPointF(0, 0)] + [QtCore.QPointF(*self._xy(R * np.cos(t), R * np.sin(t))) for t in ros])
        self.sector_poli.setPolygon(poli)

    def _analizar_sector(self):
        if not self.isVisible():
            return
        if self.b_congelar.isChecked() and self._r_mostrado is not None:
            buffer = [self._escaneo_mostrado]
        else:
            _, buffer, _ = self.ros.instantanea()
        if not buffer:
            return
        a1, a2 = self.region.getRegion()
        ang = buffer[-1].ang
        a = self._angulo_vis(ang)
        sel = (a >= a1) & (a <= a2)
        if sel.sum() < 3:
            return
        n = buffer[-1].ranges.size
        R = np.stack([self._corregir(e.ranges[sel]) for e in buffer if e.ranges.size == n])
        with warnings.catch_warnings():
            warnings.simplefilter("ignore", RuntimeWarning)
            ultimo = R[-1]
            sigma_t = np.nanmedian(np.nanstd(R, axis=0)) if R.shape[0] >= 5 else np.nan
            fits = []
            for r in R[-30:]:
                f = ajuste_pared(r * np.cos(ang[sel]), r * np.sin(ang[sel]))
                if f:
                    fits.append(f)
        filas = [("Rayos en el sector", f"{sel.sum()}  ({a2 - a1:.1f}°)"),
                 ("Válidos (último)", f"{100 * np.isfinite(ultimo).mean():.0f} %")]
        if np.isfinite(ultimo).any():
            filas += [("Mediana / mín / máx", f"{np.nanmedian(ultimo):.3f} / {np.nanmin(ultimo):.3f} / {np.nanmax(ultimo):.3f} m")]
        filas.append(("σ temporal por rayo", f"{sigma_t * 1e3:.2f} mm  ({R.shape[0]} esc.)" if np.isfinite(sigma_t) else "— (faltan escaneos)"))
        if fits:
            d = np.array([f[0] for f in fits])
            res = np.mean([f[1] for f in fits])
            nrm = fits[-1][2]
            self.a_dist.setText(f"{d.mean():.4f} m")
            filas += [("σ entre escaneos", f"{d.std() * 1e3:.2f} mm  ({len(fits)} ajustes)"),
                      ("Ruido sobre la recta", f"{res * 1e3:.2f} mm"),
                      ("Normal de la pared", f"ROS {nrm:.1f}° · sensor {ros_a_sensor(nrm):+.1f}°")]
            if self.ref.value() > 0:
                sesgo = d.mean() - self.ref.value()
                c = VERDE if abs(sesgo) < 0.01 else (AMBAR if abs(sesgo) < 0.03 else ROJO)
                filas.append(("Sesgo vs referencia",
                              f"<span style='color:{c}'>{sesgo * 1e3:+.1f} mm ({100 * sesgo / self.ref.value():+.2f} %)</span>"))
            # recta de la pared en la vista superior
            f = fits[-1]
            t = np.radians(f[2])
            nx, ny = np.cos(t), np.sin(t)
            L = 0.5 * f[0] * np.radians(a2 - a1) + 0.2
            px, py = f[0] * nx, f[0] * ny
            xs, ys = self._xy(np.array([px - L * ny, px + L * ny]), np.array([py + L * nx, py - L * nx]))
            self.recta_pared.setData(xs, ys)
        else:
            self.a_dist.setText("—")
            self.recta_pared.setData([], [])
        self.a_tabla.setText("<table cellspacing='3'>" + "".join(
            f"<tr><td style='color:{TEXTO_SUAVE}'>{k}</td><td style='padding-left:8px'><b>{v}</b></td></tr>"
            for k, v in filas) + "</table>")

    # ------------------------------------------------------------------ grabación y capturas
    def _grabar(self, on):
        if on:
            if not self.leyendo:
                self.b_grabar.setChecked(False)
                return
            CARPETA_BAGS.mkdir(parents=True, exist_ok=True)
            nombre = re.sub(r"[^\w.-]+", "_", self.nombre_bag.text().strip()) or "scan"
            self.ruta_bag = CARPETA_BAGS / f"{nombre}_{dt.datetime.now():%Y%m%d_%H%M%S}"
            self.bag.iniciar(["ros2", "bag", "record", "--topics", "/scan", "-o", str(self.ruta_bag)])
            self.t_bag = time.monotonic()
            self.b_grabar.setText("■  Detener grabación")
            self._log(f"Grabando /scan en {self.ruta_bag}", "bag")
        else:
            self.bag.detener()
            self.b_grabar.setText("●  Grabar rosbag")

    def _bag_termino(self, codigo):
        if self.b_grabar.isChecked():
            self.b_grabar.blockSignals(True)
            self.b_grabar.setChecked(False)
            self.b_grabar.setText("●  Grabar rosbag")
            self.b_grabar.blockSignals(False)
        dur = time.monotonic() - self.t_bag
        self.lbl_bag.setText(f"Última: {self.ruta_bag.name} ({dur:.0f} s)")
        self._log(f"Grabación cerrada ({dur:.0f} s): {self.ruta_bag}", "bag" if codigo == 0 else "error")

    def _captura(self):
        e, r = self._escaneo_mostrado, self._r_mostrado
        if e is None:
            QtWidgets.QMessageBox.information(self, "Captura", "Todavía no hay ningún escaneo que guardar.")
            return
        CARPETA_CAPTURAS.mkdir(parents=True, exist_ok=True)
        base = CARPETA_CAPTURAS / f"escaneo_{dt.datetime.now():%Y%m%d_%H%M%S}"
        with open(base.with_suffix(".csv"), "w", newline="") as fh:
            w = csv.writer(fh)
            w.writerow([f"# stamp={e.stamp:.6f}", f"calibracion={'si' if self.chk_calib.isChecked() else 'no'}",
                        f"mediana_de={self.promedio.value()}"])
            w.writerow(["indice", "angulo_ros_deg", "angulo_sensor_deg", "distancia_m", "x_m", "y_m", "intensidad"])
            for i, (a, d) in enumerate(zip(np.degrees(e.ang) % 360, r)):
                ok = np.isfinite(d)
                w.writerow([i, f"{a:.4f}", f"{ros_a_sensor(a):.4f}",
                            f"{d:.4f}" if ok else "", f"{d * np.cos(np.radians(a)):.4f}" if ok else "",
                            f"{d * np.sin(np.radians(a)):.4f}" if ok else "",
                            "" if e.intens is None else f"{e.intens[i]:.0f}"])
        self.centralWidget().grab().save(str(base.with_suffix(".png")))
        self._log(f"Captura guardada: {base}.csv / .png", "ok")
        self.statusBar().showMessage(f"Captura guardada en {base}.csv y .png", 6000)

    def _ayuda(self):
        QtWidgets.QMessageBox.information(self, "Cómo usar el visor", f"""
<b>Iniciar</b> (F5): comprueba que el LiDAR responde, lanza el driver sllidar_ros2 y muestra /scan.
Si ya hay un driver corriendo en una terminal, el visor se conecta a él sin lanzar otro.<br>
<b>Detener</b> (F6): para el driver y el motor del LiDAR.<br>
<b>Congelar</b> (Espacio): fija la imagen para mirarla con calma; los datos siguen llegando.<br><br>
<b>Vista superior</b>: puntos en metros, vistos desde arriba. Por defecto el frente del sensor
(marca 0°, flecha ámbar) apunta hacia arriba; desmarcándolo se ve el frame ROS tal cual.<br>
<b>Distancia vs ángulo</b>: cada rayo del giro. Los huecos son rayos sin retorno.
En ángulo del sensor el eje va de −180° a 180° con el frente en el centro (sensor = 180° − ROS).<br><br>
<b>Análisis de sector</b>: arrastra la franja sobre una pared. El visor ajusta una recta
en cada escaneo (últimos 3 s) y da la distancia perpendicular, su variación y el ruido.
Si escribes la distancia de referencia (cinta), calcula el sesgo.<br>
<b>Corregir rango</b>: aplica la calibración de velocity.yaml (r − sesgo)·escala.<br>
<b>Mediana de N</b>: suaviza el ruido promediando escaneos; úsalo solo con la escena quieta.<br><br>
<b>Prueba de paso</b> (pestaña): corre el estimador de velocidad y la máquina de estados de paso
(docs/maquina_estados_paso.md) sobre cada escaneo. Muestra el plano de escaneo con la ROI, el perfil h(x)
y el objeto detectado; la velocidad en km/h; y una fila por paso con sus banderas de calidad.
Escribe la velocidad de referencia de cada paso para obtener el error.
Se miden ambos sentidos: el signo de la velocidad y la columna «Sentido» dicen hacia dónde fue cada vehículo
(por defecto positivo = hacia +x, →). «Invertir referencia» cambia el signo sin reiniciar nada. Antes de probar, ajusta el
montaje con la pestaña Calibración y guarda en velocity.yaml si los valores sirven.
Con la fuente «Rosbag grabado» se repite un paso con otros parámetros.<br><br><b>Calibración</b> (pestaña): con la altura medida con cinta y el piso vacío, ajusta la recta del piso y da
la altura perpendicular, el rayo hacia abajo y la inclinación (sensor + pendiente del piso, compensadas juntas
porque el vehículo avanza paralelo al piso). Con un objeto del lado de entrada detecta el sentido de avance.
Guarda en velocity.yaml y deja una ficha en {DATOS / "calibraciones"}.<br><br>
<b>Pantalla</b>: F11 alterna pantalla completa; F2 muestra el registro (la última línea queda abajo).<br><br>
Rosbags: {CARPETA_BAGS}<br>Capturas: {CARPETA_CAPTURAS}<br>Pasos: {DATOS / "pruebas_paso"}""")


def main():
    os.environ.setdefault("QT_QPA_PLATFORM", "wayland;xcb")
    app = QtWidgets.QApplication(sys.argv)
    app.setApplicationName("Visor LiDAR")
    app.setDesktopFileName("falconscan-visor-lidar")
    app.setStyle("Fusion")
    app.setStyleSheet(ESTILO)
    icono = Path(__file__).with_name("icono.svg")
    if icono.exists():
        app.setWindowIcon(QtGui.QIcon(str(icono)))
    win = VentanaPrincipal()
    if win.compacto:          # pantalla de 7": sin bordes de ventana ni barra del escritorio
        win.showFullScreen()
    else:
        g = app.primaryScreen().availableGeometry()
        win.resize(min(1500, g.width()), min(900, g.height()))
        win.show()
    # rclpy instala sus propios manejadores (solo apagan ROS); cerrar la ventana detiene todo en orden
    signal.signal(signal.SIGINT, lambda *_: win.close())
    signal.signal(signal.SIGTERM, lambda *_: win.close())
    t = QtCore.QTimer(interval=300, timeout=lambda: None)   # deja que Python atienda Ctrl+C
    t.start()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()
