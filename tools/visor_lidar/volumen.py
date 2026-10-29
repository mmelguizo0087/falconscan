"""Pestaña «Volumen 3D»: el reconstructor de volumen (falconscan_volume) dentro del visor.

Corre el mismo núcleo que los nodos: ``VelocityPipeline`` (estimador + PassTracker) con el LiDAR
de velocidad y ``VolumeReconstructor`` con el transversal, conectados igual que en
``falconscan_volume/sim/scenario.py`` y ``volume_node``. Fuentes:

- Simulador del pórtico: los escenarios de tools/evaluate_volume_sim.py o uno personalizado. Se
  generan una vez con sim_volumen.py (en otro proceso; ~1 s por escaneo en la RPi) y se
  reproducen desde la caché, con el volumen VERDADERO para calcular el error.
- ROS en vivo: dos tópicos LaserScan (por defecto los de portico.launch.py / volume_sim.launch.py).
- Rosbag con los dos tópicos.

Vistas: nube y superficie 3D (OpenGL; sin PyOpenGL queda la vista superior), superficie h(x, y)
vista desde arriba, áreas A(x) de la medición y el perfil transversal en vivo. Marco del paso
(docs/maquina_estados_volumen.md): x = 0 en el frente y negativo hacia la cola, y = izquierda
del camión, z = altura sobre el piso.

Los resultados con el simulador NO son evidencia de desempeño de campo.
"""
from __future__ import annotations

import csv
import datetime as dt
import hashlib
import json
import math
import sys
import threading
import time
from pathlib import Path

import numpy as np
import pyqtgraph as pg
import yaml
from pyqtgraph.Qt import QtCore, QtGui, QtWidgets

from comun import AMBAR, BORDE, PANEL, ROJO, TEXTO_SUAVE, VERDE, pantalla_compacta, titulo_grafico

REPO = Path(__file__).resolve().parents[2]
try:
    from falconscan_volume.core import VolumeParams, VolumeReconstructor
except ImportError:   # sin `source install/setup.bash`: usar el código fuente del repo
    sys.path[:0] = [str(REPO / "src" / "falconscan_volume"), str(REPO / "src" / "falconscan_velocity")]
    from falconscan_volume.core import VolumeParams, VolumeReconstructor
from falconscan_velocity.core import PassState, Scan, VelocityParams, VelocityPipeline, polar_to_xz, validate_ranges
from falconscan_velocity.sim import SIM_VELOCITY_OVERRIDES
from falconscan_volume.sim import SIM_VOLUME_OVERRIDES
from falconscan_volume.core.odometry import VelocitySample
from falconscan_volume.core.reconstruction import Reconstruction, surface_triangles, write_json, write_ply
from falconscan_volume.core.reconstructor import CaptureStarted, CloudUpdate, Measurement, VolState

try:
    import pyqtgraph.opengl as gl
    ERROR_GL = ""
except Exception as e:   # noqa: BLE001  (falta python3-opengl)
    gl = None
    ERROR_GL = str(e)

KMH = 3.6
SIM_VOLUMEN = Path(__file__).with_name("sim_volumen.py")
TOPICO_VEL, TOPICO_VOL = "/lidar_velocity/scan", "/lidar_volume/scan"
CMAP = pg.colormap.get("viridis")
H_COLOR_MAX = 3.5          # extremo de la escala de colores [m] cuando aún no hay objeto
H_COLOR_MIN = 0.1          # escala mínima [m]: un objeto casi plano no satura el color con ruido

ESTADOS_VOL = {
    VolState.ARRANQUE: ("ARRANQUE", AMBAR), VolState.VACIO: ("VACÍO", VERDE),
    VolState.DETECTANDO: ("DETECTANDO", "#2fb8c4"), VolState.CAPTURANDO: ("CAPTURANDO", "#58a6ff"),
    VolState.HUECO: ("HUECO", "#a371f7"), VolState.CERRANDO: ("CERRANDO", "#a371f7"),
    VolState.INTEGRANDO: ("INTEGRANDO", "#58a6ff"), VolState.OBSTRUIDO: ("OBSTRUIDO", ROJO),
    VolState.SIN_DATOS: ("SIN DATOS", ROJO), VolState.SIN_VELOCIDAD: ("SIN VELOCIDAD", ROJO),
}
ESTADOS_VEL = {
    PassState.ARRANQUE: ("ARRANQUE", AMBAR), PassState.VACIO: ("VACÍO", VERDE),
    PassState.ENTRANDO: ("ENTRANDO", "#2fb8c4"), PassState.EN_PASO: ("EN PASO", "#58a6ff"),
    PassState.SALIENDO: ("SALIENDO", "#a371f7"), PassState.OBSTRUIDO: ("OBSTRUIDO", ROJO),
    PassState.SIN_DATOS: ("SIN DATOS", ROJO),
}
# Con un objeto bajo el pórtico no se redibuja nada (salvo la vista en vivo opcional): el GUI
# compite por la CPU (y el GIL) con el procesamiento en la RPi. Se dibuja al terminar el paso.
EN_PASO_VOL = {VolState.DETECTANDO, VolState.CAPTURANDO, VolState.HUECO, VolState.CERRANDO, VolState.INTEGRANDO}
EN_PASO_VEL = {PassState.ENTRANDO, PassState.EN_PASO, PassState.SALIENDO}
PERFIL_REPOSO_S = 0.25     # perfil transversal sin objeto: ~4 Hz alcanza para revisar el montaje
RESULTADO = {0: ("VÁLIDA", VERDE), 1: ("ADVERTENCIAS", AMBAR), 2: ("INVÁLIDA", ROJO), 3: ("DESCARTADA", TEXTO_SUAVE)}
COLUMNAS = ["#", "Paso", "Sentido", "Resultado", "V [m³]", "±", "Verdad", "Error", "Largo", "Ancho", "Alto",
            "v avance", "Perfiles", "Banderas"]
(C_ID, C_PASO, C_SENTIDO, C_RES, C_V, C_INC, C_VERDAD, C_ERR, C_LARGO, C_ANCHO, C_ALTO, C_VAV, C_PERF,
 C_BANDERAS) = range(len(COLUMNAS))


# ----------------------------------------------------------------------------- parámetros

def cargar_params(ruta: Path) -> tuple[VelocityParams, VolumeParams]:
    """Secciones velocity_node y volume_node de config/falconscan.yaml (nombres 1:1 con las clases)."""
    datos = yaml.safe_load(ruta.read_text())
    sv, sw = datos["velocity_node"]["ros__parameters"], datos["volume_node"]["ros__parameters"]
    nv, nw = set(VelocityParams.field_names()), set(VolumeParams.field_names())
    return (VelocityParams(**{k: v for k, v in sv.items() if k in nv}),
            VolumeParams(**{k: v for k, v in sw.items() if k in nw}))


def escala_altura(rec: Reconstruction) -> float:
    """Extremo de la escala de colores: la altura máxima medida del objeto (malla h(x, y); si no hay
    malla, la nube). Así el color usa todo el rango para cualquier objeto."""
    for a in (rec.surface_h, rec.points[:, 3] if len(rec.points) else np.empty(0)):
        if a.size and np.isfinite(a).any():
            return max(float(np.nanmax(a)), H_COLOR_MIN)
    return H_COLOR_MAX


def colores_altura(h: np.ndarray, alfa: float = 1.0, h_max: float = H_COLOR_MAX) -> np.ndarray:
    """RGBA en [0, 1] según la altura (viridis de 0 a h_max)."""
    c = CMAP.map(np.clip(np.nan_to_num(h, nan=0.0) / h_max, 0, 1), mode="float")
    c[:, 3] = alfa
    return c


# ----------------------------------------------------------------------------- procesamiento

class ProcesoVolumen:
    """Estimador + PassTracker + reconstructor, conectados como en volume_node (sin Qt).

    ``ahora`` es el reloj de llegada: el monotónico en vivo, o el instante de entrega del dato
    en una reproducción (simulador o rosbag), que así corre con su propio tiempo.
    """

    def __init__(self, vparams: VelocityParams, volparams: VolumeParams):
        self._lock = threading.Lock()
        self.configurar(vparams, volparams)

    def configurar(self, vparams: VelocityParams, volparams: VolumeParams):
        with self._lock:
            self.vp, self.wp = vparams, volparams
            self.pipe = VelocityPipeline(vparams)
            self.rec = VolumeReconstructor(volparams)
            self.nube: CloudUpdate | None = None
            self.nube_nueva = False
            self.mediciones: list[Measurement] = []
            self.capturas: list[CaptureStarted] = []
            self.ultimo_vol: tuple[Scan, object] | None = None
            self.ultimo_vel = None
            self.n_vel = self.n_vol = 0
            self.t_proc: list[float] = []

    def velocidad(self, scan: Scan, ahora: float | None):
        t1 = time.perf_counter()
        with self._lock:
            est, eventos = self.pipe.process(scan, ahora)
            out = []
            for ev in eventos:
                out += self.rec.on_pass_event(ev)
            x0, x1 = est.object_extent if est.object_extent else (math.nan, math.nan)
            tr = self.pipe.tracker
            s = VelocitySample(scan.stamp, est.mid_stamp if est.valid else math.nan, est.velocity_mps, est.valid,
                               x0, x1, pass_id=tr.current_pass_id, direction=tr.direction)
            out += self.rec.on_velocity(s, now=ahora)
            self._recoger(out)
            self.ultimo_vel = est
            self.n_vel += 1
            self.t_proc.append(time.perf_counter() - t1)

    def volumen(self, scan: Scan, ahora: float | None):
        t1 = time.perf_counter()
        with self._lock:
            self._recoger(self.rec.on_scan(scan, now=ahora))
            self.ultimo_vol = (scan, self.rec.last_section)
            self.n_vol += 1
            self.t_proc.append(time.perf_counter() - t1)

    def tick(self, ahora: float):
        with self._lock:
            out = []
            for ev in self.pipe.tick(ahora):
                out += self.rec.on_pass_event(ev)
            self._recoger(out + self.rec.tick(ahora))

    def _recoger(self, outs):
        for o in outs:
            if isinstance(o, CloudUpdate):
                self.nube, self.nube_nueva = o, True
            elif isinstance(o, Measurement):
                self.mediciones.append(o)
            elif isinstance(o, CaptureStarted):
                self.capturas.append(o)

    def tomar(self) -> dict:
        with self._lock:
            med, self.mediciones = self.mediciones, []
            cap, self.capturas = self.capturas, []
            nube = self.nube if self.nube_nueva else None
            self.nube_nueva = False
            tp, self.t_proc = self.t_proc, []
            r, tr = self.rec, self.pipe.tracker
            return dict(mediciones=med, capturas=cap, nube=nube, ultimo_vol=self.ultimo_vol,
                        estado_vel=tr.state, estado_vol=r.state, listo=tr.ready and r.ready,
                        parcial=r.partial_volume_m3, perfiles=len(r.cap.sections) if r.cap else 0,
                        n_vel=self.n_vel, n_vol=self.n_vol, t_proc=float(np.mean(tp)) if tp else math.nan,
                        v=self.ultimo_vel.velocity_mps if self.ultimo_vel is not None and self.ultimo_vel.valid
                        else math.nan)


# ----------------------------------------------------------------------------- reproducción

class Reproductor(QtCore.QObject):
    """Entrega escaneos (simulador en caché o rosbag) con su temporización × velocidad.

    ``fuente`` es un iterable de (t_entrega, "vel"|"vol", Scan) en orden temporal.
    """
    termino = QtCore.Signal(str)
    progreso = QtCore.Signal(float)

    def __init__(self, proc: ProcesoVolumen, parent=None):
        super().__init__(parent)
        self.proc = proc
        self.velocidad = 1.0
        self.pausado = False
        self._parar = threading.Event()
        self._hilo = None

    def iniciar(self, fuente, total: int):
        self.detener()
        self._parar.clear()
        self._hilo = threading.Thread(target=self._correr, args=(fuente, total), daemon=True)
        self._hilo.start()

    def detener(self):
        self._parar.set()
        if self._hilo:
            self._hilo.join(2.0)

    def activo(self) -> bool:
        return self._hilo is not None and self._hilo.is_alive()

    def _correr(self, fuente, total):
        try:
            t_dato0 = t_real0 = None
            t = 0.0
            for i, (t, tipo, scan) in enumerate(fuente):
                if self._parar.is_set():
                    break
                while self.pausado and not self._parar.is_set():
                    time.sleep(0.05)
                    t_dato0 = None
                if t_dato0 is None:
                    t_dato0, t_real0 = t, time.monotonic()
                if math.isfinite(self.velocidad):
                    espera = (t - t_dato0) / self.velocidad - (time.monotonic() - t_real0)
                    if espera > 0:
                        self._parar.wait(espera)
                else:
                    time.sleep(0.001)          # «máx.»: deja respirar al GUI
                self.proc.tick(t)
                (self.proc.velocidad if tipo == "vel" else self.proc.volumen)(scan, t)
                if i % 10 == 0:
                    self.progreso.emit((i + 1) / max(1, total))
            # sin escaneos nuevos el reloj sigue: cierran las esperas (como scenario.run_scenario)
            for k in range(1, 61):
                if self._parar.is_set():
                    break
                self.proc.tick(t + 0.1 * k)
            self.progreso.emit(1.0)
            self.termino.emit("" if not self._parar.is_set() else "detenido")
        except Exception as e:   # noqa: BLE001  (se informa en el GUI)
            self.termino.emit(f"Error en la reproducción: {e}")


def escaneos_npz(d) -> tuple[list, int]:
    """(t_entrega, tipo, Scan) de un paso generado por sim_volumen.py, en orden de entrega."""
    out = []
    for tipo in ("vel", "vol"):
        a0, inc, tinc, rmin, rmax = d[f"{tipo}_geo"]
        for t, s, r in zip(d[f"{tipo}_entrega"], d[f"{tipo}_stamp"], d[f"{tipo}_ranges"]):
            out.append((float(t), tipo, Scan(stamp=float(s), angle_min=float(a0), angle_increment=float(inc),
                                             ranges=r, time_increment=float(tinc), range_min=float(rmin),
                                             range_max=float(rmax))))
    out.sort(key=lambda e: e[0])
    return out, len(out)


def scan_de_msg(msg) -> Scan:
    return Scan(stamp=msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9, angle_min=msg.angle_min,
                angle_increment=msg.angle_increment, ranges=np.asarray(msg.ranges, dtype=np.float32),
                time_increment=msg.time_increment, range_min=msg.range_min, range_max=msg.range_max)


def topicos_bag(ruta: str) -> list[str]:
    from rosbag2_py import ConverterOptions, SequentialReader, StorageOptions
    rd = SequentialReader()
    rd.open(StorageOptions(uri=ruta, storage_id=""), ConverterOptions("", ""))
    return [t.name for t in rd.get_all_topics_and_types() if t.type.endswith("LaserScan")]


def escaneos_bag(ruta: str, topico_vel: str, topico_vol: str):
    """Generador (t_bag, tipo, Scan) con los dos tópicos en el orden del rosbag."""
    from rclpy.serialization import deserialize_message
    from rosbag2_py import ConverterOptions, SequentialReader, StorageFilter, StorageOptions
    from sensor_msgs.msg import LaserScan
    rd = SequentialReader()
    rd.open(StorageOptions(uri=ruta, storage_id=""), ConverterOptions("", ""))
    rd.set_filter(StorageFilter(topics=[topico_vel, topico_vol]))
    while rd.has_next():
        tema, data, t_ns = rd.read_next()
        yield t_ns * 1e-9, "vel" if tema == topico_vel else "vol", scan_de_msg(deserialize_message(data, LaserScan))


def contar_bag(ruta: str, topicos: list[str]) -> int:
    from rosbag2_py import ConverterOptions, SequentialReader, StorageOptions
    rd = SequentialReader()
    rd.open(StorageOptions(uri=ruta, storage_id=""), ConverterOptions("", ""))
    return sum(t.message_count for t in rd.get_metadata().topics_with_message_count
               if t.topic_metadata.name in topicos)


# ----------------------------------------------------------------------------- panel

def _etiqueta(texto, tip=""):
    l = QtWidgets.QLabel(texto)
    l.setObjectName("suave")
    if tip:
        l.setToolTip(tip)
    return l


class PanelVolumen(QtWidgets.QWidget):
    """Contenido de la pestaña «Volumen 3D»."""

    def __init__(self, ventana, yaml_ruta: Path, carpeta_datos: Path):
        super().__init__()
        self.v = ventana
        self.yaml_ruta = yaml_ruta
        self.carpeta = carpeta_datos / "pruebas_volumen"
        self.carpeta_bags = carpeta_datos / "rosbags"
        self.cache = carpeta_datos / "cache_sim_portico"
        self.vparams, self.wparams = cargar_params(yaml_ruta)
        self.proc = ProcesoVolumen(self.vparams, self.wparams)
        self.reproductor = Reproductor(self.proc, self)
        self.reproductor.termino.connect(self._fin_reproduccion)
        self.reproductor.progreso.connect(lambda f: self.barra.setValue(int(1000 * f)))
        self.generador = QtCore.QProcess(self)
        self.generador.setProcessChannelMode(QtCore.QProcess.ProcessChannelMode.MergedChannels)
        self.generador.readyReadStandardOutput.connect(self._salida_generador)
        self.generador.finished.connect(self._fin_generador)
        self._subs = []
        self.leyendo = False
        self.verdad_m3 = math.nan            # volumen verdadero del escenario simulado
        self.esperado = ""
        self.mediciones: list[Measurement] = []
        self.registro: Path | None = None
        self._rec_vista: Reconstruction | None = None
        # Valores EFECTIVOS de la pestaña «Calibración» para cada LiDAR (no van al yaml)
        self.calibracion: dict[str, dict] = {}
        self._en_vivo_dibujado = 0.0
        self._perfil_dibujado = 0.0
        self._estados_pintados = None
        self._construir()
        self.t_vivo = QtCore.QTimer(self, interval=100, timeout=self._tick_vivo)

    # ------------------------------------------------------------------ construcción
    def _construir(self):
        raiz = QtWidgets.QVBoxLayout(self)
        raiz.setContentsMargins(0, 6, 0, 0)
        raiz.setSpacing(4)

        # --- fila 1: fuente y control
        f1 = QtWidgets.QHBoxLayout()
        self.c_fuente = QtWidgets.QComboBox()
        self.c_fuente.addItems(["Simulador del pórtico", "ROS: dos LiDAR", "Rosbag (dos LiDAR)…"])
        self.c_fuente.setToolTip("Simulador: escenarios de tools/evaluate_volume_sim.py (se generan una vez y quedan\n"
                                 "en caché; ~1 s por escaneo en la RPi). ROS: los tópicos de portico.launch.py o\n"
                                 "volume_sim.launch.py. Rosbag: un bag grabado con los dos tópicos.")
        self.c_fuente.currentIndexChanged.connect(self._fuente_cambiada)
        f1.addWidget(self.c_fuente)
        self.c_escenario = QtWidgets.QComboBox()
        self.c_escenario.setMinimumContentsLength(16)
        self.c_escenario.setSizeAdjustPolicy(QtWidgets.QComboBox.SizeAdjustPolicy.AdjustToMinimumContentsLengthWithIcon)
        try:
            from sim_volumen import PERSONALIZADO, nombres
            self.c_escenario.addItems(nombres() + [PERSONALIZADO])
        except Exception as e:   # noqa: BLE001
            self.c_escenario.addItem(f"(sin escenarios: {e})")
        self.c_escenario.setToolTip("Escenario del simulador (los mismos de la matriz de regresión).\n"
                                    "«Personalizado» usa los valores de la fila de abajo.")
        self.c_escenario.currentTextChanged.connect(self._escenario_cambiado)
        f1.addWidget(self.c_escenario, 1)
        self.w_topicos = QtWidgets.QWidget()
        lt = QtWidgets.QHBoxLayout(self.w_topicos)
        lt.setContentsMargins(0, 0, 0, 0)
        self.e_vel, self.e_vol = QtWidgets.QLineEdit(TOPICO_VEL), QtWidgets.QLineEdit(TOPICO_VOL)
        for e, nombre in ((self.e_vel, "velocidad"), (self.e_vol, "volumen")):
            e.setToolTip(f"Tópico LaserScan del LiDAR de {nombre}")
            lt.addWidget(_etiqueta(nombre[:3] + "."))
            lt.addWidget(e, 1)
        f1.addWidget(self.w_topicos, 2)
        self.c_vel = QtWidgets.QComboBox()
        self.c_vel.addItems(["1×", "2×", "5×", "máx."])
        self.c_vel.setToolTip("Velocidad de reproducción (simulador y rosbag)")
        self.c_vel.currentIndexChanged.connect(self._vel_reproduccion)
        f1.addWidget(self.c_vel)
        self.b_iniciar = QtWidgets.QPushButton("▶ Medir", objectName="iniciar")
        self.b_iniciar.setToolTip("Genera (si no está en caché) y reproduce el escenario, o se suscribe a los tópicos")
        self.b_iniciar.clicked.connect(self.iniciar)
        self.b_detener = QtWidgets.QPushButton("■", objectName="detener")
        self.b_detener.setFixedWidth(34)
        self.b_detener.setToolTip("Detener")
        self.b_detener.clicked.connect(self.detener)
        self.b_pausa = QtWidgets.QPushButton("❚❚", checkable=True)
        self.b_pausa.setFixedWidth(34)
        self.b_pausa.setToolTip("Pausa la reproducción")
        self.b_pausa.toggled.connect(lambda on: setattr(self.reproductor, "pausado", on))
        for b in (self.b_iniciar, self.b_pausa, self.b_detener):
            f1.addWidget(b)
        self.b_params = QtWidgets.QPushButton("⚙", checkable=True)
        self.b_params.setFixedWidth(34)
        self.b_params.setToolTip("Escenario personalizado y parámetros del reconstructor")
        f1.addWidget(self.b_params)
        raiz.addLayout(f1)

        # --- fila 2 (plegable): escenario personalizado + parámetros de la vista
        self.w_params = QtWidgets.QWidget()
        f2 = QtWidgets.QHBoxLayout(self.w_params)
        f2.setContentsMargins(0, 0, 0, 0)
        self.s_kmh = QtWidgets.QDoubleSpinBox(minimum=2.0, maximum=20.0, value=7.5, singleStep=0.5, suffix=" km/h")
        self.c_sentido = QtWidgets.QComboBox()
        self.c_sentido.addItems(["+x", "−x"])
        self.s_lat = QtWidgets.QDoubleSpinBox(minimum=-1.5, maximum=1.5, value=0.0, singleStep=0.1, suffix=" m")
        self.s_lat.setToolTip("Desplazamiento lateral del camión respecto al eje del carril")
        self.s_yaw = QtWidgets.QDoubleSpinBox(minimum=-15.0, maximum=15.0, value=0.0, singleStep=1.0, suffix=" °")
        self.s_yaw.setToolTip("Ángulo de la trayectoria (recta en diagonal)")
        for t, w in (("Personalizado:", self.s_kmh), ("", self.c_sentido), ("lateral", self.s_lat),
                     ("guiñada", self.s_yaw)):
            if t:
                f2.addWidget(_etiqueta(t))
            f2.addWidget(w)
        f2.addSpacing(12)
        self.chk_bordes = QtWidgets.QCheckBox("Bordes por el LiDAR de velocidad")
        self.chk_bordes.setToolTip("vol_edge_from_velocity: frente y cola ubicados con el LiDAR de velocidad\n"
                                   "(si no, punto medio entre perfiles). Aplica al iniciar.")
        self.chk_bordes.setChecked(self.wparams.vol_edge_from_velocity)
        f2.addWidget(self.chk_bordes)
        self.chk_durante = QtWidgets.QCheckBox("Dibujar durante el paso")
        self.chk_durante.setToolTip("Nube provisional y perfil transversal mientras pasa el objeto (vol_cloud_live).\n"
                                    "Apagado (recomendado en la RPi): no se reconstruye ni se dibuja nada hasta que\n"
                                    "termina el paso. La medición no cambia. Aplica al iniciar.")
        f2.addWidget(self.chk_durante)
        f2.addStretch(1)
        b_cache = QtWidgets.QPushButton("Vaciar caché")
        b_cache.setToolTip(f"Borra los pasos simulados guardados en {self.cache}")
        b_cache.clicked.connect(self._vaciar_cache)
        f2.addWidget(b_cache)
        self.b_params.toggled.connect(self.w_params.setVisible)
        self.w_params.setVisible(False)
        raiz.addWidget(self.w_params)

        # --- fila 3: estado
        f3 = QtWidgets.QHBoxLayout()
        self.l_vel = QtWidgets.QLabel()
        self.l_vol = QtWidgets.QLabel()
        self.l_listo = QtWidgets.QLabel()
        self.l_parcial = QtWidgets.QLabel()
        self.l_info = QtWidgets.QLabel()
        self.l_info.setObjectName("suave")
        # cede el espacio a los estados: en 1024 px se recorta este texto, no los estados
        self.l_info.setSizePolicy(QtWidgets.QSizePolicy.Policy.Ignored, QtWidgets.QSizePolicy.Policy.Preferred)
        negrita = QtGui.QFont(self.font())
        negrita.setBold(True)
        fm = QtGui.QFontMetrics(negrita)
        for w, mas_largo in ((self.l_vel, "Velocidad: SIN DATOS"), (self.l_vol, "Volumen: SIN VELOCIDAD"),
                             (self.l_listo, "● no listo"), (self.l_parcial, "V parcial 999.99 m³ · 999 perfiles")):
            w.setMinimumWidth(fm.horizontalAdvance(mas_largo) + 4)
            f3.addWidget(w)
        f3.addWidget(self.l_info, 1)
        self.barra = QtWidgets.QProgressBar(maximum=1000, textVisible=False)
        self.barra.setFixedWidth(140)
        self.barra.setFixedHeight(10)
        f3.addWidget(self.barra)
        raiz.addLayout(f3)

        # --- zona central
        div = QtWidgets.QSplitter(QtCore.Qt.Orientation.Horizontal)
        self.vistas = QtWidgets.QTabWidget()
        self.vistas.setDocumentMode(True)
        self.vistas.addTab(self._construir_3d(), "3D")
        self.vistas.addTab(self._construir_superior(), "Superficie (arriba)")
        self.vistas.addTab(self._construir_areas(), "Áreas A(x)")
        div.addWidget(self.vistas)

        der = QtWidgets.QSplitter(QtCore.Qt.Orientation.Vertical)
        self.g_perfil = pg.PlotWidget()
        self.g_perfil.setTitle(titulo_grafico("Perfil transversal (en vivo)"))
        self.g_perfil.setMouseEnabled(x=False, y=False)
        self.g_perfil.showGrid(x=True, y=True, alpha=0.2)
        self.g_perfil.setLabel("bottom", "y [m] (+ izquierda del eje)")
        self.g_perfil.setLabel("left", "z [m]")
        p = self.wparams
        self.roi_perfil = QtWidgets.QGraphicsRectItem(p.vol_roi_y_min_m, p.floor_z_m,
                                                      p.vol_roi_y_max_m - p.vol_roi_y_min_m, p.vol_roi_z_max_m)
        self.roi_perfil.setPen(pg.mkPen("#58a6ff", width=1, style=QtCore.Qt.PenStyle.DashLine))
        self.g_perfil.addItem(self.roi_perfil)
        self.umbral_perfil = pg.InfiniteLine(pos=p.floor_z_m + p.vol_h_threshold_m, angle=0,
                                             pen=pg.mkPen(AMBAR, width=1, style=QtCore.Qt.PenStyle.DotLine))
        self.g_perfil.addItem(self.umbral_perfil)
        self.ext_perfil = pg.LinearRegionItem(movable=False, brush=pg.mkBrush(88, 166, 255, 35), pen=pg.mkPen(None))
        self.g_perfil.addItem(self.ext_perfil)
        self.pts_perfil = pg.ScatterPlotItem(size=3, pen=None, brush=pg.mkBrush("#8b98a5"))
        self.g_perfil.addItem(self.pts_perfil)
        self.h_perfil = pg.PlotDataItem(pen=pg.mkPen("#2fb8c4", width=2), stepMode="center")
        self.g_perfil.addItem(self.h_perfil)
        self.txt_perfil = pg.TextItem("", color="#e6edf3", anchor=(0, 0))
        self.g_perfil.addItem(self.txt_perfil)
        self._actualizar_perfil_estatico(p)
        der.addWidget(self.g_perfil)

        caja = QtWidgets.QWidget()
        lc = QtWidgets.QVBoxLayout(caja)
        lc.setContentsMargins(0, 0, 0, 0)
        self.tabla = QtWidgets.QTableWidget(0, len(COLUMNAS))
        self.tabla.setHorizontalHeaderLabels(COLUMNAS)
        self.tabla.verticalHeader().setVisible(False)
        self.tabla.setSelectionBehavior(QtWidgets.QAbstractItemView.SelectionBehavior.SelectRows)
        self.tabla.setSelectionMode(QtWidgets.QAbstractItemView.SelectionMode.SingleSelection)
        self.tabla.setEditTriggers(QtWidgets.QAbstractItemView.EditTrigger.NoEditTriggers)
        self.tabla.horizontalHeader().setStretchLastSection(True)
        self.tabla.setStyleSheet(f"QTableWidget {{ background: #0f1318; gridline-color: {BORDE}; border: 1px solid {BORDE}; }}"
                                 f"QHeaderView::section {{ background: {PANEL}; color: {TEXTO_SUAVE}; border: none; padding: 4px; }}")
        self.tabla.itemSelectionChanged.connect(self._seleccion)
        lc.addWidget(self.tabla, 1)
        fb = QtWidgets.QHBoxLayout()
        self.chk_vivo = QtWidgets.QCheckBox("Seguir en vivo")
        self.chk_vivo.setChecked(True)
        self.chk_vivo.setToolTip("Muestra cada medición al terminar el paso (y la captura en curso si está activo\n"
                                 "«Dibujar durante el paso», en ⚙); desmarcado, la medición seleccionada en la tabla")
        self.chk_vivo.toggled.connect(lambda on: on or self._seleccion())
        fb.addWidget(self.chk_vivo)
        fb.addStretch(1)
        b_exp = QtWidgets.QPushButton("Exportar PLY + JSON")
        b_exp.setToolTip(f"Guarda la nube (PLY) y la medición (JSON) seleccionadas en {self.carpeta}\n"
                         "(mismo formato que vol_export_dir del volume_node)")
        b_exp.clicked.connect(self.exportar)
        b_vaciar = QtWidgets.QPushButton("Vaciar")
        b_vaciar.setToolTip("Vacía la tabla (el registro CSV de la sesión se conserva)")
        b_vaciar.clicked.connect(self.vaciar)
        fb.addWidget(b_exp)
        fb.addWidget(b_vaciar)
        lc.addLayout(fb)
        der.addWidget(caja)
        der.setSizes([260, 240])
        div.addWidget(der)
        div.setStretchFactor(0, 3)
        div.setStretchFactor(1, 2)
        div.setSizes([640, 400])
        raiz.addWidget(div, 1)

        self._fuente_cambiada(0)
        self._escenario_cambiado(self.c_escenario.currentText())
        self._pintar_estados(PassState.ARRANQUE, VolState.ARRANQUE, False, math.nan, 0)

    def _construir_3d(self) -> QtWidgets.QWidget:
        caja = QtWidgets.QWidget()
        l = QtWidgets.QVBoxLayout(caja)
        l.setContentsMargins(0, 0, 0, 0)
        l.setSpacing(2)
        fila = QtWidgets.QHBoxLayout()
        self.chk_nube = QtWidgets.QCheckBox("Nube")
        self.chk_sup = QtWidgets.QCheckBox("Superficie")
        self.chk_planos = QtWidgets.QCheckBox("Planos de los LiDAR")
        for c in (self.chk_nube, self.chk_sup, self.chk_planos):
            c.setChecked(True)
            fila.addWidget(c)
        self.chk_sup.setToolTip("Malla h(x, y) de la medición (solo en la final, como en RViz)")
        self.chk_planos.setToolTip("Plano de corte del LiDAR de volumen en la captura en curso\n"
                                   "(solo con «Dibujar durante el paso», en ⚙)")
        fila.addStretch(1)
        b_vista = QtWidgets.QPushButton("Vista inicial")
        b_vista.clicked.connect(self._vista_inicial)
        fila.addWidget(b_vista)
        l.addLayout(fila)
        if gl is None:
            aviso = QtWidgets.QLabel(
                "Vista 3D no disponible: falta PyOpenGL.<br><br>Instálalo con<br>"
                "<code>sudo apt install python3-opengl</code><br>y vuelve a abrir el visor.<br><br>"
                f"<span style='color:{TEXTO_SUAVE}'>Mientras tanto usa «Superficie (arriba)» y «Áreas A(x)».<br>"
                f"({ERROR_GL})</span>")
            aviso.setAlignment(QtCore.Qt.AlignmentFlag.AlignCenter)
            aviso.setWordWrap(True)
            l.addWidget(aviso, 1)
            for c in (self.chk_nube, self.chk_sup, self.chk_planos, b_vista):
                c.setEnabled(False)
            self.vista3d = None
            return caja
        self.vista3d = gl.GLViewWidget()
        self.vista3d.setBackgroundColor("#0f1318")
        piso = gl.GLGridItem()
        piso.setSize(24, 8)
        piso.setSpacing(1, 1)
        piso.translate(-6, 0, 0)
        piso.setColor((90, 100, 115, 120))
        self.vista3d.addItem(piso)
        ejes = gl.GLAxisItem()
        ejes.setSize(1, 1, 1)
        self.vista3d.addItem(ejes)
        self.gl_nube = gl.GLScatterPlotItem(pos=np.zeros((1, 3)), size=2.5, pxMode=True)
        self.gl_nube.setGLOptions("opaque")
        self.vista3d.addItem(self.gl_nube)
        self.gl_sup = gl.GLMeshItem(vertexes=np.zeros((1, 3, 3)), smooth=False, drawEdges=False, glOptions="opaque")
        self.vista3d.addItem(self.gl_sup)
        self.gl_planos = gl.GLLinePlotItem(pos=np.zeros((2, 3)), color=(0.35, 0.65, 1.0, 0.8), width=1.5, mode="lines")
        self.vista3d.addItem(self.gl_planos)
        for c in (self.chk_nube, self.chk_sup, self.chk_planos):
            c.toggled.connect(self._visibilidad_3d)
        self._visibilidad_3d()
        self._vista_inicial()
        l.addWidget(self.vista3d, 1)
        return caja

    def _construir_superior(self) -> QtWidgets.QWidget:
        self.g_sup = pg.PlotWidget()
        self.g_sup.setAspectLocked(True)
        self.g_sup.setLabel("bottom", "x [m] (frente = 0, cola hacia −x)")
        self.g_sup.setLabel("left", "y [m] (izquierda del camión)")
        self.g_sup.showGrid(x=True, y=True, alpha=0.15)
        self.img_sup = pg.ImageItem()
        self.g_sup.addItem(self.img_sup)
        self.barra_sup = barra = pg.ColorBarItem(values=(0, H_COLOR_MAX), colorMap=CMAP, label="altura [m]",
                                                 interactive=False)
        barra.setImageItem(pg.ImageItem(), insert_in=self.g_sup.getPlotItem())   # solo la escala
        self.txt_sup = pg.TextItem("", color="#e6edf3", anchor=(0, 1))
        self.g_sup.addItem(self.txt_sup)
        # Altura bajo el cursor: celda MEDIDA (perfil más cercano, celda lateral), no la imagen remuestreada
        self._malla_sup = None
        self.celda_sup = QtWidgets.QGraphicsRectItem()
        self.celda_sup.setPen(pg.mkPen("#ffffff", width=1))
        self.celda_sup.setZValue(10)
        self.celda_sup.hide()
        self.g_sup.addItem(self.celda_sup)
        self.txt_cursor = pg.TextItem("", color="#e6edf3", anchor=(0, 1), fill=pg.mkBrush(15, 19, 24, 220),
                                      border=pg.mkPen(BORDE))
        self.txt_cursor.setZValue(11)
        self.txt_cursor.hide()
        self.g_sup.addItem(self.txt_cursor)
        self._proxy_cursor = pg.SignalProxy(self.g_sup.scene().sigMouseMoved, rateLimit=30, slot=self._cursor_superior)
        return self.g_sup

    def _limpiar_cursor_superior(self, malla: bool = True):
        if malla:
            self._malla_sup = None
        self.celda_sup.hide()
        self.txt_cursor.hide()

    def _cursor_superior(self, evento):
        pos = evento[0]
        malla = self._malla_sup
        if malla is None or not self.g_sup.sceneBoundingRect().contains(pos):
            self.celda_sup.hide()
            self.txt_cursor.hide()
            return
        xu, xs, hs, y, dx, dy, idx = malla
        p = self.g_sup.getPlotItem().vb.mapSceneToView(pos)
        i = int(round((p.x() - xu[0]) / dx))
        j = int(round((p.y() - y[0]) / dy))
        if not (0 <= i < len(xu) and 0 <= j < len(y)):
            self.celda_sup.hide()
            self.txt_cursor.hide()
            return
        k = int(idx[i])
        h = float(hs[k, j])
        # celda medida: de punto medio a punto medio entre perfiles vecinos
        x0 = 0.5 * (xs[k - 1] + xs[k]) if k > 0 else xs[k] - dx / 2
        x1 = 0.5 * (xs[k] + xs[k + 1]) if k + 1 < len(xs) else xs[k] + dx / 2
        self.celda_sup.setRect(QtCore.QRectF(x0, y[j] - dy / 2, x1 - x0, dy))
        self.celda_sup.show()
        alto = f"h = {h:.3f} m" if math.isfinite(h) else "sin dato"
        self.txt_cursor.setText(f"{alto}\nx = {xs[k]:+.2f} m  y = {y[j]:+.2f} m\nperfil {k + 1}/{len(xs)}")
        self.txt_cursor.setPos(p.x(), p.y())
        self.txt_cursor.show()

    def _construir_areas(self) -> QtWidgets.QWidget:
        self.g_area = pg.PlotWidget()
        self.g_area.showGrid(x=True, y=True, alpha=0.2)
        self.g_area.setLabel("bottom", "x [m] (frente = 0)")
        self.g_area.setLabel("left", "área de la sección A(x) [m²]")
        self.c_area = pg.PlotDataItem(pen=pg.mkPen("#2fb8c4", width=2), symbol="o", symbolSize=4,
                                      symbolBrush="#2fb8c4", symbolPen=None, fillLevel=0,
                                      brush=pg.mkBrush(47, 184, 196, 50))
        self.g_area.addItem(self.c_area)
        return self.g_area

    # ------------------------------------------------------------------ fuente y control
    def _fuente_cambiada(self, i: int):
        self.c_escenario.setVisible(i == 0)
        self.w_topicos.setVisible(i == 1)
        self.c_vel.setVisible(i != 1)
        self.b_pausa.setVisible(i != 1)

    def _escenario_cambiado(self, nombre: str):
        personalizado = nombre.startswith("Personalizado")
        if personalizado and not self.b_params.isChecked():
            self.b_params.setChecked(True)

    def _vel_reproduccion(self, i: int):
        self.reproductor.velocidad = (1.0, 2.0, 5.0, math.inf)[i]

    def _params_fuente(self, sim: bool) -> tuple[VelocityParams, VolumeParams]:
        from dataclasses import replace
        vp, wp = cargar_params(self.yaml_ruta)
        if sim:   # yaml de los sensores físicos; el simulador es el modelo (igual que volume_sim.launch.py)
            vp, wp = replace(vp, **SIM_VELOCITY_OVERRIDES), replace(wp, **SIM_VOLUME_OVERRIDES)
        else:     # datos reales: la calibración aplicada (offset de rango, rayo abajo, sentido, relación)
            vp = replace(vp, **self.calibracion.get("vel", {}))
            wp = replace(wp, **self.calibracion.get("vol", {}))
        return vp, replace(wp, vol_edge_from_velocity=self.chk_bordes.isChecked(),
                           vol_cloud_live=self.chk_durante.isChecked())

    def fijar_calibracion(self, clave: str, efectivos: dict | None):
        """Valores efectivos de un LiDAR ("vel" o "vol") que deja la pestaña «Calibración». Se usan con
        las fuentes reales (ROS, rosbag) desde la próxima medición; el yaml no cambia."""
        if efectivos:
            self.calibracion[clave] = dict(efectivos)
        else:
            self.calibracion.pop(clave, None)

    def iniciar(self):
        if self.leyendo or self.generador.state() != QtCore.QProcess.ProcessState.NotRunning:
            return
        i = self.c_fuente.currentIndex()
        if i == 0:
            self._iniciar_sim()
        elif i == 1:
            self._iniciar_ros()
        else:
            self._iniciar_bag()

    def _bloquear(self, on: bool):
        """Mientras se genera o se lee no se cambia la fuente ni el escenario."""
        self.leyendo = on
        for w in (self.c_fuente, self.c_escenario, self.w_topicos, self.b_iniciar, self.chk_bordes, self.chk_durante,
                  self.s_kmh, self.c_sentido, self.s_lat, self.s_yaw):
            w.setEnabled(not on)

    def _spec(self) -> dict:
        return dict(nombre=self.c_escenario.currentText(), kmh=self.s_kmh.value(),
                    sentido=1 if self.c_sentido.currentIndex() == 0 else -1,
                    lateral=self.s_lat.value(), guinada=self.s_yaw.value())

    def _ruta_cache(self, spec: dict) -> Path:
        from sim_volumen import PERSONALIZADO, VERSION
        clave = spec["nombre"] if spec["nombre"] != PERSONALIZADO else repr(sorted(spec.items()))
        h = hashlib.sha1(f"{VERSION}|{clave}".encode()).hexdigest()[:12]
        return self.cache / f"paso_{h}.npz"

    def _iniciar_sim(self):
        spec = self._spec()
        ruta = self._ruta_cache(spec)
        if ruta.exists():
            self._reproducir_npz(ruta)
            return
        self.cache.mkdir(parents=True, exist_ok=True)
        args = [str(SIM_VOLUMEN), "--salida", str(ruta)]
        if spec["nombre"].startswith("Personalizado"):
            args += ["--kmh", str(spec["kmh"]), "--sentido", str(spec["sentido"]),
                     "--lateral", str(spec["lateral"]), "--guinada", str(spec["guinada"])]
        else:
            args += ["--caso", spec["nombre"]]
        self._ruta_generando = ruta
        self.barra.setValue(0)
        self.l_info.setText(f"Generando «{spec['nombre']}» en el simulador 3D (una sola vez; queda en caché)…")
        self.v._log(f"Volumen: generando el escenario «{spec['nombre']}» (~1 s por escaneo en la RPi, en paralelo).", "info")
        self._bloquear(True)
        self.generador.start(sys.executable, args)

    def _salida_generador(self):
        for linea in bytes(self.generador.readAllStandardOutput()).decode(errors="replace").splitlines():
            if linea.startswith("progreso "):
                a, b = linea.split()[1].split("/")
                self.barra.setValue(int(1000 * int(a) / max(1, int(b))))
                self.l_info.setText(f"Generando escenario: {a}/{b} escaneos de cada LiDAR…")
            elif linea.strip() and not linea.startswith("listo"):
                self.v._log(f"sim_volumen: {linea}", "aviso")

    def _fin_generador(self, codigo, _estado):
        self._bloquear(False)
        ruta = getattr(self, "_ruta_generando", None)
        if getattr(self, "_generacion_cancelada", False):
            self._generacion_cancelada = False
            self.l_info.setText("Generación cancelada.")
            if ruta is not None and ruta.exists():
                ruta.unlink()
            return
        if codigo != 0 or ruta is None or not ruta.exists():
            self.l_info.setText("No se pudo generar el escenario (ver registro).")
            self.v._log(f"Volumen: el generador terminó con código {codigo}.", "error")
            return
        self._reproducir_npz(ruta)

    def _reproducir_npz(self, ruta: Path):
        d = np.load(ruta, allow_pickle=False)
        self.verdad_m3, self.esperado = float(d["verdad_m3"]), str(d["esperado"])
        vp, wp = self._params_fuente(sim=True)
        extra = json.loads(str(d["vol_params"])) if "vol_params" in d else {}   # parámetros propios del caso
        if extra:
            from dataclasses import replace
            wp = replace(wp, **extra)
        self.proc.configurar(vp, wp)
        self._actualizar_perfil_estatico(wp)
        fuente, total = escaneos_npz(d)
        esperado = {"V": "válida", "D": "descartada"}.get(self.esperado, "inválida por " + self.esperado[2:])
        self.l_info.setText(f"Simulador: «{d['nombre']}» · verdad {self.verdad_m3:.2f} m³ · esperado: {esperado}")
        self.v._log(f"Volumen: reproduciendo «{d['nombre']}» (simulación; no es evidencia de campo).", "info")
        self._bloquear(True)
        self.reproductor.iniciar(fuente, total)

    def _iniciar_ros(self):
        from rclpy.qos import qos_profile_sensor_data
        from sensor_msgs.msg import LaserScan
        tv0, tw0 = self.e_vel.text().strip(), self.e_vol.text().strip()
        portico = getattr(self.v, "portico", None)
        nodo0 = self.v.ros.node
        if portico is not None and not portico.encendido and \
                (nodo0.count_publishers(tv0) == 0 or nodo0.count_publishers(tw0) == 0):
            r = QtWidgets.QMessageBox.question(
                self, "LiDAR del pórtico", f"Nadie publica {tv0} y {tw0}.\n\n¿Encender los dos LiDAR del pórtico "
                "(velocidad 192.168.8.8, volumen 192.168.11.2)?")
            if r == QtWidgets.QMessageBox.StandardButton.Yes:
                portico.encender()
        vp, wp = self._params_fuente(sim=False)
        self.proc.configurar(vp, wp)
        self._actualizar_perfil_estatico(wp)
        self.verdad_m3, self.esperado = math.nan, ""
        nodo = self.v.ros.node
        tv, tw = self.e_vel.text().strip(), self.e_vol.text().strip()
        self._subs = [
            nodo.create_subscription(LaserScan, tv, lambda m: self.proc.velocidad(scan_de_msg(m), time.monotonic()),
                                     qos_profile_sensor_data),
            nodo.create_subscription(LaserScan, tw, lambda m: self.proc.volumen(scan_de_msg(m), time.monotonic()),
                                     qos_profile_sensor_data)]
        cal = " + calibración aplicada" if self.calibracion else ""
        self.l_info.setText(f"ROS en vivo: {tv} + {tw} · parámetros de config/falconscan.yaml{cal}")
        self.v._log(f"Volumen: suscrito a {tv} y {tw}.", "info")
        self._bloquear(True)
        self.t_vivo.start()

    def _tick_vivo(self):
        self.proc.tick(time.monotonic())

    def _iniciar_bag(self):
        ruta = QtWidgets.QFileDialog.getExistingDirectory(self, "Rosbag con los dos LiDAR", str(self.carpeta_bags))
        if not ruta:
            return
        try:
            temas = topicos_bag(ruta)
        except Exception as e:   # noqa: BLE001
            QtWidgets.QMessageBox.warning(self, "Rosbag", f"No se pudo abrir el rosbag:\n{e}")
            return
        tv = next((t for t in temas if "vel" in t), self.e_vel.text())
        tw = next((t for t in temas if "vol" in t), self.e_vol.text())
        if tv not in temas or tw not in temas or tv == tw:
            QtWidgets.QMessageBox.warning(self, "Rosbag", "El rosbag debe tener un LaserScan de cada LiDAR "
                                          f"(con «vel» y «vol» en el nombre).\nEncontrados: {', '.join(temas) or 'ninguno'}")
            return
        vp, wp = self._params_fuente(sim=False)
        self.proc.configurar(vp, wp)
        self._actualizar_perfil_estatico(wp)
        self.verdad_m3, self.esperado = math.nan, ""
        self.l_info.setText(f"Rosbag: {Path(ruta).name} · {tv} + {tw}")
        self._bloquear(True)
        self.reproductor.iniciar(escaneos_bag(ruta, tv, tw), contar_bag(ruta, [tv, tw]))

    def detener(self):
        if self.generador.state() != QtCore.QProcess.ProcessState.NotRunning:
            self._generacion_cancelada = True
            self.generador.kill()
        self.reproductor.detener()
        for s in self._subs:
            self.v.ros.node.destroy_subscription(s)
        self._subs = []
        self.t_vivo.stop()
        self._bloquear(False)

    def _fin_reproduccion(self, msg: str):
        self._bloquear(False)
        if msg and msg != "detenido":
            self.v._log(msg, "error")
        else:
            self.v._log("Volumen: reproducción terminada.", "info")

    def cerrar(self):
        self.detener()

    def _vaciar_cache(self):
        n = 0
        for f in self.cache.glob("paso_*.npz"):
            f.unlink()
            n += 1
        self.v._log(f"Volumen: {n} pasos simulados borrados de la caché.", "info")

    # ------------------------------------------------------------------ refresco
    def refrescar(self, dibujar: bool):
        """Mediciones y estado siempre; los gráficos solo con la pestaña visible y sin objeto bajo el
        pórtico (salvo «Dibujar durante el paso»): la reconstrucción se muestra al terminar."""
        d = self.proc.tomar()
        for m in d["mediciones"]:
            self._medicion(m)
        for c in d["capturas"]:
            self.v._log(f"Volumen: medición {c.measurement_id}: inicio de captura (paso {c.pass_id}).", "info")
        self._pintar_estados(d["estado_vel"], d["estado_vol"], d["listo"], d["parcial"], d["perfiles"])
        tp = f" · {d['t_proc'] * 1e3:.0f} ms/escaneo" if math.isfinite(d["t_proc"]) else ""
        v = f" · v {d['v'] * KMH:+.1f} km/h" if math.isfinite(d["v"]) else ""
        self.l_info.setToolTip(f"escaneos: {d['n_vel']} velocidad, {d['n_vol']} volumen{tp}{v}")
        en_paso = VolState(d["estado_vol"]) in EN_PASO_VOL or PassState(d["estado_vel"]) in EN_PASO_VEL
        durante = self.proc.wp.vol_cloud_live
        if not dibujar or (en_paso and not durante):
            return
        ahora = time.monotonic()
        if d["ultimo_vol"] is not None and (en_paso or ahora - self._perfil_dibujado > PERFIL_REPOSO_S):
            self._perfil_dibujado = ahora
            self._dibujar_perfil(*d["ultimo_vol"])
        nube = d["nube"]
        if nube is not None and nube.provisional and self.chk_vivo.isChecked():
            if ahora - self._en_vivo_dibujado > 0.25:     # la nube crece en cada perfil: ~4 Hz alcanza
                self._en_vivo_dibujado = ahora
                self._mostrar(nube.reconstruction, final=False)

    def _pintar_estados(self, ev, ew, listo, parcial, perfiles):
        clave = (ev, ew, listo, round(parcial, 2) if math.isfinite(parcial) else None, perfiles)
        if clave == self._estados_pintados:
            return
        self._estados_pintados = clave
        tv, cv = ESTADOS_VEL[PassState(ev)]
        tw, cw = ESTADOS_VOL[VolState(ew)]
        self.l_vel.setText(f"Velocidad: <b style='color:{cv}'>{tv}</b>")
        self.l_vol.setText(f"Volumen: <b style='color:{cw}'>{tw}</b>")
        self.l_listo.setText(f"<b style='color:{VERDE if listo else ROJO}'>● {'listo' if listo else 'no listo'}</b>")
        self.l_parcial.setText(f"V parcial <b>{parcial:.2f} m³</b> · {perfiles} perfiles" if math.isfinite(parcial)
                               else "")

    def _actualizar_perfil_estatico(self, p: VolumeParams):
        self.roi_perfil.setRect(p.vol_roi_y_min_m, p.floor_z_m, p.vol_roi_y_max_m - p.vol_roi_y_min_m, p.vol_roi_z_max_m)
        self.umbral_perfil.setPos(p.floor_z_m + p.vol_h_threshold_m)
        self.g_perfil.setRange(xRange=(p.vol_roi_y_min_m - 1.0, p.vol_roi_y_max_m + 1.0),
                               yRange=(p.floor_z_m - 0.3, p.mount_height_m + 0.3), padding=0)

    def _dibujar_perfil(self, scan: Scan, sec):
        p = self.proc.wp
        r = validate_ranges(scan.ranges, max(p.range_min_m, scan.range_min), min(p.range_max_m, scan.range_max))
        y, z = polar_to_xz(r - p.range_bias_m, scan.angles, p.geometry)
        ok = np.isfinite(y) & (y > p.vol_roi_y_min_m - 1.0) & (y < p.vol_roi_y_max_m + 1.0)
        self.pts_perfil.setData(y[ok], z[ok])
        if sec is None or sec.h is None:
            self.h_perfil.setData([], [])
            self.ext_perfil.hide()
            self.txt_perfil.setText("sin datos" if sec is not None and sec.no_data else "")
            return
        g = p.grid
        bordes = g.x_min + np.arange(sec.h.size + 1) * g.dx
        self.h_perfil.setData(bordes, sec.h + p.floor_z_m)
        if sec.occupied:
            self.ext_perfil.setRegion((sec.y0, sec.y1))
            self.ext_perfil.show()
            avisos = [n for n, on in (("truncado", sec.truncated), ("sombra", sec.shadow), ("alto", sec.high),
                                      ("piso", sec.floor_drift)) if on]
            self.txt_perfil.setText(f"A = {sec.area_m2:.2f} m²  ancho {sec.width:.2f} m  alto {sec.max_h:.2f} m"
                                    + (f"\n⚠ {', '.join(avisos)}" if avisos else ""))
        else:
            self.ext_perfil.hide()
            self.txt_perfil.setText("vacío")
        self.txt_perfil.setPos(p.vol_roi_y_min_m - 0.9, p.mount_height_m + 0.25)

    # ------------------------------------------------------------------ reconstrucción
    def _mostrar(self, rec: Reconstruction, final: bool, m: Measurement | None = None):
        self._rec_vista = rec
        self._h_escala = escala_altura(rec)        # la misma escala en la 3D y en la vista superior
        self.barra_sup.setLevels((0.0, self._h_escala))
        self._mostrar_3d(rec, final)
        self._mostrar_superior(rec, m)
        if m is not None:
            self._mostrar_areas(m)

    def _visibilidad_3d(self):
        if self.vista3d is None:
            return
        self.gl_nube.setVisible(self.chk_nube.isChecked())
        self.gl_sup.setVisible(self.chk_sup.isChecked())
        self.gl_planos.setVisible(self.chk_planos.isChecked())

    def _vista_inicial(self):
        if self.vista3d is None:
            return
        self.vista3d.setCameraPosition(pos=QtGui.QVector3D(-4.0, 0.0, 1.2), distance=17, elevation=28, azimuth=-125)

    def _mostrar_3d(self, rec: Reconstruction, final: bool):
        if self.vista3d is None:
            return
        pts = rec.points
        if len(pts) > 150_000:      # la RPi dibuja bien hasta ~150 mil puntos
            pts = pts[:: math.ceil(len(pts) / 150_000)]
        if len(pts):
            self.gl_nube.setData(pos=pts[:, :3], color=colores_altura(pts[:, 3], h_max=self._h_escala), size=2.5)
        else:
            self.gl_nube.setData(pos=np.zeros((1, 3)), color=(0, 0, 0, 0))
        tri = surface_triangles(rec, step=2) if final else np.empty((0, 3, 3))
        if len(tri):
            col = colores_altura(tri[:, :, 2].reshape(-1), alfa=0.85, h_max=self._h_escala).reshape(-1, 3, 4)
            self.gl_sup.setMeshData(vertexes=tri, vertexColors=col)
        else:
            self.gl_sup.setMeshData(vertexes=np.zeros((1, 3, 3)))
        # Planos de corte en la captura en vivo: el del LiDAR de volumen va en el último perfil
        if not final and len(rec.surface_x):
            p = self.proc.wp
            xs = float(rec.surface_x[-1])
            y0, y1, zt = p.vol_roi_y_min_m, p.vol_roi_y_max_m, p.mount_height_m
            seg = [(xs, y0, 0), (xs, y1, 0), (xs, y1, 0), (xs, y1, zt), (xs, y1, zt), (xs, y0, zt), (xs, y0, zt), (xs, y0, 0)]
            self.gl_planos.setData(pos=np.array(seg, dtype=float), mode="lines")
        else:
            self.gl_planos.setData(pos=np.zeros((2, 3)), mode="lines")

    def _mostrar_superior(self, rec: Reconstruction, m: Measurement | None):
        x, y, h = rec.surface_x, rec.surface_y, rec.surface_h
        if len(x) < 2 or h.size == 0:
            self.img_sup.clear()
            self.txt_sup.setText("")
            self._limpiar_cursor_superior()
            return
        # Perfiles a x irregulares: se remuestrea a una malla uniforme (vecino más cercano) solo para
        # mostrar. NaN = sin dato -> transparente, nunca 0.
        orden = np.argsort(x)
        xs, hs = x[orden], h[orden]
        dx = 0.05
        xu = np.arange(xs[0], xs[-1] + dx / 2, dx)
        idx = np.clip(np.searchsorted(xs, xu), 1, len(xs) - 1)
        idx -= (xu - xs[idx - 1]) < (xs[idx] - xu)
        img = hs[idx]
        rgba = CMAP.map(np.clip(np.nan_to_num(img, nan=0.0) / self._h_escala, 0, 1).reshape(-1), mode="byte")
        rgba = rgba.reshape(img.shape[0], img.shape[1], 4)
        rgba[..., 3] = np.where(np.isfinite(img), 255, 0)
        self.img_sup.setImage(rgba, autoLevels=False)
        dy = float(np.median(np.diff(y))) if len(y) > 1 else 0.02
        self.img_sup.setRect(QtCore.QRectF(xu[0] - dx / 2, y[0] - dy / 2, len(xu) * dx, len(y) * dy))
        self._malla_sup = (xu, xs, hs, y, dx, dy, idx)
        self._limpiar_cursor_superior(malla=False)
        if m is not None:
            self.txt_sup.setText(f"#{m.measurement_id}  V = {m.volume_m3:.2f} m³  L = {m.length_m:.2f} m")
            self.txt_sup.setPos(xu[0], y[0] - 0.2)
        else:
            self.txt_sup.setText("captura en curso")
            self.txt_sup.setPos(xu[0], y[0] - 0.2)
        self.g_sup.autoRange()

    def _mostrar_areas(self, m: Measurement):
        a = np.array(m.area_profile, dtype=float)
        if a.size == 0:
            self.c_area.setData([], [])
            return
        self.c_area.setData(a[:, 0], a[:, 1])
        self.g_area.setTitle(titulo_grafico(
            f"#{m.measurement_id}: V = ∫A dx = {m.volume_m3:.2f} m³ ± {m.end_uncertainty_m3:.2f} (frente y cola, 1σ) · "
            f"{m.n_profiles} perfiles · Δx máx {m.max_dx_m:.2f} m · bordes: {m.edge_method}"))
        self.g_area.autoRange()

    # ------------------------------------------------------------------ mediciones
    def _medicion(self, m: Measurement):
        self.mediciones.append(m)
        self._registrar(m)
        res, color = RESULTADO.get(m.result, (m.result_text, TEXTO_SUAVE))
        err = (m.volume_m3 - self.verdad_m3) / self.verdad_m3 if math.isfinite(self.verdad_m3) else math.nan
        txt_err = f" · error {err:+.1%} (simulación)" if math.isfinite(err) else ""
        self.v._log(f"Volumen: medición {m.measurement_id} (paso {m.pass_id}): V = {m.volume_m3:.2f} m³ ± "
                    f"{m.end_uncertainty_m3:.2f}, L = {m.length_m:.2f} m · {res}{txt_err}"
                    + (f" · {', '.join(m.flags)}" if m.flags else ""), "ok" if m.result <= 1 else "aviso")
        f = self.tabla.rowCount()
        self.tabla.insertRow(f)
        num = lambda v, fmt: "—" if not math.isfinite(v) else format(v, fmt)   # noqa: E731
        celdas = {
            C_ID: str(m.measurement_id), C_PASO: str(m.pass_id or "—"),
            C_SENTIDO: {1: "+x", -1: "−x"}.get(m.direction, "?"), C_RES: res,
            C_V: num(m.volume_m3, ".2f"), C_INC: num(m.end_uncertainty_m3, ".2f"),
            C_VERDAD: num(self.verdad_m3, ".2f"), C_ERR: "—" if not math.isfinite(err) else f"{err:+.1%}",
            C_LARGO: num(m.length_m, ".2f"), C_ANCHO: num(m.max_width_m, ".2f"), C_ALTO: num(m.max_height_m, ".2f"),
            C_VAV: num(m.mean_advance_mps * KMH, ".1f"), C_PERF: str(m.n_profiles), C_BANDERAS: ", ".join(m.flags) or "—",
        }
        for c, t in celdas.items():
            it = QtWidgets.QTableWidgetItem(t)
            it.setData(QtCore.Qt.ItemDataRole.UserRole, len(self.mediciones) - 1)
            if c == C_RES:
                it.setForeground(QtGui.QColor(color))
            if c == C_ERR and math.isfinite(err):
                it.setForeground(QtGui.QColor(VERDE if abs(err) < 0.03 else (AMBAR if abs(err) < 0.08 else ROJO)))
            self.tabla.setItem(f, c, it)
        self.tabla.resizeColumnsToContents()
        self.tabla.scrollToBottom()
        if self.chk_vivo.isChecked() and m.reconstruction is not None:
            self._mostrar(m.reconstruction, final=True, m=m)
        else:
            self._mostrar_areas(m)

    def _medicion_seleccionada(self) -> Measurement | None:
        filas = self.tabla.selectionModel().selectedRows()
        if not filas:
            return None
        k = self.tabla.item(filas[0].row(), 0).data(QtCore.Qt.ItemDataRole.UserRole)
        return self.mediciones[k]

    def _seleccion(self):
        m = self._medicion_seleccionada()
        if m is None or m.reconstruction is None:
            return
        if self.leyendo and self.chk_vivo.isChecked():
            self.chk_vivo.setChecked(False)      # ver una medición vieja deja de seguir la captura
        self._mostrar(m.reconstruction, final=True, m=m)

    def _registrar(self, m: Measurement):
        """Registro automático de la sesión (no se pierde aunque se cierre el visor)."""
        if self.registro is None:
            self.carpeta.mkdir(parents=True, exist_ok=True)
            self.registro = self.carpeta / f"mediciones_{dt.datetime.now():%Y%m%d_%H%M%S}.csv"
            with open(self.registro, "w", newline="") as fh:
                csv.writer(fh).writerow(["hora_registro", "fuente", "measurement_id", "pass_id", "sentido", "resultado",
                                         "volumen_m3", "incertidumbre_m3", "verdad_m3", "error_pct", "largo_m",
                                         "ancho_m", "alto_m", "v_avance_kmh", "perfiles", "dx_max_m",
                                         "sync_offset_s", "banderas"])
            self.v._log(f"Registro de mediciones de volumen: {self.registro}", "info")
        err = 100 * (m.volume_m3 - self.verdad_m3) / self.verdad_m3 if math.isfinite(self.verdad_m3) else math.nan
        f = lambda v: "" if not math.isfinite(v) else f"{v:.4f}"   # noqa: E731
        fuente = self.c_fuente.currentText() + (f": {self.c_escenario.currentText()}" if self.c_fuente.currentIndex() == 0 else "")
        with open(self.registro, "a", newline="") as fh:
            csv.writer(fh).writerow([dt.datetime.now().isoformat(timespec="seconds"), fuente, m.measurement_id,
                                     m.pass_id, m.direction, m.result_text, f(m.volume_m3), f(m.end_uncertainty_m3),
                                     f(self.verdad_m3), f(err), f(m.length_m), f(m.max_width_m), f(m.max_height_m),
                                     f(m.mean_advance_mps * KMH), m.n_profiles, f(m.max_dx_m), f(m.sync_offset_s),
                                     "|".join(m.flags)])

    def exportar(self):
        m = self._medicion_seleccionada() or (self.mediciones[-1] if self.mediciones else None)
        if m is None or m.reconstruction is None:
            QtWidgets.QMessageBox.information(self, "Exportar", "Todavía no hay mediciones.")
            return
        base = self.carpeta / f"medicion_{m.measurement_id:04d}_{dt.datetime.now():%Y%m%d_%H%M%S}"
        write_ply(base.with_suffix(".ply"), m.reconstruction.points)
        datos = {k: v for k, v in m.__dict__.items() if k != "reconstruction"}
        datos["fuente"] = self.c_fuente.currentText()
        if math.isfinite(self.verdad_m3):
            datos["verdad_m3_simulacion"] = self.verdad_m3
        write_json(base.with_suffix(".json"), datos)
        self.v._log(f"Volumen: medición {m.measurement_id} exportada en {base}.ply/.json", "ok")

    def vaciar(self):
        self.tabla.setRowCount(0)
        self.mediciones.clear()
        self.c_area.setData([], [])
        self.g_area.setTitle("")
        self.img_sup.clear()
        self._limpiar_cursor_superior()
        if self.vista3d is not None:
            self.gl_nube.setData(pos=np.zeros((1, 3)), color=(0, 0, 0, 0))
            self.gl_sup.setMeshData(vertexes=np.zeros((1, 3, 3)))
