"""Pestaña «Prueba de paso» del visor: VelocityEstimator + PassTracker en vivo sobre /scan.

Corre el mismo núcleo que velocity_node (core/estimator.py y core/pass_tracker.py) dentro del
visor, para ver en cada escaneo el plano del pórtico (x-z), el perfil h(x), la velocidad y el
estado del paso, y registrar cada paso con su velocidad de referencia.
"""
from __future__ import annotations

import csv
import datetime as dt
import math
import re
import sys
import threading
import time
import warnings
from collections import deque
from dataclasses import fields, replace
from pathlib import Path

import numpy as np
import pyqtgraph as pg
import yaml
from pyqtgraph.Qt import QtCore, QtGui, QtWidgets

from comun import ajuste_pared, ros_a_sensor, tarjeta

try:
    from falconscan_velocity.core import (PassEvent, PassFlag, PassState, PassTracker, Scan, Status,
                                         VelocityEstimator, VelocityParams, polar_to_xz, validate_ranges)
except ImportError:   # sin `source install/setup.bash`: usar el código fuente del repo
    sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "src" / "falconscan_velocity"))
    from falconscan_velocity.core import (PassEvent, PassFlag, PassState, PassTracker, Scan, Status,
                                         VelocityEstimator, VelocityParams, polar_to_xz, validate_ranges)

# La mediana del núcleo avisa en cada ventana de rayos sin retorno (normal en el S2E)
warnings.filterwarnings("ignore", "All-NaN slice", RuntimeWarning)

KMH = 3.6
VENTANA_V_S = 60.0   # segundos visibles en el gráfico de velocidad

ESTADOS = {
    PassState.ARRANQUE: ("ARRANQUE", "#d29922", "Esperando ROI vacía. Un camión a medio pasar no se mide."),
    PassState.VACIO: ("VACÍO", "#3fb950", "Listo: ROI libre, esperando vehículo."),
    PassState.ENTRANDO: ("ENTRANDO", "#2fb8c4", "Ocupación detectada; confirmando que es un vehículo."),
    PassState.EN_PASO: ("EN PASO", "#58a6ff", "Paso confirmado: midiendo."),
    PassState.SALIENDO: ("SALIENDO", "#a371f7", "ROI aparentemente libre; confirmando salida."),
    PassState.OBSTRUIDO: ("OBSTRUIDO", "#f85149", "Ocupación demasiado larga: objeto fijo, vehículo detenido o mala calibración."),
    PassState.SIN_DATOS: ("SIN DATOS", "#f85149", "No llegan escaneos del LiDAR."),
}
BANDERAS_TXT = {
    PassFlag.ENTRADA_FUERA_DE_BORDE: "entrada fuera de borde",
    PassFlag.SALIDA_FUERA_DE_BORDE: "salida fuera de borde",
    PassFlag.VELOCIDAD_FUERA_DE_RANGO: "velocidad fuera de rango",
    PassFlag.REVERSA: "reversa",
    PassFlag.POCOS_VALIDOS: "pocos válidos",
    PassFlag.TIMEOUT: "timeout",
    PassFlag.SENSOR_PERDIDO: "sensor perdido",
}
STATUS_TXT = {   # por qué no hay velocidad en este escaneo (core/estimator.py, Status)
    Status.NO_OBJECT: "sin objeto", Status.WARMING_UP: "iniciando", Status.BAD_DT: "Δt inválido",
    Status.LOW_OVERLAP: "poco solape", Status.LOW_PEAK: "correlación baja", Status.AMBIGUOUS: "ambiguo",
    Status.AT_SEARCH_LIMIT: "en el límite", Status.IMPLAUSIBLE: "salto implausible",
}


# ----------------------------------------------------------------------------- parámetros

def cargar_params(ruta: Path) -> VelocityParams:
    datos = yaml.safe_load(ruta.read_text())["velocity_node"]["ros__parameters"]
    nombres = set(VelocityParams.field_names())
    return VelocityParams(**{k: v for k, v in datos.items() if k in nombres})


def guardar_params(ruta: Path, p: VelocityParams, nombres: list[str]) -> list[str]:
    """Reescribe solo los valores de `nombres` en el yaml, conservando comentarios y orden."""
    txt = ruta.read_text()
    cambiados = []
    for n in nombres:
        v = getattr(p, n)
        if isinstance(v, float):
            val = f"{v:.7g}"
            val = val if any(c in val for c in ".e") else val + ".0"   # ROS 2 exige punto en los float
        else:
            val = str(int(v))
        nuevo, k = re.subn(rf"^(\s*{n}:\s*)([^\s#]+)", lambda m: m.group(1) + val, txt, count=1, flags=re.M)
        if k and nuevo != txt:
            cambiados.append(n)
        txt = nuevo
    ruta.write_text(txt)
    return cambiados


def nadir_ros_deg(p: VelocityParams) -> float:
    """Ángulo ROS que apunta al piso: phi = dir·theta + offset = −90°."""
    a = round((-90.0 - math.degrees(p.angle_offset_rad)) / p.angle_direction, 4) % 360
    return 0.0 if a >= 359.995 else a   # el yaml guarda −π/2 con 7 decimales


def offset_desde_nadir(nadir_deg: float, direccion: int) -> float:
    a = (-90.0 - direccion * nadir_deg + 180.0) % 360 - 180.0     # en [−180°, 180°)
    return math.radians(a)


# ----------------------------------------------------------------------------- procesamiento

class Procesamiento:
    """Estimador + tracker, alimentado desde el hilo de ROS o del reproductor de rosbag."""

    def __init__(self, params: VelocityParams):
        self._lock = threading.Lock()
        self.activo = False
        self.configurar(params)

    def configurar(self, params: VelocityParams):
        with self._lock:
            self.p = params
            self.est = VelocityEstimator(params)
            self.tr = PassTracker(params)
            self.hist: deque = deque(maxlen=int(VENTANA_V_S * 12))
            self.eventos: list[PassEvent] = []
            self.ultimo = None
            self.t0 = None
            self.t_proc: deque = deque(maxlen=50)

    def procesar(self, scan: Scan, ahora: float | None):
        if not self.activo:
            return
        t1 = time.perf_counter()
        with self._lock:
            e = self.est.process(scan)
            ev = self.tr.update_from_estimate(e, ahora)
            if self.t0 is None:
                self.t0 = scan.stamp
            h = None if self.est.last_profile is None else self.est.last_profile.copy()
            self.hist.append((scan.stamp, int(e.status), e.velocity_mps, e.velocity_filtered_mps, int(self.tr.state)))
            self.eventos += ev
            self.ultimo = (scan, e, h)
            self.t_proc.append(time.perf_counter() - t1)

    def tick(self, ahora: float):
        if not self.activo:
            return
        with self._lock:
            self.eventos += self.tr.tick(ahora)

    def cortar(self):
        """La fuente se detuvo: un paso abierto se cierra como SENSOR_PERDIDO (T11) y queda SIN_DATOS."""
        with self._lock:
            self.eventos += self.tr.tick(math.inf)
        self.activo = False

    def tomar(self):
        with self._lock:
            ev, self.eventos = self.eventos, []
            return dict(ultimo=self.ultimo, hist=list(self.hist), eventos=ev, estado=self.tr.state,
                        pass_id=self.tr.pass_id, abierto=self.tr.open_pass, falsos=self.tr.n_false_entries,
                        obstrucciones=self.tr.n_obstructions, t0=self.t0,
                        t_proc=float(np.mean(self.t_proc)) if self.t_proc else math.nan)


# ----------------------------------------------------------------------------- reproductor de rosbag

def _abrir_bag(ruta: str):
    from rosbag2_py import ConverterOptions, SequentialReader, StorageOptions
    rd = SequentialReader()
    rd.open(StorageOptions(uri=ruta, storage_id=""), ConverterOptions("", ""))
    temas = {t.name: t.type for t in rd.get_all_topics_and_types()}
    tema = "/scan" if temas.get("/scan", "").endswith("LaserScan") else \
        next((n for n, t in temas.items() if t.endswith("LaserScan")), None)
    return rd, tema


def inspeccionar_bag(ruta: str) -> tuple[str | None, bool, int]:
    """(tópico LaserScan, ¿es del simulador?, nº de escaneos). El simulador no publica intensidades."""
    from rclpy.serialization import deserialize_message
    from rosbag2_py import StorageFilter
    from sensor_msgs.msg import LaserScan
    rd, tema = _abrir_bag(ruta)
    if tema is None:
        return None, False, 0
    n = next((t.message_count for t in rd.get_metadata().topics_with_message_count
              if t.topic_metadata.name == tema), 0)
    rd.set_filter(StorageFilter(topics=[tema]))
    msg = deserialize_message(rd.read_next()[1], LaserScan) if rd.has_next() else None
    return tema, msg is not None and len(msg.intensities) == 0, n


class ReproductorBag(QtCore.QObject):
    """Lee /scan de un rosbag y lo entrega al visor con la temporización original (× velocidad)."""
    termino = QtCore.Signal(str)
    progreso = QtCore.Signal(float)

    def __init__(self, entregar, parent=None):
        super().__init__(parent)
        self.entregar = entregar
        self.velocidad = 1.0
        self.pausado = False
        self._parar = threading.Event()
        self._hilo = None

    def iniciar(self, ruta: str):
        self._parar.clear()
        self._hilo = threading.Thread(target=self._correr, args=(ruta,), daemon=True)
        self._hilo.start()

    def detener(self):
        self._parar.set()
        if self._hilo:
            self._hilo.join(2.0)

    def activo(self) -> bool:
        return self._hilo is not None and self._hilo.is_alive()

    def _correr(self, ruta):
        from rclpy.serialization import deserialize_message
        from rosbag2_py import StorageFilter
        from sensor_msgs.msg import LaserScan
        try:
            rd, tema = _abrir_bag(ruta)
            if tema is None:
                self.termino.emit("El rosbag no tiene ningún tópico LaserScan.")
                return
            meta = rd.get_metadata()
            n_total = max(1, next((t.message_count for t in meta.topics_with_message_count
                                   if t.topic_metadata.name == tema), 1))
            rd.set_filter(StorageFilter(topics=[tema]))
            n_leidos = 0
            t_bag0 = t_real0 = None
            while rd.has_next() and not self._parar.is_set():
                _, data, t_ns = rd.read_next()
                tb = t_ns * 1e-9
                while self.pausado and not self._parar.is_set():
                    time.sleep(0.05)
                    t_bag0 = None
                if t_bag0 is None:
                    t_bag0, t_real0 = tb, time.monotonic()
                if math.isfinite(self.velocidad):
                    espera = (tb - t_bag0) / self.velocidad - (time.monotonic() - t_real0)
                    if espera > 0:
                        self._parar.wait(espera)
                else:
                    time.sleep(0.002)   # «máx.»: deja respirar al GUI
                self.entregar(deserialize_message(data, LaserScan))
                n_leidos += 1
                self.progreso.emit(n_leidos / n_total)
            self.termino.emit("" if not self._parar.is_set() else "detenido")
        except Exception as e:   # noqa: BLE001  (se informa en el GUI)
            self.termino.emit(f"Error leyendo el rosbag: {e}")


# ----------------------------------------------------------------------------- widgets

def _spin(minimo, maximo, decimales, paso, sufijo=""):
    if decimales == 0:
        w = QtWidgets.QSpinBox(minimum=int(minimo), maximum=int(maximo), singleStep=int(paso))
    else:
        w = QtWidgets.QDoubleSpinBox(minimum=minimo, maximum=maximo, decimals=decimales, singleStep=paso)
    if sufijo:
        w.setSuffix(" " + sufijo)
    return w


# (parámetro, etiqueta, mín, máx, decimales, paso, unidad, factor de visualización, ayuda)
CAMPOS = {
    "Montaje": [
        ("mount_height_m", "Altura del sensor H", 0.1, 30, 3, 0.01, "m", 1, "Altura del centro óptico sobre el piso"),
        ("x_offset_m", "Desplazamiento x", -10, 10, 3, 0.01, "m", 1, "Posición x del sensor en el marco mundo"),
        ("floor_z_m", "Altura del piso", -2, 2, 3, 0.01, "m", 1, "z del piso (0 si H se mide desde el piso)"),
    ],
    "ROI y detección": [
        ("roi_x_min_m", "ROI x mín (entrada)", -15, 0, 2, 0.1, "m", 1, "Borde de entrada: el vehículo avanza hacia +x"),
        ("roi_x_max_m", "ROI x máx (salida)", 0, 15, 2, 0.1, "m", 1, "Borde de salida"),
        ("roi_z_max_m", "Altura máx. considerada", 0.5, 30, 2, 0.1, "m", 1, "Por encima se descarta (estructura)"),
        ("object_h_threshold_m", "Umbral de altura objeto", 0.05, 5, 2, 0.05, "m", 1, "Altura sobre el piso a partir de la cual una celda es objeto"),
        ("object_min_bins", "Celdas mínimas objeto", 1, 500, 0, 1, "celdas", 1, "Con dx = 2 cm, 25 celdas = 0.5 m"),
    ],
    "Máquina de estados": [
        ("pass_enter_scans", "Confirmar entrada", 1, 50, 0, 1, "escaneos", 1, "Escaneos ocupados seguidos para INICIO"),
        ("pass_exit_scans", "Confirmar salida", 1, 100, 0, 1, "escaneos", 1, "Escaneos vacíos seguidos para FIN (tolera huecos más cortos)"),
        ("pass_empty_scans", "Vacíos para quedar listo", 1, 100, 0, 1, "escaneos", 1, "Para salir de ARRANQUE y OBSTRUIDO"),
        ("pass_max_s", "Ocupación máxima", 1, 600, 1, 1, "s", 1, "Más tiempo ocupado -> TIMEOUT / OBSTRUIDO"),
        ("scan_timeout_s", "Timeout sin datos", 0.1, 10, 2, 0.1, "s", 1, "Sin escaneos -> SIN_DATOS"),
        ("pass_edge_margin_m", "Margen de borde", 0, 5, 2, 0.1, "m", 1, "Entrada/salida deben verse dentro de esta franja del borde"),
        ("pass_v_min_mps", "Velocidad mínima", 0, 100, 1, 0.5, "km/h", KMH, ""),
        ("pass_v_max_mps", "Velocidad máxima", 0, 100, 1, 0.5, "km/h", KMH, ""),
        ("pass_v_tolerance_mps", "Tolerancia de velocidad", 0, 20, 1, 0.1, "km/h", KMH, ""),
        ("pass_reverse_v_mps", "Umbral de reversa", -20, 0, 1, 0.1, "km/h", KMH, "v OK por debajo de esto cuenta como reversa"),
        ("pass_reverse_scans", "Escaneos de reversa", 1, 100, 0, 1, "escaneos", 1, ""),
        ("pass_min_valid_fraction", "Fracción mínima válida", 0, 100, 0, 5, "%", 100, "Escaneos con velocidad OK dentro del paso"),
    ],
}
NOMBRES_EDITABLES = [c[0] for g in CAMPOS.values() for c in g] + ["angle_offset_rad", "angle_direction"]


class PanelPaso(QtWidgets.QWidget):
    """Contenido de la pestaña «Prueba de paso»."""

    def __init__(self, ventana, yaml_ruta: Path, carpeta_datos: Path):
        super().__init__()
        self.v = ventana
        self.yaml_ruta = yaml_ruta
        self.carpeta = carpeta_datos
        self.params_yaml = cargar_params(yaml_ruta)
        self.fuente_sim = False
        self.proc = Procesamiento(self._params_efectivos(self.params_yaml))
        self.filas: dict[int, int] = {}            # pass_id -> fila de la tabla
        self.regiones: dict[int, tuple] = {}       # pass_id -> (región, texto)
        self.archivo_sesion: Path | None = None
        self.eventos_sesion: list[PassEvent] = []
        self._construir()
        self._cargar_en_formulario(self.params_yaml)
        self._dibujar_estatico()

    # ------------------------------------------------------------------ parámetros
    def _params_efectivos(self, base: VelocityParams) -> VelocityParams:
        """El simulador publica en orden de medición y sin error de rango (igual que el launch)."""
        if self.fuente_sim:
            return replace(base, scan_time_reversed=False, range_scale=1.0, range_bias_m=0.0)
        return base

    def fijar_fuente(self, sim: bool):
        self.fuente_sim = sim
        self.aplicar(silencioso=True)

    def _cargar_en_formulario(self, p: VelocityParams):
        for n, (w, factor) in self.campos.items():
            w.blockSignals(True)
            v = getattr(p, n) * factor
            w.setValue(int(round(v)) if isinstance(w, QtWidgets.QSpinBox) else v)
            w.blockSignals(False)
        self.c_dir.setCurrentIndex(0 if p.angle_direction > 0 else 1)
        self.c_nadir.setValue(nadir_ros_deg(p))
        self._actualizar_nadir_txt()
        self.lbl_cambios.setText("")

    def _desde_formulario(self) -> VelocityParams:
        """Valores del formulario. Un campo que muestra el valor del yaml (redondeado) conserva el
        valor exacto del yaml: si no, 1.39 m/s -> 5.0 km/h -> 1.3889 parecería un cambio."""
        base = self.params_yaml
        tipos = {f.name: f.type for f in fields(VelocityParams)}
        valores = {}
        for n, (w, factor) in self.campos.items():
            dec = w.decimals() if isinstance(w, QtWidgets.QDoubleSpinBox) else 0
            if math.isclose(w.value(), round(getattr(base, n) * factor, dec), abs_tol=1e-9):
                valores[n] = getattr(base, n)
            else:
                valores[n] = w.value() / factor
                if tipos[n] in (int, "int"):
                    valores[n] = int(round(valores[n]))
        d = -1 if self.c_dir.currentIndex() == 1 else 1
        valores["angle_direction"] = d
        if d == base.angle_direction and math.isclose(self.c_nadir.value(), round(nadir_ros_deg(base), 2), abs_tol=1e-9):
            valores["angle_offset_rad"] = base.angle_offset_rad
        else:
            valores["angle_offset_rad"] = offset_desde_nadir(self.c_nadir.value(), d)
        return replace(base, **valores)

    def aplicar(self, silencioso=False):
        p = self._desde_formulario()
        if p.roi_x_max_m - p.roi_x_min_m < 0.5:
            QtWidgets.QMessageBox.warning(self, "ROI", "La ROI debe medir al menos 0.5 m.")
            return
        activo = self.proc.activo
        self.proc.configurar(self._params_efectivos(p))
        self.proc.activo = activo
        self.params_form = p
        self._dibujar_estatico()
        self._limpiar_grafico_v()
        self.lbl_cambios.setText("" if p == self.params_yaml else "● Aplicado, sin guardar en velocity.yaml")
        if not silencioso:
            self.v._log("Parámetros de paso aplicados; máquina de estados reiniciada (ARRANQUE).", "aviso")

    def guardar(self):
        p = self._desde_formulario()
        cambios = [n for n in NOMBRES_EDITABLES if getattr(p, n) != getattr(self.params_yaml, n)]
        if not cambios:
            QtWidgets.QMessageBox.information(self, "Guardar", "No hay cambios respecto a velocity.yaml.")
            return
        lista = "\n".join(f"  {n}: {getattr(self.params_yaml, n)} → {getattr(p, n):.6g}" for n in cambios)
        r = QtWidgets.QMessageBox.question(
            self, "Guardar en velocity.yaml",
            f"Se escribirán estos valores en\n{self.yaml_ruta}\n\n{lista}\n\n"
            "velocity_node los usará la próxima vez que se lance.")
        if r != QtWidgets.QMessageBox.StandardButton.Yes:
            return
        escritos = guardar_params(self.yaml_ruta, p, cambios)
        self.params_yaml = cargar_params(self.yaml_ruta)
        self.aplicar(silencioso=True)
        self.v._log(f"velocity.yaml actualizado: {', '.join(escritos)}", "ok")

    def recargar(self):
        self.params_yaml = cargar_params(self.yaml_ruta)
        self._cargar_en_formulario(self.params_yaml)
        self.aplicar()

    def _actualizar_nadir_txt(self):
        self.lbl_nadir.setText(f"= {ros_a_sensor(self.c_nadir.value()):+.1f}° en la marca del sensor (S2E)")

    # ------------------------------------------------------------------ calibración con el piso
    def calibrar_piso(self, buffer, corregir):
        if len(buffer) < 5:
            QtWidgets.QMessageBox.information(self, "Calibrar con el piso",
                                              "Inicia la lectura y espera unos segundos con la ROI vacía.")
            return
        n = buffer[-1].ranges.size
        with warnings.catch_warnings():
            warnings.simplefilter("ignore", RuntimeWarning)
            r = corregir(np.nanmedian(np.stack([e.ranges for e in buffer[-10:] if e.ranges.size == n]), axis=0))
        ang = np.degrees(buffer[-1].ang) % 360
        actual = self.c_nadir.value()
        candidatos = []
        for c in range(0, 360, 5):
            sel = np.abs((ang - c + 180) % 360 - 180) < 25
            f = ajuste_pared(r[sel] * np.cos(np.radians(ang[sel])), r[sel] * np.sin(np.radians(ang[sel])))
            if not f:
                continue
            d, sigma, normal, npts = f
            # El pie de la perpendicular debe verse dentro de la ventana y casi todos los puntos en la recta
            if abs((normal - c + 180) % 360 - 180) < 10 and npts >= 0.7 * np.isfinite(r[sel]).sum():
                candidatos.append((abs((normal - actual + 180) % 360 - 180), d, sigma, normal, npts))
        if not candidatos:
            QtWidgets.QMessageBox.warning(self, "Calibrar con el piso",
                                          "No encontré ninguna superficie plana clara. Revisa que el piso se vea "
                                          "y que no haya objetos en el plano de escaneo.")
            return
        _, d, sigma, normal, npts = min(candidatos)
        H = d + self.campos["floor_z_m"][0].value()
        r = QtWidgets.QMessageBox.question(
            self, "Calibrar con el piso",
            f"Superficie plana más cercana a la dirección «abajo» actual ({actual:.1f}° ROS):\n\n"
            f"  dirección perpendicular: {normal:.2f}° ROS ({ros_a_sensor(normal):+.2f}° en la marca del sensor)\n"
            f"  distancia: {d:.3f} m   (σ {sigma * 1e3:.1f} mm, {npts} puntos, mediana de 10 escaneos)\n\n"
            f"¿Usarla como piso? Quedaría H = {H:.3f} m y «abajo» = {normal:.2f}° ROS.\n"
            "Supone piso horizontal y plano de escaneo vertical. Si no es el piso, cancela y\n"
            "ajusta el ángulo a mano.")
        if r != QtWidgets.QMessageBox.StandardButton.Yes:
            return
        self.campos["mount_height_m"][0].setValue(H)
        self.c_nadir.setValue(normal)
        self.aplicar()
        self.v._log(f"Calibración con el piso: H = {H:.3f} m, abajo = {normal:.2f}° ROS (aplicado, sin guardar).", "ok")

    # ------------------------------------------------------------------ construcción
    def _construir(self):
        raiz = QtWidgets.QVBoxLayout(self)
        raiz.setContentsMargins(0, 6, 0, 0)

        # --- cabecera: estado + contadores
        cab = QtWidgets.QHBoxLayout()
        self.estado = QtWidgets.QLabel()
        self.estado.setMinimumWidth(230)
        self.estado.setAlignment(QtCore.Qt.AlignmentFlag.AlignCenter)
        cab.addWidget(self.estado)
        cont = QtWidgets.QVBoxLayout()
        self.estado_desc = QtWidgets.QLabel()
        self.estado_desc.setWordWrap(True)
        self.estado_desc.setStyleSheet("font-size: 13px;")
        self.listo = QtWidgets.QLabel()
        cont.addWidget(self.estado_desc)
        cont.addWidget(self.listo)
        cab.addLayout(cont, 1)
        w, self.k_v = tarjeta("Velocidad", tip="Velocidad cruda del último escaneo válido (no la EMA)")
        cab.addWidget(w)
        w, self.k_id = tarjeta("Paso actual")
        cab.addWidget(w)
        w, self.k_pasos = tarjeta("Pasos (válidos)", tip="Pasos cerrados en esta sesión; válidos = sin banderas")
        cab.addWidget(w)
        w, self.k_falsos = tarjeta("Falsas entradas", tip="Ocupaciones que no llegaron a confirmarse (T4)")
        cab.addWidget(w)
        w, self.k_obs = tarjeta("Obstrucciones")
        cab.addWidget(w)
        w, self.k_cpu = tarjeta("Cómputo", tip="Tiempo medio de estimador + máquina de estados por escaneo")
        cab.addWidget(w)
        raiz.addLayout(cab)

        div = QtWidgets.QSplitter(QtCore.Qt.Orientation.Horizontal)
        izq = QtWidgets.QSplitter(QtCore.Qt.Orientation.Vertical)

        # --- plano del pórtico
        self.g_xz = pg.PlotWidget()
        self.g_xz.setTitle("<span style='color:#e6edf3;font-size:12pt'>Plano de escaneo (marco mundo)</span>")
        self.g_xz.setAspectLocked(True)
        self.g_xz.showGrid(x=True, y=True, alpha=0.12)
        self.g_xz.setLabel("bottom", "x: avance →  [m]")
        self.g_xz.setLabel("left", "z: altura [m]")
        self.roi_rect = QtWidgets.QGraphicsRectItem()
        self.roi_rect.setPen(pg.mkPen("#2fb8c4", width=1, style=QtCore.Qt.PenStyle.DashLine))
        self.roi_rect.setBrush(pg.mkBrush(47, 184, 196, 18))
        self.g_xz.addItem(self.roi_rect)
        self.bordes = [QtWidgets.QGraphicsRectItem() for _ in range(2)]
        for b in self.bordes:
            b.setPen(pg.mkPen(None))
            b.setBrush(pg.mkBrush(210, 153, 34, 30))
            self.g_xz.addItem(b)
        self.piso = pg.InfiniteLine(angle=0, pen=pg.mkPen("#8b98a5", width=2))
        self.umbral = pg.InfiniteLine(angle=0, pen=pg.mkPen("#d29922", width=1, style=QtCore.Qt.PenStyle.DotLine))
        self.g_xz.addItem(self.piso)
        self.g_xz.addItem(self.umbral)
        self.ext_rect = QtWidgets.QGraphicsRectItem()
        self.ext_rect.setPen(pg.mkPen("#58a6ff", width=2))
        self.ext_rect.setBrush(pg.mkBrush(88, 166, 255, 40))
        self.g_xz.addItem(self.ext_rect)
        self.puntos_xz = pg.ScatterPlotItem(size=3, pen=None, brush=pg.mkBrush("#c9d1d9"))
        self.g_xz.addItem(self.puntos_xz)
        self.perfil_h = pg.PlotDataItem(pen=pg.mkPen("#3fb950", width=2), stepMode="center", connect="finite")
        self.g_xz.addItem(self.perfil_h)
        self.sensor = pg.ScatterPlotItem(size=14, brush=pg.mkBrush("#2fb8c4"), pen=pg.mkPen("w"))
        self.g_xz.addItem(self.sensor)
        self.txt_roi = pg.TextItem("", color="#2fb8c4", anchor=(0, 1))
        self.g_xz.addItem(self.txt_roi)
        izq.addWidget(self.g_xz)

        # --- velocidad vs tiempo
        self.g_v = pg.PlotWidget()
        self.g_v.setTitle("<span style='color:#e6edf3;font-size:12pt'>Velocidad estimada</span>")
        self.g_v.showGrid(x=True, y=True, alpha=0.15)
        self.g_v.setLabel("left", "km/h")
        self.g_v.setLabel("bottom", "tiempo [s]")
        self.g_v.addLegend(offset=(-10, 5), labelTextColor="#8b98a5")
        self.banda = pg.LinearRegionItem(orientation="horizontal", movable=False,
                                         brush=pg.mkBrush(63, 185, 80, 22), pen=pg.mkPen(None))
        self.g_v.addItem(self.banda)
        self.v_ema = pg.PlotDataItem(pen=pg.mkPen("#2fb8c4", width=2), name="EMA (solo visual)")
        self.v_ok = pg.ScatterPlotItem(size=6, brush=pg.mkBrush("#3fb950"), pen=None, name="v cruda (OK)")
        self.g_v.addItem(self.v_ema)
        self.g_v.addItem(self.v_ok)
        izq.addWidget(self.g_v)

        # --- tabla de pasos
        caja = QtWidgets.QWidget()
        lt = QtWidgets.QVBoxLayout(caja)
        lt.setContentsMargins(0, 4, 0, 0)
        fila = QtWidgets.QHBoxLayout()
        t = QtWidgets.QLabel("Pasos de la sesión")
        t.setStyleSheet("font-weight: 700; color: #2fb8c4;")
        fila.addWidget(t)
        ayuda = QtWidgets.QLabel("Escribe la velocidad de referencia (radar, GPS o cronómetro) para calcular el error.")
        ayuda.setObjectName("suave")
        fila.addWidget(ayuda, 1)
        b = QtWidgets.QPushButton("Exportar CSV")
        b.clicked.connect(self.exportar)
        fila.addWidget(b)
        b = QtWidgets.QPushButton("Nueva sesión")
        b.setToolTip("Vacía la tabla y reinicia la máquina de estados (los ids vuelven a 1)")
        b.clicked.connect(self.nueva_sesion)
        fila.addWidget(b)
        lt.addLayout(fila)
        self.tabla = QtWidgets.QTableWidget(0, 11)
        self.tabla.setHorizontalHeaderLabels(["#", "Inicio", "Duración", "v mediana", "v mín–máx", "Válidos",
                                              "Huecos", "Banderas", "v ref (km/h)", "Error", "Notas"])
        self.tabla.verticalHeader().setVisible(False)
        self.tabla.horizontalHeader().setStretchLastSection(True)
        self.tabla.setStyleSheet("QTableWidget { background: #0f1318; gridline-color: #2a323d; border: 1px solid #2a323d; }"
                                 "QHeaderView::section { background: #1b2129; color: #8b98a5; border: none; padding: 4px; }")
        self.tabla.itemChanged.connect(self._celda_editada)
        lt.addWidget(self.tabla)
        izq.addWidget(caja)
        izq.setSizes([380, 230, 220])
        div.addWidget(izq)

        # --- panel de configuración
        panel = QtWidgets.QWidget()
        lp = QtWidgets.QVBoxLayout(panel)
        lp.setContentsMargins(4, 0, 0, 0)
        self.campos: dict[str, tuple[QtWidgets.QAbstractSpinBox, float]] = {}
        for grupo, campos in CAMPOS.items():
            g = QtWidgets.QGroupBox(grupo)
            f = QtWidgets.QFormLayout(g)
            if grupo == "Montaje":
                self.c_nadir = _spin(0, 359.99, 2, 0.5, "°")
                self.c_nadir.setWrapping(True)
                self.c_nadir.setToolTip("Ángulo ROS del rayo que apunta verticalmente hacia el piso.\n"
                                        "Define angle_offset_rad = −90° − sentido·ángulo.")
                self.c_nadir.valueChanged.connect(self._actualizar_nadir_txt)
                f.addRow("Rayo hacia abajo (ROS)", self.c_nadir)
                self.lbl_nadir = QtWidgets.QLabel()
                self.lbl_nadir.setObjectName("suave")
                f.addRow("", self.lbl_nadir)
                self.c_dir = QtWidgets.QComboBox()
                self.c_dir.addItems(["+1 (normal)", "−1 (espejado)"])
                self.c_dir.setToolTip("angle_direction. Cambia el signo de x: úsalo si el vehículo\n"
                                      "avanza hacia −x en la vista (velocidades negativas).")
                f.addRow("Sentido del ángulo", self.c_dir)
            for n, etiqueta, mn, mx, dec, paso, unidad, factor, ayuda in campos:
                w = _spin(mn, mx, dec, paso, unidad)
                if ayuda:
                    w.setToolTip(f"{ayuda}\n[{n}]")
                else:
                    w.setToolTip(n)
                self.campos[n] = (w, factor)
                f.addRow(etiqueta, w)
            if grupo == "Montaje":
                b = QtWidgets.QPushButton("Calibrar con el piso vacío")
                b.setToolTip("Busca la superficie plana del piso cerca de la dirección «abajo» actual\n"
                             "y propone H y el ángulo. Con la ROI vacía.")
                b.clicked.connect(lambda: self.calibrar_piso(
                    self.v.ros.instantanea()[1],
                    lambda r: (r - self.proc.p.range_bias_m) * self.proc.p.range_scale))
                f.addRow(b)
            lp.addWidget(g)
        fila = QtWidgets.QHBoxLayout()
        for texto, fn, tip in (("Aplicar", self.aplicar, "Usa estos valores ahora (reinicia la máquina de estados)"),
                               ("Guardar en yaml", self.guardar, "Escribe los valores en config/velocity.yaml"),
                               ("Recargar", self.recargar, "Vuelve a los valores de velocity.yaml")):
            b = QtWidgets.QPushButton(texto)
            b.setToolTip(tip)
            b.clicked.connect(fn)
            fila.addWidget(b)
        lp.addLayout(fila)
        self.lbl_cambios = QtWidgets.QLabel("")
        self.lbl_cambios.setStyleSheet("color: #d29922;")
        lp.addWidget(self.lbl_cambios)
        lp.addStretch(1)
        area = QtWidgets.QScrollArea()
        area.setWidgetResizable(True)
        area.setWidget(panel)
        area.setMinimumWidth(360)
        area.setHorizontalScrollBarPolicy(QtCore.Qt.ScrollBarPolicy.ScrollBarAlwaysOff)
        div.addWidget(area)
        div.setStretchFactor(0, 1)
        div.setSizes([1100, 380])
        raiz.addWidget(div, 1)
        self.params_form = self.params_yaml
        self._pintar_estado(PassState.ARRANQUE, False)

    # ------------------------------------------------------------------ dibujo
    def _dibujar_estatico(self):
        p = self.proc.p
        f = p.floor_z_m
        self.roi_rect.setRect(p.roi_x_min_m, f, p.roi_x_max_m - p.roi_x_min_m, p.roi_z_max_m)
        m = p.pass_edge_margin_m
        self.bordes[0].setRect(p.roi_x_min_m, f, m, p.roi_z_max_m)
        self.bordes[1].setRect(p.roi_x_max_m - m, f, m, p.roi_z_max_m)
        self.piso.setPos(f)
        self.umbral.setPos(f + p.object_h_threshold_m)
        self.sensor.setData([p.x_offset_m], [p.mount_height_m])
        self.txt_roi.setText("ROI · franjas ámbar = bordes de entrada y salida")
        self.txt_roi.setAnchor((0, 0))
        self.txt_roi.setPos(p.roi_x_min_m, f - 0.05)
        ancho = max(p.roi_x_max_m - p.roi_x_min_m, p.mount_height_m) * 0.75
        cx = 0.5 * (p.roi_x_min_m + p.roi_x_max_m)
        self.g_xz.setRange(xRange=(cx - ancho - 1, cx + ancho + 1), yRange=(f - 0.5, max(p.mount_height_m, p.roi_z_max_m) + 0.8))
        self.banda.setRegion(((p.pass_v_min_mps - p.pass_v_tolerance_mps) * KMH,
                              (p.pass_v_max_mps + p.pass_v_tolerance_mps) * KMH))
        self.g_v.setYRange(-5, (p.pass_v_max_mps + 1.5) * KMH)

    def _limpiar_grafico_v(self):
        for reg, txt in self.regiones.values():
            self.g_v.removeItem(reg)
            self.g_v.removeItem(txt)
        self.regiones.clear()
        self.v_ok.setData([], [])
        self.v_ema.setData([], [])

    def _pintar_estado(self, estado, abierto):
        nombre, color, desc = ESTADOS[estado]
        self.estado.setText(nombre)
        self.estado.setStyleSheet(f"font-size: 26px; font-weight: 800; color: {color}; border: 2px solid {color};"
                                  f"border-radius: 10px; padding: 8px 18px; background: transparent;")
        self.estado_desc.setText(desc)
        listo = estado in (PassState.VACIO, PassState.ENTRANDO, PassState.EN_PASO, PassState.SALIENDO)
        self.listo.setText(f"<b style='color:{'#3fb950' if listo else '#f85149'}'>"
                           f"{'● Sistema listo' if listo else '● Sistema NO listo'}</b>"
                           + ("  ·  <span style='color:#58a6ff'>paso abierto</span>" if abierto else ""))

    def refrescar(self, congelado: bool, dibujar: bool = True):
        """Procesa eventos y contadores siempre; los gráficos solo si la pestaña está visible."""
        d = self.proc.tomar()
        for ev in d["eventos"]:
            self._evento(ev, d["t0"])
        self._pintar_estado(d["estado"], d["abierto"])
        self.k_id.setText(f"#{d['pass_id']}" if d["abierto"] else "—")
        n_fin = len([e for e in self.eventos_sesion if e.event == "FIN"])
        n_ok = len([e for e in self.eventos_sesion if e.valid])
        self.k_pasos.setText(f"{n_fin} ({n_ok})")
        self.k_falsos.setText(str(d["falsos"]))
        self.k_obs.setText(str(d["obstrucciones"]))
        if math.isfinite(d["t_proc"]):
            self.k_cpu.setText(f"{d['t_proc'] * 1e3:.0f} ms")
        if d["ultimo"] is None or congelado or not dibujar:
            return
        scan, est, h = d["ultimo"]
        p = self.proc.p
        self.k_v.setText(f"{est.velocity_mps * KMH:.1f} km/h" if est.valid else STATUS_TXT[Status(est.status)])

        # Puntos en el plano del pórtico (misma validación y corrección que el estimador)
        r = (validate_ranges(scan.ranges, max(p.range_min_m, scan.range_min), min(p.range_max_m, scan.range_max))
             - p.range_bias_m) * p.range_scale
        x, z = polar_to_xz(r, scan.angles, p.geometry)
        ok = np.isfinite(x) & (z > p.floor_z_m - 1.0) & (z < p.mount_height_m + 1.0)
        self.puntos_xz.setData(x[ok], z[ok])
        if h is not None:
            xc = p.grid.x_min + np.arange(h.size + 1) * p.grid.dx
            self.perfil_h.setData(xc, h + p.floor_z_m)
        if est.object_extent:
            x0, x1 = est.object_extent
            hmax = np.nanmax(h) if h is not None and np.isfinite(h).any() else p.object_h_threshold_m
            self.ext_rect.setRect(x0, p.floor_z_m, x1 - x0, hmax)
            self.ext_rect.show()
        else:
            self.ext_rect.hide()

        # Velocidad
        hist = d["hist"]
        if hist and d["t0"] is not None:
            a = np.array([(s - d["t0"], st, v, vf) for s, st, v, vf, _ in hist], dtype=float)
            ok = a[:, 1] == Status.OK
            self.v_ok.setData(a[ok, 0], a[ok, 2] * KMH)
            self.v_ema.setData(a[:, 0], np.where(a[:, 1] == Status.NO_OBJECT, np.nan, a[:, 3] * KMH), connect="finite")
            t_fin = a[-1, 0]
            self.g_v.setXRange(max(0, t_fin - VENTANA_V_S), max(VENTANA_V_S, t_fin) + 1, padding=0)
            for pid, (reg, txt) in self.regiones.items():
                if getattr(reg, "abierta", False):
                    reg.setRegion((reg.getRegion()[0], t_fin))

    # ------------------------------------------------------------------ eventos y tabla
    def _evento(self, ev: PassEvent, t0):
        self.eventos_sesion.append(ev)
        self._registrar(ev)
        t_ini = ev.t_start - (t0 or ev.t_start)
        if ev.event == "INICIO":
            reg = pg.LinearRegionItem((t_ini, t_ini), movable=False, brush=pg.mkBrush(88, 166, 255, 40),
                                      pen=pg.mkPen(None))
            reg.abierta = True
            txt = pg.TextItem(f"#{ev.pass_id}", color="#58a6ff", anchor=(0, 0))
            txt.setPos(t_ini, (self.proc.p.pass_v_max_mps + 1.2) * KMH)
            self.g_v.addItem(reg)
            self.g_v.addItem(txt)
            self.regiones[ev.pass_id] = (reg, txt)
            self._fila_inicio(ev)
            self.v._log(f"Paso #{ev.pass_id}: INICIO", "info")
            return
        color = (63, 185, 80, 45) if ev.valid else (210, 153, 34, 45)
        if ev.pass_id in self.regiones:
            reg, txt = self.regiones[ev.pass_id]
            reg.abierta = False
            reg.setRegion((t_ini, ev.t_end - (t0 or ev.t_start)))
            reg.setBrush(pg.mkBrush(*color))
            txt.setColor("#3fb950" if ev.valid else "#d29922")
        self._fila_fin(ev)
        banderas = ", ".join(BANDERAS_TXT[f] for f in PassFlag if f and f in ev.flags) or "sin banderas"
        self.v._log(f"Paso #{ev.pass_id}: FIN · {ev.duration_s:.1f} s · v mediana "
                    f"{ev.v_median_mps * KMH:.1f} km/h · {banderas}", "ok" if ev.valid else "aviso")

    def _item(self, texto, editable=False, color=None):
        it = QtWidgets.QTableWidgetItem(texto)
        if not editable:
            it.setFlags(it.flags() & ~QtCore.Qt.ItemFlag.ItemIsEditable)
        if color:
            it.setForeground(QtGui.QColor(color))
        return it

    def _fila_inicio(self, ev):
        self.tabla.blockSignals(True)
        f = self.tabla.rowCount()
        self.tabla.insertRow(f)
        self.filas[ev.pass_id] = f
        self.tabla.setItem(f, 0, self._item(str(ev.pass_id)))
        self.tabla.setItem(f, 1, self._item(dt.datetime.fromtimestamp(ev.t_start).strftime("%H:%M:%S")))
        self.tabla.setItem(f, 2, self._item("en curso…", color="#58a6ff"))
        for c in range(3, 8):
            self.tabla.setItem(f, c, self._item(""))
        self.tabla.setItem(f, 8, self._item("", editable=True))
        self.tabla.setItem(f, 9, self._item(""))
        self.tabla.setItem(f, 10, self._item("", editable=True))
        self.tabla.scrollToBottom()
        self.tabla.blockSignals(False)

    def _fila_fin(self, ev):
        if ev.pass_id not in self.filas:
            self._fila_inicio(ev)
        f = self.filas[ev.pass_id]
        self.tabla.blockSignals(True)
        km = lambda v: f"{v * KMH:.1f}" if math.isfinite(v) else "—"   # noqa: E731
        frac = ev.n_valid / ev.n_scans if ev.n_scans else 0
        self.tabla.setItem(f, 2, self._item(f"{ev.duration_s:.2f} s"))
        self.tabla.setItem(f, 3, self._item(f"{km(ev.v_median_mps)} km/h"))
        self.tabla.setItem(f, 4, self._item(f"{km(ev.v_min_mps)} – {km(ev.v_max_mps)}"))
        self.tabla.setItem(f, 5, self._item(f"{ev.n_valid}/{ev.n_scans} ({frac:.0%})"))
        self.tabla.setItem(f, 6, self._item(str(ev.n_gaps)))
        txt = ", ".join(BANDERAS_TXT[x] for x in PassFlag if x and x in ev.flags) or "✓ válido"
        self.tabla.setItem(f, 7, self._item(txt, color="#3fb950" if ev.valid else "#d29922"))
        self.tabla.blockSignals(False)
        self.tabla.resizeColumnsToContents()
        self._error_ref(f)

    def _celda_editada(self, it):
        if it.column() == 8:
            self._error_ref(it.row())

    def _evento_de_fila(self, fila):
        pid = int(self.tabla.item(fila, 0).text())
        return next((e for e in reversed(self.eventos_sesion) if e.pass_id == pid and e.event == "FIN"), None)

    def _error_ref(self, fila):
        ev = self._evento_de_fila(fila)
        it = self.tabla.item(fila, 8)
        txt = it.text().replace(",", ".").strip() if it else ""
        self.tabla.blockSignals(True)
        try:
            ref = float(txt)
            if ev is None or not math.isfinite(ev.v_median_mps) or ref <= 0:
                raise ValueError
            err = (ev.v_median_mps * KMH - ref) / ref
            c = "#3fb950" if abs(err) < 0.02 else ("#d29922" if abs(err) < 0.05 else "#f85149")
            self.tabla.setItem(fila, 9, self._item(f"{err * 100:+.1f} %", color=c))
        except ValueError:
            self.tabla.setItem(fila, 9, self._item(""))
        self.tabla.blockSignals(False)
        self.tabla.resizeColumnToContents(9)

    def _registrar(self, ev: PassEvent):
        """Registro automático de eventos de la sesión (no se pierde aunque se cierre el visor)."""
        if self.archivo_sesion is None:
            carpeta = self.carpeta / "pruebas_paso"
            carpeta.mkdir(parents=True, exist_ok=True)
            self.archivo_sesion = carpeta / f"eventos_{dt.datetime.now():%Y%m%d_%H%M%S}.csv"
            with open(self.archivo_sesion, "w", newline="") as fh:
                csv.writer(fh).writerow(["hora_registro", "pass_id", "evento", "t_start", "t_end", "duracion_s",
                                         "v_mediana_kmh", "v_min_kmh", "v_max_kmh", "n_escaneos", "n_validos",
                                         "huecos", "banderas", "ext_inicial", "ext_final", "fuente"])
            self.v._log(f"Registro de eventos: {self.archivo_sesion}", "info")
        with open(self.archivo_sesion, "a", newline="") as fh:
            km = lambda v: f"{v * KMH:.3f}" if math.isfinite(v) else ""   # noqa: E731
            ext = lambda e: "" if e is None else f"{e[0]:.2f};{e[1]:.2f}"   # noqa: E731
            csv.writer(fh).writerow([
                dt.datetime.now().isoformat(timespec="seconds"), ev.pass_id, ev.event, f"{ev.t_start:.3f}",
                f"{ev.t_end:.3f}" if math.isfinite(ev.t_end) else "", f"{ev.duration_s:.3f}" if math.isfinite(ev.duration_s) else "",
                km(ev.v_median_mps), km(ev.v_min_mps), km(ev.v_max_mps), ev.n_scans, ev.n_valid, ev.n_gaps,
                "|".join(ev.flags.nombres()), ext(ev.ext_start), ext(ev.ext_end), self.v.fuente.currentText()])

    def exportar(self):
        if self.tabla.rowCount() == 0:
            QtWidgets.QMessageBox.information(self, "Exportar", "Todavía no hay pasos en la tabla.")
            return
        carpeta = self.carpeta / "pruebas_paso"
        carpeta.mkdir(parents=True, exist_ok=True)
        ruta, _ = QtWidgets.QFileDialog.getSaveFileName(
            self, "Exportar pasos", str(carpeta / f"pasos_{dt.datetime.now():%Y%m%d_%H%M%S}.csv"), "CSV (*.csv)")
        if not ruta:
            return
        p = self.proc.p
        with open(ruta, "w", newline="") as fh:
            w = csv.writer(fh)
            w.writerow([f"# fuente={self.v.fuente.currentText()}", f"H={p.mount_height_m}",
                        f"abajo_ros_deg={nadir_ros_deg(p):.2f}", f"roi=[{p.roi_x_min_m},{p.roi_x_max_m}]",
                        f"range_scale={p.range_scale}", f"range_bias_m={p.range_bias_m}"])
            w.writerow(["pass_id", "inicio", "duracion_s", "v_mediana_kmh", "v_min_kmh", "v_max_kmh", "n_validos",
                        "n_escaneos", "huecos", "banderas", "valido", "v_ref_kmh", "error_pct", "notas"])
            for f in range(self.tabla.rowCount()):
                ev = self._evento_de_fila(f)
                if ev is None:
                    continue
                ref = self.tabla.item(f, 8).text().replace(",", ".").strip()
                err = self.tabla.item(f, 9).text().replace("%", "").strip()
                km = lambda v: f"{v * KMH:.3f}" if math.isfinite(v) else ""   # noqa: E731
                w.writerow([ev.pass_id, dt.datetime.fromtimestamp(ev.t_start).isoformat(timespec="milliseconds"),
                            f"{ev.duration_s:.3f}", km(ev.v_median_mps), km(ev.v_min_mps), km(ev.v_max_mps),
                            ev.n_valid, ev.n_scans, ev.n_gaps, "|".join(ev.flags.nombres()), int(ev.valid),
                            ref, err, self.tabla.item(f, 10).text()])
        self.v._log(f"Pasos exportados: {ruta}", "ok")

    def nueva_sesion(self):
        if self.tabla.rowCount() and QtWidgets.QMessageBox.question(
                self, "Nueva sesión", "¿Vaciar la tabla de pasos? (el registro automático en CSV se conserva)"
        ) != QtWidgets.QMessageBox.StandardButton.Yes:
            return
        self.tabla.setRowCount(0)
        self.filas.clear()
        self.eventos_sesion.clear()
        self.archivo_sesion = None
        self.aplicar(silencioso=True)
        self.v._log("Nueva sesión de pruebas de paso.", "info")
