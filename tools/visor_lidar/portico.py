"""Los dos LiDAR del pórtico dentro del visor: drivers, tópicos y buffers de escaneos.

Direcciones (D22): el LONGITUDINAL (velocidad) está en 192.168.8.8 y el TRANSVERSAL (volumen) en
192.168.11.2. Cada driver sllidar_ros2 publica en el tópico que usan portico.launch.py y los
nodos (/lidar_velocity/scan, /lidar_volume/scan), así que lo que se enciende aquí sirve a la
pestaña «Calibración» y a «Volumen 3D» (fuente ROS). Si ya hay quien publique esos tópicos
(p. ej. portico.launch.py en otra terminal), no se lanza nada: solo se escucha.
"""
from __future__ import annotations

import subprocess
import threading
import time
from collections import deque
from dataclasses import dataclass

import numpy as np
from pyqtgraph.Qt import QtCore

from falconscan_velocity.core import Scan

BUFFER = 120       # 12 s a 10 Hz: alcanza para la captura más larga de la calibración


@dataclass(frozen=True)
class Lidar:
    clave: str             # "vel" | "vol"
    nombre: str
    ip: str
    topico: str
    frame: str
    nodo: str


VEL = Lidar("vel", "velocidad (longitudinal)", "192.168.8.8", "/lidar_velocity/scan", "lidar_velocity",
            "sllidar_velocity")
VOL = Lidar("vol", "volumen (transversal)", "192.168.11.2", "/lidar_volume/scan", "lidar_volume", "sllidar_volume")
LIDARS = (VEL, VOL)


def cmd_driver(l: Lidar) -> list[str]:
    """sllidar_node con los mismos parámetros que velocity_s2e.launch.py / volume_lidar.launch.py."""
    return ["ros2", "run", "sllidar_ros2", "sllidar_node", "--ros-args", "-r", f"__node:={l.nodo}",
            "-r", f"scan:={l.topico}", "-p", "channel_type:=udp", "-p", f"udp_ip:={l.ip}", "-p", "udp_port:=8089",
            "-p", f"frame_id:={l.frame}", "-p", "inverted:=false", "-p", "angle_compensate:=true",
            "-p", "scan_mode:=Sensitivity"]


class Flujo:
    """Últimos escaneos de un LiDAR (los escriben el hilo de ROS o el simulador)."""

    def __init__(self):
        self._lock = threading.Lock()
        self.buffer: deque[Scan] = deque(maxlen=BUFFER)
        self.recibidos = 0
        self.t_rx: deque[float] = deque(maxlen=30)

    def agregar(self, scan: Scan):
        with self._lock:
            self.buffer.append(scan)
            self.recibidos += 1
            self.t_rx.append(time.monotonic())

    def limpiar(self):
        with self._lock:
            self.buffer.clear()
            self.t_rx.clear()

    def estado(self) -> tuple[int, Scan | None]:
        with self._lock:
            return self.recibidos, (self.buffer[-1] if self.buffer else None)

    def ultimos(self, n: int) -> list[Scan]:
        with self._lock:
            return list(self.buffer)[-n:]

    def hz(self) -> float:
        """Escaneos por segundo de llegada (0 si no llega nada hace más de 2 s)."""
        with self._lock:
            t = list(self.t_rx)
        if len(t) < 2 or time.monotonic() - t[-1] > 2.0:
            return 0.0
        return (len(t) - 1) / (t[-1] - t[0]) if t[-1] > t[0] else 0.0


def scan_de_msg(msg) -> Scan:
    return Scan(stamp=msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9, angle_min=msg.angle_min,
                angle_increment=msg.angle_increment, ranges=np.asarray(msg.ranges, dtype=np.float32),
                time_increment=msg.time_increment, range_min=msg.range_min, range_max=msg.range_max)


class Portico(QtCore.QObject):
    """Drivers y suscripciones de los dos LiDAR. ``crear_proceso(nombre)`` devuelve un Proceso del visor."""
    cambio = QtCore.Signal()
    log = QtCore.Signal(str, str)

    def __init__(self, nodo_ros, crear_proceso, parent=None):
        super().__init__(parent)
        self.nodo = nodo_ros
        self.flujos = {l.clave: Flujo() for l in LIDARS}
        self.procesos = {}
        for l in LIDARS:
            p = crear_proceso(f"driver {l.nombre}")
            p.linea.connect(lambda txt, l=l: self._linea(l, txt))
            p.termino.connect(lambda codigo, l=l: self._termino(l, codigo))
            self.procesos[l.clave] = p
        self._subs = []
        self.externo = False
        self.encendido = False

    # ------------------------------------------------------------------ estado
    def drivers_propios(self) -> bool:
        return any(p.activo() for p in self.procesos.values())

    def hz(self, clave: str) -> float:
        return self.flujos[clave].hz()

    # ------------------------------------------------------------------ control
    def encender(self):
        """Escucha los dos tópicos; lanza los drivers que falten (los que respondan al ping)."""
        if self.encendido:
            return
        self.encendido = True
        self._suscribir()
        lanzados = []
        for l in LIDARS:
            if self.nodo.count_publishers(l.topico) > 0:
                self.externo = True
                self.log.emit(f"Ya hay quien publica {l.topico} (p. ej. portico.launch.py): solo escucho.", "aviso")
                continue
            if subprocess.run(["ping", "-c", "1", "-W", "1", l.ip], capture_output=True).returncode != 0:
                self.log.emit(f"El LiDAR de {l.nombre} no responde en {l.ip}: revisa alimentación, cable y que "
                              f"eth0 tenga IP en esa red (perfil «lidar»).", "error")
                continue
            cmd = cmd_driver(l)
            self.log.emit("$ " + " ".join(cmd), "info")
            self.procesos[l.clave].iniciar(cmd)
            lanzados.append(l.nombre)
        if lanzados:
            self.log.emit(f"Drivers lanzados: {', '.join(lanzados)} (los datos llegan en ~3 s).", "ok")
        self.cambio.emit()

    def apagar(self):
        self._desuscribir()
        for p in self.procesos.values():
            p.detener()
        self.encendido = False
        self.externo = False
        self.cambio.emit()

    def cerrar(self):
        self._desuscribir()
        for p in self.procesos.values():
            p.detener(esperar_ms=6000)

    def _suscribir(self):
        from rclpy.qos import qos_profile_sensor_data
        from sensor_msgs.msg import LaserScan
        self._desuscribir()
        for l in LIDARS:
            f = self.flujos[l.clave]
            f.limpiar()
            self._subs.append(self.nodo.create_subscription(
                LaserScan, l.topico, lambda m, f=f: f.agregar(scan_de_msg(m)), qos_profile_sensor_data))

    def _desuscribir(self):
        for s in self._subs:
            self.nodo.destroy_subscription(s)
        self._subs = []

    def _linea(self, l: Lidar, txt: str):
        msg = txt.split("]: ", 1)[-1].strip()
        tipo = "error" if ("[ERROR]" in txt or "Error" in msg) else ("aviso" if "[WARN]" in txt else "info")
        self.log.emit(f"[{l.clave}] {msg}", tipo)

    def _termino(self, l: Lidar, codigo: int):
        if self.encendido:
            self.log.emit(f"El driver del LiDAR de {l.nombre} terminó (código {codigo}).", "error")
        self.cambio.emit()
