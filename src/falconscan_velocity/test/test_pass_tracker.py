"""Máquina de estados de paso (docs/maquina_estados_paso.md).

Las pruebas con secuencias sintéticas cubren cada transición y bandera; las de extremo a
extremo usan el simulador (regresión, NO evidencia de desempeño de campo).
"""
import numpy as np
import pytest

from falconscan_velocity.core import (PassFlag, PassState, PassTracker, Status, VelocityEstimate,
                                     VelocityEstimator, VelocityParams)
from falconscan_velocity.sim import LidarSim, LidarSimConfig, TruckMotion

DT = 0.1
ENTRA = (-3.0, -2.2)    # aparece por el borde de entrada (roi_x_min = -3)
MEDIO = (-1.0, 1.0)
SALE = (2.4, 3.0)       # desaparece por el borde de salida (roi_x_max = 3)


class Secuencia:
    """Alimenta el tracker a 10 Hz con escaneos sintéticos."""

    def __init__(self, params=None, **kw):
        self.tr = PassTracker(replace_params(params, **kw))
        self.t = 0.0
        self.eventos = []

    def vacio(self, n=1):
        for _ in range(n):
            self._paso(False, None, None)
        return self

    def ocupado(self, n=1, ext=MEDIO, v=2.0, status=Status.OK):
        for _ in range(n):
            est = VelocityEstimate(stamp=self.t, status=status,
                                   velocity_mps=v if status == Status.OK else float("nan"))
            self._paso(True, ext, est)
        return self

    def _paso(self, ocupado, ext, est):
        self.eventos += self.tr.update(self.t, ocupado, ext, est)
        self.t += DT

    def pasada(self, n_medio=30, v=2.0):
        """Camión completo: entra por x_min, cruza y sale por x_max."""
        return self.ocupado(1, ENTRA, v).ocupado(n_medio, MEDIO, v).ocupado(1, SALE, v)

    @property
    def fines(self):
        return [e for e in self.eventos if e.event == "FIN"]


def replace_params(params=None, **kw):
    p = params or VelocityParams()
    for k, v in kw.items():
        setattr(p, k, v)
    return p


def listo():
    """Tracker ya en VACIO."""
    s = Secuencia().vacio(5)
    assert s.tr.state == PassState.VACIO
    return s


def test_arranque_pasa_a_vacio_tras_escaneos_vacios():
    s = Secuencia().vacio(4)
    assert s.tr.state == PassState.ARRANQUE and not s.tr.ready
    s.vacio(1)
    assert s.tr.state == PassState.VACIO and s.tr.ready


def test_camion_a_medio_pasar_al_arrancar_no_se_mide():
    s = Secuencia().ocupado(20).vacio(5)
    assert s.eventos == [] and s.tr.state == PassState.VACIO


def test_paso_normal_inicio_y_fin_sin_banderas():
    s = listo()
    t0 = s.t
    s.pasada(30)
    t_ultimo = s.t - DT
    s.vacio(5)
    ini, fin = s.eventos
    assert ini.event == "INICIO" and ini.pass_id == 1 and ini.t_start == pytest.approx(t0)
    assert fin.event == "FIN" and fin.pass_id == 1 and fin.valid, fin.flags.nombres()
    assert fin.t_end == pytest.approx(t_ultimo)          # fin = último escaneo ocupado, no el de cierre
    assert fin.n_scans == 32 and fin.n_valid == 32 and fin.v_median_mps == pytest.approx(2.0)
    assert s.tr.state == PassState.VACIO


def test_inicio_tras_pass_enter_scans():
    s = listo().ocupado(2, ENTRA)
    assert s.tr.state == PassState.ENTRANDO and s.eventos == []
    s.ocupado(1, ENTRA)
    assert s.tr.state == PassState.EN_PASO and s.eventos[0].event == "INICIO"


def test_falso_positivo_vuelve_a_vacio_sin_eventos():
    s = listo().ocupado(2, ENTRA).vacio(1)
    assert s.tr.state == PassState.VACIO and s.eventos == [] and s.tr.n_false_entries == 1


def test_hueco_corto_no_parte_el_paso():
    s = listo().ocupado(10, ENTRA).vacio(4).ocupado(10).ocupado(1, SALE).vacio(5)
    assert len(s.fines) == 1 and s.fines[0].n_gaps == 1 and s.fines[0].pass_id == 1


def test_dos_camiones_seguidos_son_dos_pasos():
    s = listo().pasada().vacio(5).pasada().vacio(5)
    assert [e.pass_id for e in s.fines] == [1, 2]
    assert all(e.valid for e in s.fines)


def test_ocupacion_larga_cierra_por_timeout_y_queda_obstruido():
    s = listo().ocupado(1, ENTRA).ocupado(int(30.0 / DT) + 2)
    assert s.tr.state == PassState.OBSTRUIDO and not s.tr.ready
    assert len(s.fines) == 1 and PassFlag.TIMEOUT in s.fines[0].flags
    s.vacio(4)
    assert s.tr.state == PassState.OBSTRUIDO
    s.vacio(1)
    assert s.tr.state == PassState.VACIO and s.tr.n_obstructions == 1


def test_objeto_fijo_al_arrancar_queda_obstruido():
    s = Secuencia().ocupado(int(30.0 / DT) + 2)
    assert s.tr.state == PassState.OBSTRUIDO and s.eventos == []


def test_sensor_perdido_con_paso_abierto():
    s = listo().ocupado(10, ENTRA)
    ev = s.tr.tick(s.t + 0.3)
    assert ev == [] and s.tr.state == PassState.EN_PASO
    ev = s.tr.tick(s.t + 0.6)
    assert s.tr.state == PassState.SIN_DATOS and not s.tr.ready
    assert ev[0].event == "FIN" and PassFlag.SENSOR_PERDIDO in ev[0].flags
    s.vacio(1)
    assert s.tr.state == PassState.ARRANQUE           # T12: no reanuda el paso


def test_sin_datos_sin_paso_abierto_no_emite_fin():
    s = listo()
    assert s.tr.tick(s.t + 1.0) == [] and s.tr.state == PassState.SIN_DATOS


def test_sin_datos_si_nunca_llega_un_escaneo():
    tr = PassTracker()
    assert tr.tick(100.0) == [] and tr.state == PassState.ARRANQUE
    tr.tick(100.6)
    assert tr.state == PassState.SIN_DATOS


@pytest.mark.parametrize("secuencia, bandera", [
    (lambda s: s.ocupado(1, MEDIO).ocupado(30).ocupado(1, SALE), PassFlag.ENTRADA_FUERA_DE_BORDE),
    (lambda s: s.ocupado(1, ENTRA).ocupado(30), PassFlag.SALIDA_FUERA_DE_BORDE),
    (lambda s: s.pasada(v=0.5), PassFlag.VELOCIDAD_FUERA_DE_RANGO),
    (lambda s: s.pasada(v=6.0), PassFlag.VELOCIDAD_FUERA_DE_RANGO),
    (lambda s: s.ocupado(1, ENTRA).ocupado(20).ocupado(3, MEDIO, v=-0.5).ocupado(1, SALE), PassFlag.REVERSA),
    (lambda s: s.ocupado(1, ENTRA).ocupado(10).ocupado(20, status=Status.LOW_PEAK).ocupado(1, SALE),
     PassFlag.POCOS_VALIDOS),
])
def test_banderas(secuencia, bandera):
    s = listo()
    secuencia(s)
    s.vacio(5)
    assert len(s.fines) == 1
    assert s.fines[0].flags == bandera, s.fines[0].flags.nombres()


def test_velocidad_en_el_limite_con_tolerancia_no_es_bandera():
    s = listo().pasada(v=1.39 - 0.25).vacio(5)
    assert PassFlag.VELOCIDAD_FUERA_DE_RANGO not in s.fines[0].flags


# ----------------------------------------------------------------------------- simulador

def _simular_pasos(velocidades, separacion_s=4.0, cfg=None):
    """Camiones seguidos por el simulador -> (eventos, ventanas reales de ocupación)."""
    p = VelocityParams()
    sim = LidarSim(cfg or LidarSimConfig())
    est, tr = VelocityEstimator(p), PassTracker(p)
    eventos, t0 = [], 0.0
    for s, _ in sim.run(TruckMotion(x0=100.0, v=0.0), 1.0, t0=t0):   # ROI vacía: sale de ARRANQUE
        eventos += tr.update_from_estimate(est.process(s))
    t0 = 1.0
    for v in velocidades:
        dur = 26.5 / v + separacion_s
        m = TruckMotion(x0=-10.0 - v * t0, v=v)
        for s, _ in sim.run(m, dur, t0=t0):
            eventos += tr.update_from_estimate(est.process(s))
        t0 += dur
    return eventos, tr


@pytest.mark.parametrize("v", [1.5, 3.0, 4.0])
def test_simulador_un_paso_por_camion(v):
    eventos, tr = _simular_pasos([v, v])
    fines = [e for e in eventos if e.event == "FIN"]
    assert [e.pass_id for e in fines] == [1, 2]
    for e in fines:
        assert e.valid, e.flags.nombres()
        assert e.v_median_mps == pytest.approx(v, rel=0.02)
    assert tr.n_false_entries == 0 and tr.state == PassState.VACIO


def test_simulador_con_perdida_de_puntos_no_fragmenta():
    cfg = LidarSimConfig(dropout_prob=0.15)
    eventos, _ = _simular_pasos([2.0], cfg=cfg)
    fines = [e for e in eventos if e.event == "FIN"]
    assert len(fines) == 1, [(e.pass_id, e.flags.nombres()) for e in fines]
