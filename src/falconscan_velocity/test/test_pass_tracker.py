"""Máquina de estados de paso (docs/maquina_estados_paso.md).

Las pruebas con secuencias sintéticas cubren cada transición y bandera; las de extremo a
extremo usan el simulador (regresión, NO evidencia de desempeño de campo).
"""
import numpy as np
import pytest

from falconscan_velocity.core import (PassFlag, PassState, PassTracker, Status, VelocityEstimate,
                                     VelocityEstimator, VelocityParams, VelocityPipeline)
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

def _simular_pasos(velocidades, separacion_s=4.0, cfg=None, reset_on_empty=False):
    """Camiones seguidos por el simulador -> (eventos, tracker)."""
    pipe = VelocityPipeline(VelocityParams(), reset_on_empty=reset_on_empty)
    sim = LidarSim(cfg or LidarSimConfig())
    eventos, t0 = [], 0.0
    for s, _ in sim.run(TruckMotion(x0=100.0, v=0.0), 1.0, t0=t0):   # ROI vacía: sale de ARRANQUE
        eventos += pipe.process(s)[1]
    t0 = 1.0
    for v in velocidades:
        dur = 26.5 / abs(v) + separacion_s
        m = TruckMotion(x0=(-10.0 if v > 0 else 16.5) - v * t0, v=v)   # v < 0: entra por x máx
        for s, _ in sim.run(m, dur, t0=t0):
            eventos += pipe.process(s)[1]
        t0 += dur
    return eventos, pipe.tracker


@pytest.mark.parametrize("v", [1.5, 2.2, 2.7])     # 5.4 / 7.9 / 9.7 km/h
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


# ---------------------------------------------------------------- ambos sentidos (D13)
ENTRA_MAX = (2.2, 3.0)     # aparece por x máx: vehículo hacia −x
SALE_MIN = (-3.0, -2.4)


def pasada_contraria(s, n_medio=30, v=-2.0):
    return s.ocupado(1, ENTRA_MAX, v).ocupado(n_medio, MEDIO, v).ocupado(1, SALE_MIN, v)


def test_paso_en_sentido_contrario_es_valido_con_sentido_negativo():
    s = pasada_contraria(listo()).vacio(5)
    ini = [e for e in s.eventos if e.event == "INICIO"]
    assert ini[0].direction == -1                       # por el borde de x máx
    (fin,) = s.fines
    assert fin.valid, fin.flags.nombres()
    assert fin.direction == -1 and fin.v_median_mps == pytest.approx(-2.0)


@pytest.mark.parametrize("secuencia, bandera", [
    (lambda s: s.ocupado(1, MEDIO, -2.0).ocupado(30, v=-2.0).ocupado(1, SALE_MIN, -2.0), PassFlag.ENTRADA_FUERA_DE_BORDE),
    (lambda s: s.ocupado(1, ENTRA_MAX, -2.0).ocupado(30, v=-2.0), PassFlag.SALIDA_FUERA_DE_BORDE),
    (lambda s: s.ocupado(1, ENTRA_MAX, -2.0).ocupado(20, v=-2.0).ocupado(3, MEDIO, v=0.5).ocupado(1, SALE_MIN, -2.0),
     PassFlag.REVERSA),
    (lambda s: pasada_contraria(s, v=-0.5), PassFlag.VELOCIDAD_FUERA_DE_RANGO),
])
def test_banderas_en_sentido_contrario(secuencia, bandera):
    s = secuencia(listo()).vacio(5)
    (fin,) = s.fines
    assert fin.flags == bandera, fin.flags.nombres()


def test_reversa_es_relativa_al_sentido_del_paso():
    """Hacia −x, velocidades negativas no son reversa (antes de D13 lo eran)."""
    s = pasada_contraria(listo()).vacio(5)
    fin = s.fines[0]
    assert PassFlag.REVERSA not in fin.flags
    assert fin.advance_median_mps == pytest.approx(2.0)            # rapidez de avance positiva


def test_sentido_se_fija_en_la_entrada_y_esta_disponible_durante_el_paso():
    s = listo().ocupado(1, ENTRA_MAX, -2.0).ocupado(5, MEDIO, -2.0)
    assert s.tr.open_pass and s.tr.direction == -1
    assert s.eventos[0].event == "INICIO" and s.eventos[0].direction == -1


def test_entra_y_sale_en_reversa_por_el_mismo_lado_es_sentido_inconsistente():
    """Entra por x mín (+1), avanza poco y sale retrocediendo por x mín."""
    s = listo().ocupado(1, ENTRA, 1.5).ocupado(3, (-2.5, -1.5), 1.5).ocupado(20, (-2.5, -1.5), -1.5) \
        .ocupado(1, SALE_MIN, -1.5).vacio(5)
    (fin,) = s.fines
    assert fin.direction == 1
    assert PassFlag.SENTIDO_INCONSISTENTE in fin.flags and PassFlag.REVERSA in fin.flags
    assert PassFlag.SALIDA_FUERA_DE_BORDE in fin.flags
    assert fin.advance_median_mps == pytest.approx(-1.5)


def test_entrada_ambigua_decide_por_las_primeras_velocidades():
    s = listo().ocupado(1, MEDIO, -2.0).ocupado(30, v=-2.0).ocupado(1, SALE_MIN, -2.0).vacio(5)
    ini = [e for e in s.eventos if e.event == "INICIO"][0]
    (fin,) = s.fines
    assert ini.direction in (0, -1) and fin.direction == -1
    assert fin.flags == PassFlag.ENTRADA_FUERA_DE_BORDE


def test_un_solo_sentido_marca_el_contrario():
    s = pasada_contraria(Secuencia(pass_both_directions=False).vacio(5)).vacio(5)
    (fin,) = s.fines
    assert fin.direction == 1
    esperadas = (PassFlag.ENTRADA_FUERA_DE_BORDE | PassFlag.SALIDA_FUERA_DE_BORDE
                 | PassFlag.VELOCIDAD_FUERA_DE_RANGO | PassFlag.REVERSA)
    assert fin.flags & esperadas == esperadas


def test_sentido_sin_velocidades_sale_del_borde():
    s = listo().ocupado(1, ENTRA_MAX, status=Status.LOW_PEAK).ocupado(30, status=Status.LOW_PEAK) \
        .ocupado(1, SALE_MIN, status=Status.LOW_PEAK).vacio(5)
    assert s.fines[0].direction == -1 and PassFlag.POCOS_VALIDOS in s.fines[0].flags


def test_referencia_invertida_cambia_signos_y_no_la_logica():
    fin = pasada_contraria(listo()).vacio(5).fines[0]
    ev = fin.with_sign(-1)
    assert ev.direction == 1 and ev.v_median_mps == pytest.approx(2.0)
    assert ev.v_min_mps == -fin.v_max_mps and ev.v_max_mps == -fin.v_min_mps
    assert ev.flags == fin.flags and ev.ext_start == fin.ext_start
    assert fin.with_sign(1) is fin


def test_simulador_ambos_sentidos_seguidos():
    eventos, tr = _simular_pasos([2.0, -2.0, 2.7, -2.7])
    fines = [e for e in eventos if e.event == "FIN"]
    assert [e.direction for e in fines] == [1, -1, 1, -1]
    assert [e.direction for e in eventos if e.event == "INICIO"] == [1, -1, 1, -1]   # por el borde
    for e, v in zip(fines, [2.0, -2.0, 2.7, -2.7]):
        assert e.valid, (e.pass_id, e.flags.nombres())
        assert e.v_median_mps == pytest.approx(v, rel=0.02)
    assert tr.n_false_entries == 0


# ---------------------------------------------------------------- reinicio del estimador (2026-10-21)
def test_hueco_dentro_del_paso_no_reinicia_el_estimador():
    """Con 30 % de pérdidas, mantener el historial en SALIENDO da más escaneos válidos que la
    política anterior (reiniciar en cada vacío), sin fragmentar el paso."""
    cfg = LidarSimConfig(dropout_prob=0.30, seed=3)
    nuevos, _ = _simular_pasos([2.0, -2.0], cfg=cfg)
    viejos, _ = _simular_pasos([2.0, -2.0], cfg=cfg, reset_on_empty=True)
    fn, fv = ([e for e in ev if e.event == "FIN"] for ev in (nuevos, viejos))
    assert len(fn) == 2
    assert sum(e.n_valid for e in fn) >= sum(e.n_valid for e in fv)


def test_estimador_se_reinicia_al_terminar_el_paso():
    pipe = VelocityPipeline(VelocityParams())
    sim = LidarSim(LidarSimConfig())
    for s, _ in sim.run(TruckMotion(x0=100.0, v=0.0), 1.0):
        pipe.process(s)
    for s, _ in sim.run(TruckMotion(x0=-10.0 - 2.0, v=2.0), 26.5 / 2.0 + 2.0, t0=1.0):
        pipe.process(s)
    assert pipe.tracker.state == PassState.VACIO
    assert len(pipe.estimator._history) == 0
