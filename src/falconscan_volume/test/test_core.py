"""Pruebas unitarias del núcleo de volumen: sección, odometría e integración."""
import math

import numpy as np
import pytest

from falconscan_volume.core import (Profile, Section, SectionProcessor, VelocityHistory, VelocitySample, VolumeParams,
                                    integrate)
from falconscan_volume.sim import Box3D, DumpTruck3D, Motion3D, PorticoSim, PorticoSimConfig

CLEAN = PorticoSimConfig(range_noise_m=0.0, dropout_prob=0.0, outlier_prob=0.0)


def _section_at(obj, s_cut, cfg=CLEAN, y_offset=0.0):
    """Sección del objeto quieto cortado por el plano a s_cut del frente."""
    sim = PorticoSim(cfg, obj, Motion3D(direction=1, v=0.0, d0=s_cut, y_offset=y_offset))
    return SectionProcessor(VolumeParams()).process(sim.volume_scan(0.0))


@pytest.mark.parametrize("s_cut", [1.0, 3.0, 4.0, 5.0, 6.5])
def test_section_area_matches_truth(s_cut):
    truck = DumpTruck3D()
    sec = _section_at(truck, s_cut)
    w = np.arange(-1.6, 1.6, 0.001)
    truth = truck.height(np.full_like(w, s_cut), w).sum() * 0.001
    assert sec.occupied and not sec.truncated
    assert sec.area_m2 == pytest.approx(truth, rel=0.005)
    assert sec.yc == pytest.approx(0.0, abs=0.02)


def test_box_section_edges_have_point_resolution():
    sec = _section_at(Box3D(2.0, 2.45, 2.0), 1.0)
    assert sec.width == pytest.approx(2.45, abs=0.02)        # la malla es de 2 cm
    assert sec.area_m2 == pytest.approx(2.45 * 2.0, rel=0.01)


def test_empty_scene_is_empty_not_no_data():
    sec = _section_at(Box3D(), -10.0)                           # objeto lejos del plano
    assert not sec.occupied and not sec.no_data
    assert abs(sec.floor_dev_m) < 0.01 and not sec.floor_drift


def test_lost_scan_is_no_data():
    sim = PorticoSim(PorticoSimConfig(scan_dropout_prob=1.0), DumpTruck3D(), Motion3D(v=0.0, d0=3.0))
    sec = SectionProcessor(VolumeParams()).process(sim.volume_scan(0.0))
    assert sec.no_data and not sec.occupied


def test_off_center_is_truncated():
    sec = _section_at(DumpTruck3D(), 4.0, y_offset=1.0)
    assert sec.occupied and sec.truncated


def test_floor_drift_detected():
    p = VolumeParams(range_bias_m=0.10)                         # sensor "se movió" 10 cm
    sim = PorticoSim(CLEAN, Box3D(), Motion3D(v=0.0, d0=-10.0))
    sec = SectionProcessor(p).process(sim.volume_scan(0.0))
    assert sec.floor_drift


def test_range_scale_corrects_a_sensor_that_measures_long():
    """Sensor que mide 0.43 % de más (unidad 1, D11): con range_scale el área vuelve a la verdad."""
    sim = PorticoSim(CLEAN, Box3D(2.0, 2.45, 2.0), Motion3D(v=0.0, d0=1.0))
    scan = sim.volume_scan(0.0)
    largo = type(scan)(scan.stamp, scan.angle_min, scan.angle_increment, scan.ranges * 1.004265,
                       scan.time_increment, scan.range_min, scan.range_max)
    sin = SectionProcessor(VolumeParams()).process(largo)
    con = SectionProcessor(VolumeParams(range_scale=1 / 1.004265)).process(largo)
    ref = SectionProcessor(VolumeParams()).process(scan)
    assert con.area_m2 == pytest.approx(ref.area_m2, rel=1e-3)
    assert abs(sin.area_m2 - ref.area_m2) > 3 * abs(con.area_m2 - ref.area_m2)



def test_fan_of_interest_drops_rays_outside():
    """Abanico del transversal (D25): uno que cubre la ROI no cambia nada; uno estrecho recorta el objeto."""
    scan = PorticoSim(CLEAN, Box3D(2.0, 2.45, 2.0), Motion3D(v=0.0, d0=1.0)).volume_scan(0.0)
    ref = SectionProcessor(VolumeParams()).process(scan)
    ancho = SectionProcessor(VolumeParams(fan_angle_rad=math.radians(60))).process(scan)
    estrecho = SectionProcessor(VolumeParams(fan_angle_rad=math.radians(30))).process(scan)
    assert ancho.area_m2 == pytest.approx(ref.area_m2, rel=1e-6)
    assert estrecho.occupied and estrecho.width < 1.7          # techo a 3 m del sensor: ±3·tan 15° = ±0.80 m
    # Con menos de vol_min_data_fraction de la ROI dentro del abanico, todo escaneo es SIN DATO
    assert SectionProcessor(VolumeParams(fan_angle_rad=math.radians(20))).process(scan).no_data

def test_nominal_is_arrow_to_floor():
    """Flecha 0° del S2E al piso = rayo ROS 180° (D23): +π/2, también en el simulador del pórtico."""
    assert VolumeParams().angle_offset_rad == pytest.approx(math.pi / 2)
    assert PorticoSimConfig().vol_nadir_angle_deg == 180.0


# ------------------------------------------------------------------ odometría
def _hist(v=2.0, n=30, dt=0.1, valid=lambda k: True):
    h = VelocityHistory()
    for k in range(n):
        h.append(VelocitySample(k * dt, k * dt, v if valid(k) else math.nan, valid(k)))
    return h


def test_positions_integrate_velocity():
    h = _hist(v=2.0)
    pos = h.positions([0.5, 1.5], direction=1, max_gap_s=0.5)
    assert pos.x[1] - pos.x[0] == pytest.approx(2.0)
    assert not pos.missing.any() and not pos.interpolated.any()


def test_reverse_subtracts_and_direction_sign():
    h = VelocityHistory()
    for k in range(30):
        t = k * 0.1
        h.append(VelocitySample(t, t, -2.0 if t < 1.5 else 1.0, True))   # avanza en -x y retrocede
    pos = h.positions([0.0, 1.5, 2.5], direction=-1, max_gap_s=0.5)
    # v lineal entre muestras: de 1.4 a 1.5 s pasa de -2 a +1 (media rampa = +0.05 m de avance)
    assert pos.x[1] - pos.x[0] == pytest.approx(2.0 * 1.4 + 0.05)
    assert pos.x[2] - pos.x[1] == pytest.approx(-1.0)                    # la reversa resta


def test_velocity_gaps():
    h = _hist(valid=lambda k: not 10 <= k < 18)                          # 0.8 s sin velocidad
    pos = h.positions([1.4], direction=1, max_gap_s=1.0)
    assert pos.interpolated[0] and not pos.missing[0]
    pos = h.positions([1.4], direction=1, max_gap_s=0.5)
    assert pos.missing[0]


def test_edge_crossing():
    h = VelocityHistory()
    for k in range(20):
        t = k * 0.1
        h.append(VelocitySample(t, t, 2.0, True, x_start=-3.0, x_end=-1.0 + 2.0 * t))
    assert h.edge_crossing(0.0, 1, "front", 0.0, 5.0) == pytest.approx(0.5)
    assert math.isnan(h.edge_crossing(0.0, 1, "tail", 0.0, 5.0))


# ------------------------------------------------------------------ integración
def _sec(area, occupied=True):
    return Section(stamp=0.0, occupied=occupied, area_m2=area, y0=-1.0, y1=1.0,
                   h=np.zeros(3) if occupied else np.zeros(3))


def test_integrate_box_midpoint_and_edges():
    # caja de 4 m (x en [0.05, 4.05]) con A = 5 m², perfiles cada 0.3 m
    xs = np.arange(-0.25, 4.6, 0.3)
    prof = [Profile(x, _sec(5.0, 0.05 <= x < 4.05)) for x in xs]
    r = integrate(prof, merge_dx=0.02)
    assert r.edge_method == "punto_medio"
    assert r.volume_m3 == pytest.approx(20.0, abs=5.0 * 0.3)
    r = integrate(prof, merge_dx=0.02, x_front=0.05, x_tail=4.05)
    assert r.edge_method == "velocidad"
    assert r.volume_m3 == pytest.approx(20.0, rel=1e-9)
    assert r.length_m == pytest.approx(4.0)


def test_integrate_stop_and_reverse_profiles_are_merged():
    xs = [0.0, 0.3, 0.6, 0.6, 0.6, 0.601, 0.4, 0.6, 0.9, 1.2, 1.5]     # se detiene y retrocede
    prof = [Profile(x, _sec(2.0, 0.1 < x < 1.4)) for x in xs]
    r = integrate(prof, merge_dx=0.02, x_front=0.1, x_tail=1.4)
    assert r.volume_m3 == pytest.approx(2.0 * 1.3)


def test_edge_outside_bracket_falls_back():
    xs = np.arange(0.0, 3.0, 0.3)
    prof = [Profile(x, _sec(1.0, 0.5 < x < 2.5)) for x in xs]
    r = integrate(prof, merge_dx=0.02, x_front=-1.0, x_tail=2.55)   # frente imposible
    assert not r.front_edge_used and r.tail_edge_used and r.edge_method == "mixto"
