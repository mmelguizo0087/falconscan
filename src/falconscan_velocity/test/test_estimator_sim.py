"""Pruebas de extremo a extremo contra el simulador (regresión, NO validación de campo).

Los umbrales aquí son de regresión sobre datos sintéticos; los criterios de
aceptación reales se fijan con el sensor físico (reporte §8.5).
"""
from dataclasses import replace

import numpy as np
import pytest

from falconscan_velocity.core import Status, VelocityEstimator, VelocityParams
from falconscan_velocity.sim import LidarSim, LidarSimConfig, TruckMotion


TRUCK_LEN = 7.5


def _pass(motion, duration, params=None, cfg=None):
    p = params or VelocityParams()
    sim = LidarSim(cfg or LidarSimConfig())
    est = VelocityEstimator(p)
    out = []
    for s, _ in sim.run(motion, duration):
        front = float(motion.x(np.array([s.stamp]))[0])
        in_roi = front > p.roi_x_min_m + 0.5 and front - TRUCK_LEN < p.roi_x_max_m - 0.5
        out.append((est.process(s), motion, in_roi))
    return out


def _metrics(results):
    """(fracción válida mientras la volqueta está en la ROI, errores vs. verdad en mid_stamp)."""
    in_roi = [e for e, _, inside in results if inside]
    ok = [(e.velocity_mps, m.velocity(e.mid_stamp)) for e, m, _ in results if e.valid]
    ok = np.array(ok)
    return sum(e.valid for e in in_roi) / max(1, len(in_roi)), ok[:, 0] - ok[:, 1]


@pytest.mark.parametrize("v", [0.8, 1.5, 3.0, 5.0])
def test_constant_speed(v):
    frac_ok, err = _metrics(_pass(TruckMotion(v=v), 26.5 / v))
    assert frac_ok > 0.6
    assert abs(np.median(err)) / v < 0.01         # sesgo < 1 %
    assert np.sqrt(np.mean(err ** 2)) / v < 0.05  # RMSE por intervalo < 5 %


def test_deskew_removes_rotation_bias_at_high_speed():
    v = 5.0
    _, err_off = _metrics(_pass(TruckMotion(v=v), 26.5 / v, VelocityParams(deskew_iterations=0)))
    _, err_on = _metrics(_pass(TruckMotion(v=v), 26.5 / v))
    assert abs(np.mean(err_on)) < abs(np.mean(err_off)) / 3


def _time_reversed(scan):
    """Mismo escaneo con el arreglo invertido, como lo publica sllidar_ros2 (inverted:=false).

    La geometría es idéntica (ángulos recorridos al revés); solo cambia el orden temporal.
    """
    n = scan.ranges.size
    return replace(scan, ranges=scan.ranges[::-1].copy(),
                   angle_min=scan.angle_min + (n - 1) * scan.angle_increment,
                   angle_increment=-scan.angle_increment)


@pytest.mark.parametrize("flag", [True, False])
def test_deskew_with_reversed_scan_order(flag):
    v = 5.0
    motion = TruckMotion(v=v)
    sim = LidarSim(LidarSimConfig())
    est = VelocityEstimator(VelocityParams(scan_time_reversed=flag))
    err = np.array([e.velocity_mps - motion.velocity(e.mid_stamp)
                    for e in (est.process(_time_reversed(s)) for s, _ in sim.run(motion, 26.5 / v))
                    if e.valid])
    _, err_off = _metrics(_pass(motion, 26.5 / v, VelocityParams(deskew_iterations=0)))
    if flag:   # orden declarado correctamente: el deskew funciona igual que en orden directo
        assert abs(np.mean(err)) < abs(np.mean(err_off)) / 3
    else:      # orden equivocado: el deskew empeora el sesgo en vez de corregirlo
        assert abs(np.mean(err)) > abs(np.mean(err_off))


def _range_error(scan, scale=1.004265, bias=0.0027):
    """Rangos con el error medido en la prueba de pared (unidad 1): r_medido = scale·r + bias."""
    return replace(scan, ranges=scale * scan.ranges + bias)


@pytest.mark.parametrize("corrected", [True, False])
def test_range_calibration_removes_scale_bias(corrected):
    v = 3.0
    motion = TruckMotion(v=v)
    p = VelocityParams(range_bias_m=0.0027, range_scale=1 / 1.004265) if corrected else VelocityParams()
    est = VelocityEstimator(p)
    err = np.array([e.velocity_mps - motion.velocity(e.mid_stamp)
                    for e in (est.process(_range_error(s)) for s, _ in LidarSim(LidarSimConfig()).run(motion, 26.5 / v))
                    if e.valid])
    _, err_ref = _metrics(_pass(motion, 26.5 / v))
    shift = (np.mean(err) - np.mean(err_ref)) / v
    if corrected:
        assert abs(shift) < 0.001
    else:   # sin corregir, el error de escala del rango pasa a la velocidad
        assert shift > 0.002


def test_stationary_truck_reads_zero():
    res = _pass(TruckMotion(x0=2.0, v=0.0), 3.0)
    v = np.array([e.velocity_mps for e, _, _ in res if e.valid])
    assert v.size > 10
    assert np.max(np.abs(v)) < 0.05


def test_empty_scene_reports_no_object():
    res = _pass(TruckMotion(x0=-40.0, v=0.0), 1.0)
    assert all(e.status == Status.NO_OBJECT for e, _, _ in res)


def test_acceleration_is_tracked():
    # arranca a 0.5 m/s y acelera a 0.6 m/s² (camión arrancando bajo el pórtico)
    motion = TruckMotion(position=lambda t: -9.0 + 0.5 * t + 0.3 * t ** 2)
    frac_ok, err = _metrics(_pass(motion, 9.0))
    assert frac_ok > 0.5
    assert np.sqrt(np.mean(err ** 2)) < 0.08


def test_robust_to_heavy_dropout():
    cfg = LidarSimConfig(dropout_prob=0.15, outlier_prob=0.01)
    frac_ok, err = _metrics(_pass(TruckMotion(v=1.5), 18.0, cfg=cfg))
    assert frac_ok > 0.4
    assert abs(np.median(err)) / 1.5 < 0.02
