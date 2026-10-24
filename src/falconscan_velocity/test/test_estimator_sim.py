"""Pruebas de extremo a extremo contra el simulador (regresión, NO validación de campo).

Los umbrales aquí son de regresión sobre datos sintéticos; los criterios de
aceptación reales se fijan con el sensor físico (reporte §8.5).
"""
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


def _scans_with_gaps(motion, duration, empty_idx):
    """Escaneos del simulador; los índices de `empty_idx` llegan vacíos (escaneo perdido)."""
    sim = LidarSim(LidarSimConfig())
    for k, (s, _) in enumerate(sim.run(motion, duration)):
        if k in empty_idx:
            s.ranges[:] = np.inf
        yield k, s


def test_isolated_empty_scan_does_not_restart_warmup():
    # volqueta en plena ROI desde t = 0; un escaneo perdido en k = 20
    est = VelocityEstimator(VelocityParams())
    st = {k: est.process(s).status for k, s in _scans_with_gaps(TruckMotion(x0=3.0, v=1.5), 3.0, {20})}
    assert st[20] == Status.NO_OBJECT
    assert st[21] != Status.WARMING_UP
    assert st[22] != Status.WARMING_UP


def test_consecutive_empty_scans_reset_history():
    p = VelocityParams()
    gap = set(range(20, 20 + p.empty_reset_scans))
    est = VelocityEstimator(p)
    st = {k: est.process(s).status for k, s in _scans_with_gaps(TruckMotion(x0=3.0, v=1.5), 3.0, gap)}
    k = 20 + p.empty_reset_scans
    assert [st[k + i] for i in range(p.baseline_scans)] == [Status.WARMING_UP] * p.baseline_scans
