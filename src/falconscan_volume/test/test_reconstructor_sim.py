"""Máquina de estados y cadena completa contra el simulador 3D (regresión, NO validación de campo).

Los umbrales son de regresión sobre datos sintéticos. La matriz completa está en
tools/evaluate_volume_sim.py.
"""
import math

import pytest

from falconscan_velocity.core import PassEvent, Scan
from falconscan_volume.core import VelocitySample, VolState, VolumeParams, VolumeReconstructor
from falconscan_volume.sim import (Box3D, DumpTruck3D, Motion3D, PorticoSim, PorticoSimConfig, lane_change,
                                   run_scenario)

TRUCK = DumpTruck3D()


def _run(motion, cfg=None, obj=TRUCK, **kw):
    v = max(0.5, abs(motion.speed(3.0)))
    sim = PorticoSim(cfg or PorticoSimConfig(), obj, motion)
    return run_scenario(sim, (abs(motion.d0) + obj.length + 7.0) / v, **kw)


@pytest.mark.parametrize("direction", [1, -1])
def test_pass_measures_envelope_volume(direction):
    r = _run(Motion3D(direction=direction, v=7.5 / 3.6, d0=-6.0))
    assert len(r.measurements) == 1 and r.captures == 1
    m = r.m
    assert m.result in (0, 1), m.flags                     # válida (puede llevar advertencias)
    assert m.direction == direction and m.pass_id == 1
    assert abs(r.error_pct) < 3.0
    assert m.length_m == pytest.approx(TRUCK.length, abs=0.1)
    assert m.edge_method == "velocidad"
    # la reconstrucción queda en el marco del paso: frente en x = 0, cola en x ≈ −L
    pts = m.reconstruction.points
    assert pts[:, 0].max() == pytest.approx(0.0, abs=0.15)
    assert pts[:, 0].min() == pytest.approx(-TRUCK.length, abs=0.3)
    assert pts[:, 2].max() == pytest.approx(3.2, abs=0.2)


def test_lane_change_is_not_parallel():
    r = _run(Motion3D(v=10 / 3.6, d0=-6.0, lateral=lane_change(0.9, -2.0, 6.0)))
    assert r.m.result == 2 and "CONDUCCION_NO_PARALELA" in r.m.flags


def test_off_center_invalidates():
    r = _run(Motion3D(v=10 / 3.6, d0=-6.0, y_offset=0.6))
    assert r.m.result == 2 and "DESCENTRADO" in r.m.flags


def test_clock_offset_is_detected():
    r = _run(Motion3D(v=10 / 3.6, d0=-6.0), PorticoSimConfig(vol_clock_offset_s=0.25))
    assert r.m.result == 2 and "DESINCRONIZADO" in r.m.flags


def test_lost_scans_do_not_remove_volume():
    r = _run(Motion3D(v=7.5 / 3.6, d0=-6.0), PorticoSimConfig(scan_dropout_prob=0.05, seed=3))
    assert r.m.result in (0, 1) and abs(r.error_pct) < 3.0


def test_person_is_discarded():
    r = _run(Motion3D(v=1.2, d0=-5.0), obj=Box3D(0.4, 0.6, 1.7))
    assert all(m.result == 3 for m in r.measurements)


# ------------------------------------------------------------------ máquina de estados
def _scan(t, ranges):
    """±0.5 rad alrededor del nadir, que el S2E con la flecha al piso publica en ROS 180° (D23)."""
    return Scan(stamp=t, angle_min=math.pi - 0.5, angle_increment=0.01, ranges=ranges, time_increment=0.0,
                range_min=0.05, range_max=30.0)


def test_sensor_and_velocity_timeouts():
    import numpy as np
    rec = VolumeReconstructor(VolumeParams())
    floor = np.full(101, 5.0, dtype=np.float32) / np.cos(-0.5 + 0.01 * np.arange(101))
    rec.tick(0.0)
    assert rec.state == VolState.SIN_DATOS and not rec.ready
    t = 0.0
    for k in range(6):
        t = 0.1 * k
        rec.on_velocity(VelocitySample(t, math.nan, math.nan, False), now=t)
        rec.on_scan(_scan(t, floor), now=t)
    assert rec.state == VolState.VACIO and rec.ready
    rec.tick(t + 0.3)                                          # llegan escaneos pero no velocidad
    rec.on_scan(_scan(t + 0.3, floor), now=t + 0.3)
    rec.tick(t + 0.7)
    assert rec.state == VolState.SIN_VELOCIDAD and not rec.ready
    rec.on_velocity(VelocitySample(t + 0.8, math.nan, math.nan, False), now=t + 0.8)
    assert rec.state == VolState.ARRANQUE


def test_pass_event_association():
    rec = VolumeReconstructor(VolumeParams())
    rec.on_pass_event(PassEvent("INICIO", 7, -1, 0.0))
    assert rec._open_pass.pass_id == 7
    rec.on_pass_event(PassEvent("FIN", 7, -1, 0.0, 5.0, ["REVERSA"]))
    assert rec._open_pass is None
