"""PassTracker y VelocityPipeline sobre el simulador (regresión, NO validación de campo)."""
import numpy as np
import pytest

from falconscan_velocity.core import (PassState, PassTracker, Status, VelocityEstimate, VelocityParams,
                                      VelocityPipeline)
from falconscan_velocity.sim import LidarSim, LidarSimConfig, TruckMotion


def _run(motion, duration, cfg=None, params=None):
    pipe = VelocityPipeline(params or VelocityParams())
    sim = LidarSim(cfg or LidarSimConfig())
    events, states, ests = [], [], []
    for scan, _ in sim.run(motion, duration):
        e, ev = pipe.process(scan)
        events += ev
        states.append(pipe.tracker.state)
        ests.append(e)
    return pipe, events, states, ests


@pytest.mark.parametrize("direction", [1, -1])
@pytest.mark.parametrize("v", [1.39, 2.78])
def test_one_pass_each_direction(direction, v):
    motion = TruckMotion(x0=-10.0 * direction, v=v * direction)
    pipe, events, states, ests = _run(motion, 26.5 / v, LidarSimConfig(direction=direction))
    assert [e.kind for e in events] == ["INICIO", "FIN"]
    ini, fin = events
    assert ini.direction == direction and fin.direction == direction
    assert fin.flags == []
    assert fin.v_median_mps == pytest.approx(v, rel=0.03)      # rapidez de avance, positiva
    assert states[-1] == PassState.VACIO
    # la búsqueda según el sentido mide igual en los dos sentidos
    v_ok = [e.velocity_mps for e in ests if e.valid]
    assert np.median(v_ok) == pytest.approx(direction * v, rel=0.02)


def test_fixed_search_misses_trucks_in_minus_x():
    # comportamiento original: búsqueda [v_min, v_max] = [-0.5, 6] solo ve +x
    motion = TruckMotion(x0=10.0, v=-2.0)
    _, events, _, ests = _run(motion, 13.0, LidarSimConfig(direction=-1), VelocityParams(search_mode="fija"))
    assert sum(e.valid for e in ests) < 5
    assert any("POCOS_VALIDOS" in e.flags for e in events if e.kind == "FIN")


def _est(ext):
    return VelocityEstimate(stamp=0.0, status=Status.WARMING_UP if ext else Status.NO_OBJECT, object_extent=ext)


def _feed(tr, ext, n, t0, dt=0.1):
    ev = []
    for i in range(n):
        ev += tr.update(t0 + i * dt, _est(ext))
    return ev, t0 + n * dt


def test_false_entry_and_startup():
    tr = PassTracker()
    assert tr.state == PassState.ARRANQUE and not tr.ready
    _, t = _feed(tr, None, 5, 0.0)
    assert tr.state == PassState.VACIO and tr.ready
    ev, t = _feed(tr, (-3.0, -2.5), 2, t)               # 2 escaneos < pass_enter_scans
    ev2, t = _feed(tr, None, 1, t)
    assert ev == ev2 == [] and tr.state == PassState.VACIO and tr.false_entries == 1


def test_no_pass_from_startup_with_truck_inside():
    tr = PassTracker()
    ev, _ = _feed(tr, (-1.0, 2.0), 50, 0.0)             # camión ya dentro al arrancar
    assert ev == [] and tr.state == PassState.ARRANQUE


def test_timeout_and_sensor_loss():
    p = VelocityParams()
    tr = PassTracker(p)
    _, t = _feed(tr, None, 5, 0.0)
    ev, t = _feed(tr, (-3.0, -1.0), int(p.pass_max_s / 0.1) + 5, t)
    assert [e.kind for e in ev] == ["INICIO", "FIN"] and "TIMEOUT" in ev[1].flags
    assert tr.state == PassState.OBSTRUIDO
    _, t = _feed(tr, None, 5, t)
    assert tr.state == PassState.VACIO
    ev, t = _feed(tr, (-3.0, -1.0), 4, t)
    assert ev[0].kind == "INICIO"
    ev = tr.tick(t + 1.0)
    assert ev[0].kind == "FIN" and "SENSOR_PERDIDO" in ev[0].flags and tr.state == PassState.SIN_DATOS
    tr.update(t + 1.1, _est(None))
    assert tr.state == PassState.ARRANQUE


def test_stamps_behind_node_clock_are_not_sensor_loss():
    # driver con 2 s de latencia: los stamps van atrasados respecto al reloj del nodo
    tr = PassTracker()
    for k in range(10):
        tr.update(0.1 * k, _est(None), now=2.0 + 0.1 * k)
        assert tr.tick(2.0 + 0.1 * k + 0.05) == []
    assert tr.state == PassState.VACIO
