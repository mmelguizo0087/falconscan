"""Los mensajes ROS de paso deben coincidir con el núcleo (constantes y conversión).

Se saltan si falconscan_interfaces no está compilado y cargado (`source install/setup.bash`).
"""
import math

import pytest

msgs = pytest.importorskip("falconscan_interfaces.msg")
pytest.importorskip("rclpy")

from builtin_interfaces.msg import Time  # noqa: E402

from falconscan_velocity.core import PassEvent, PassFlag, PassState, PassTracker  # noqa: E402
from falconscan_velocity.velocity_node import pass_event_msg, pass_state_msg  # noqa: E402


def test_constantes_de_estado_coinciden_con_el_nucleo():
    for s in PassState:
        assert getattr(msgs.PassState, f"STATE_{s.name}") == int(s)


def test_constantes_de_banderas_coinciden_con_el_nucleo():
    for f in PassFlag:
        if f:
            assert getattr(msgs.PassEvent, f"FLAG_{f.name}") == int(f)


def test_mensaje_de_estado():
    tr = PassTracker()
    m = pass_state_msg(tr, Time(sec=5), "lidar")
    assert m.state == msgs.PassState.STATE_ARRANQUE and m.state_text == "ARRANQUE"
    assert not m.ready and not m.pass_open and m.pass_id == 0 and m.header.frame_id == "lidar"


def test_mensaje_de_evento_fin():
    ev = PassEvent(7, "FIN", t_start=100.25, t_end=108.5, duration_s=8.25,
                   flags=PassFlag.REVERSA | PassFlag.TIMEOUT, n_gaps=1, n_scans=80, n_valid=60,
                   v_median_mps=1.5, v_min_mps=1.2, v_max_mps=1.7, ext_start=(-3.0, -2.4), ext_end=None)
    m = pass_event_msg(ev, Time(sec=108), "lidar")
    assert m.event == msgs.PassEvent.EVENT_FIN and m.pass_id == 7 and not m.valid
    assert m.flags == msgs.PassEvent.FLAG_REVERSA | msgs.PassEvent.FLAG_TIMEOUT
    assert list(m.flag_names) == ["REVERSA", "TIMEOUT"]
    assert m.t_start.sec == 100 and m.t_start.nanosec == 250_000_000 and m.t_end.sec == 108
    assert m.n_scans == 80 and m.n_valid == 60 and m.v_median_mps == 1.5
    assert m.start_x_start_m == pytest.approx(-3.0) and math.isnan(m.end_x_start_m)


def test_mensaje_de_evento_inicio():
    m = pass_event_msg(PassEvent(1, "INICIO", t_start=10.0), Time(), "")
    assert m.event == msgs.PassEvent.EVENT_INICIO and m.t_end.sec == 0 and math.isnan(m.duration_s)


def test_mensaje_de_evento_con_referencia_invertida():
    ev = PassEvent(3, "FIN", t_start=1.0, t_end=5.0, duration_s=4.0, v_median_mps=-2.0,
                   v_min_mps=-2.3, v_max_mps=-1.8, advance_median_mps=2.0, advance_min_mps=1.8,
                   advance_max_mps=2.3, direction=-1)
    m = pass_event_msg(ev, Time(), "lidar")
    assert m.direction == -1 and m.v_median_mps == -2.0
    m = pass_event_msg(ev, Time(), "lidar", sign=-1)
    assert m.direction == 1 and m.v_median_mps == 2.0
    assert (m.v_min_mps, m.v_max_mps) == (1.8, 2.3)
    assert m.advance_median_mps == 2.0 and (m.advance_min_mps, m.advance_max_mps) == (1.8, 2.3)
