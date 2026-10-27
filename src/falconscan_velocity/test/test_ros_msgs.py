"""Los mensajes ROS de paso deben coincidir con el núcleo (constantes y conversión).

Se saltan si falconscan_interfaces no está compilado y cargado (`source install/setup.bash`).
"""
import math

import pytest

msgs = pytest.importorskip("falconscan_interfaces.msg")
pytest.importorskip("rclpy")

from std_msgs.msg import Header  # noqa: E402

from falconscan_velocity.core import PassEvent, PassState, PassTracker  # noqa: E402
from falconscan_velocity.velocity_node import pass_event_msg, pass_state_msg  # noqa: E402


def _hdr(sec=0, frame="lidar"):
    h = Header()
    h.stamp.sec, h.frame_id = sec, frame
    return h


def test_constantes_de_estado_coinciden_con_el_nucleo():
    for s in PassState:
        assert getattr(msgs.PassState, s.name) == int(s)


def test_mensaje_de_estado():
    m = pass_state_msg(PassTracker(), _hdr(5))
    assert m.state == msgs.PassState.ARRANQUE and m.state_text == "ARRANQUE"
    assert not m.ready and not m.pass_open and m.pass_id == 0 and m.direction == 0
    assert m.header.frame_id == "lidar" and m.header.stamp.sec == 5


def test_mensaje_de_evento_fin():
    ev = PassEvent("FIN", 7, -1, t_start=100.25, t_end=108.5, flags=["REVERSA", "TIMEOUT"], n_gaps=1,
                   n_scans=80, n_valid=60, v_median_mps=1.5, v_min_mps=1.2, v_max_mps=1.7,
                   ext_start=(2.4, 3.0), ext_end=None)
    m = pass_event_msg(ev, _hdr(108))
    assert m.event == msgs.PassEvent.EVENT_FIN and m.event_text == "FIN" and m.pass_id == 7 and not m.valid
    assert list(m.flags) == ["REVERSA", "TIMEOUT"] and m.direction == -1
    assert m.t_start.sec == 100 and m.t_start.nanosec == 250_000_000 and m.t_end.sec == 108
    assert m.duration_s == pytest.approx(8.25)
    assert m.n_scans == 80 and m.n_valid == 60 and m.v_median_mps == 1.5    # rapidez de avance
    assert m.start_x_start_m == pytest.approx(2.4) and math.isnan(m.end_x_start_m)


def test_mensaje_de_evento_fin_valido():
    m = pass_event_msg(PassEvent("FIN", 2, 1, t_start=1.0, t_end=5.0), _hdr())
    assert m.valid and list(m.flags) == []


def test_mensaje_de_evento_inicio():
    m = pass_event_msg(PassEvent("INICIO", 1, 1, t_start=10.0), _hdr())
    assert m.event == msgs.PassEvent.EVENT_INICIO and not m.valid
    assert m.t_end.sec == 0 and math.isnan(m.duration_s)
