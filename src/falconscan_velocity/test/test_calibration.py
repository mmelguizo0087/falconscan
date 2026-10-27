"""Calibración del montaje con el piso vacío (core/calibration.py), con escaneos sintéticos.

Convención decidida: H = cinta (fijo), se ajustan el offset de rango b y el desvío α de la flecha
0° (rayo nominal ROS 180°) respecto a la perpendicular al piso.
"""
import math

import numpy as np
import pytest

from falconscan_velocity.core import (VelocityEstimator, VelocityParams, Scan, calibrate_mount,
                                      detect_direction, fit_floor, offset_from_down)
from falconscan_velocity.core.calibration import CORREGIDO, FALLA, OK

N_RAYOS = 3240          # S2E con angle_compensate: 1/9°
ANG = np.arange(N_RAYOS) * 2 * math.pi / N_RAYOS


def escena(Hv=3.0, pendiente_deg=0.0, giro_deg=0.0, abajo_deg=180.0, sentido=1, ruido=0.002,
           segmentos=(), seed=0, n=1, offset=0.0):
    """Escaneos (n × rayos) de un sensor a altura vertical Hv sobre un piso con pendiente.

    Marco gravedad: X horizontal (avance), Z arriba; piso Z = X·tan(s). El rayo theta del sensor
    va en la dirección mundo phi = sentido·(theta − abajo) − 90° + giro: con giro = 0 el rayo
    `abajo` apunta a la plomada; `giro` es la inclinación del sensor dentro de su plano.
    `offset` se suma a todos los rangos (error de rango del sensor, lo que la calibración llama b).
    `segmentos`: [(X0, Z0, X1, Z1)] obstáculos (paredes, cajas) en el marco gravedad.
    """
    s = math.radians(pendiente_deg)
    phi = sentido * (ANG - math.radians(abajo_deg)) - math.pi / 2 + math.radians(giro_deg)
    dx, dz = np.cos(phi), np.sin(phi)
    with np.errstate(divide="ignore", invalid="ignore"):
        t = Hv / (dx * math.tan(s) - dz)          # Hv + t·dz = t·dx·tan(s)
    t[~(t > 0)] = np.inf
    for X0, Z0, X1, Z1 in segmentos:
        ex, ez = X1 - X0, Z1 - Z0
        wx, wz = X0, Z0 - Hv
        den = dx * ez - dz * ex
        with np.errstate(divide="ignore", invalid="ignore"):
            tt = (wx * ez - wz * ex) / den
            u = (wx * dz - wz * dx) / den
        tt[~((tt > 1e-6) & (u >= 0) & (u <= 1))] = np.inf
        t = np.minimum(t, tt)
    rng = np.random.default_rng(seed)
    r = t[None, :] + offset + ruido * rng.standard_normal((n, t.size))
    r[~np.isfinite(r) | (r > 12.0)] = np.nan
    return r[0] if n == 1 else r


def normal_esperada_deg(pendiente_deg, giro_deg, abajo_deg=180.0, sentido=1):
    """Ángulo del escaneo de la normal del piso hacia abajo: phi_n = −90° + s."""
    phi_n = -90.0 + pendiente_deg
    return (abajo_deg + sentido * (phi_n + 90.0 - giro_deg)) % 360


ABAJO = math.radians(180.0)     # flecha 0° del S2E = rayo ROS 180°


@pytest.mark.parametrize("pendiente, giro", [(0.0, 0.0), (3.0, 0.0), (0.0, -4.0), (2.5, 1.5), (-4.0, 0.5)])
def test_recupera_rotacion_respecto_al_piso_con_altura_fija(pendiente, giro):
    r = escena(Hv=3.0, pendiente_deg=pendiente, giro_deg=giro, n=10)
    c = calibrate_mount(r, ANG, 3.0, ABAJO)
    assert math.degrees(c.down_angle_rad) == pytest.approx(normal_esperada_deg(pendiente, giro), abs=0.02)
    assert c.height_m == 3.0                                     # la cinta no se toca
    # Sin la pendiente medida, el piso más cerca en perpendicular (H·cos s) se ve como offset. No
    # es un offset puro: el modelo con H fijo lo infla por ~1/cos de los haces oblicuos (4° -> 7.3
    # mm reales salen como ~9.7 mm). Lo mismo le pasa a un error de la cinta.
    real = 3.0 * math.cos(math.radians(pendiente)) - 3.0
    assert c.range_offset_m <= real + 0.002 and c.range_offset_m >= 1.5 * real - 0.002
    assert c.down_angle_sigma_rad < math.radians(0.05)


@pytest.mark.parametrize("b", [0.0, 0.025, -0.06, 0.15])
def test_recupera_el_offset_de_rango(b):
    c = calibrate_mount(escena(Hv=3.0, giro_deg=1.0, offset=b, n=10), ANG, 3.0, ABAJO)
    assert c.range_offset_m == pytest.approx(b, abs=0.0015)
    assert c.seen_height_m == pytest.approx(3.0 + b, abs=0.0015)
    assert math.degrees(c.down_angle_rad) == pytest.approx(180.0 - 1.0, abs=0.02)
    assert c.offset_status == (OK if abs(b) <= 0.03 else CORREGIDO) and not c.failed
    assert c.model_rms_m < 0.003                                  # el modelo (ν, b) explica la curvatura


def test_offset_mayor_al_maximo_es_falla():
    c = calibrate_mount(escena(Hv=3.0, offset=0.22), ANG, 3.0, ABAJO)
    assert c.offset_status == FALLA and c.failed and "revise el montaje" in c.failures[0]


@pytest.mark.parametrize("giro, estado", [(0.3, OK), (-2.0, CORREGIDO), (6.0, FALLA)])
def test_desvio_de_la_flecha(giro, estado):
    c = calibrate_mount(escena(Hv=3.0, giro_deg=giro, n=5), ANG, 3.0, ABAJO)
    # El sensor girado +giro mueve la normal a 180° − giro (ROS); en la marca del sensor es +giro
    assert math.degrees(c.arrow_deviation_rad) == pytest.approx(-giro, abs=0.02)
    assert c.angle_status == estado and c.failed == (estado == FALLA)


def test_marco_del_piso_deja_el_piso_plano_en_la_roi():
    """Con H, b y ν aplicados como en operación, el piso con pendiente queda en z = 0."""
    r = escena(Hv=3.0, pendiente_deg=3.0, giro_deg=-1.0, offset=0.04, ruido=0.0)
    c = calibrate_mount(r, ANG, 3.0, ABAJO)
    p = VelocityParams(mount_height_m=c.height_m, angle_offset_rad=c.angle_offset_rad,
                       range_bias_m=c.range_bias_for(0.0, 1.0))
    h = VelocityEstimator(p).profile_from_scan(Scan(0.0, 0.0, ANG[1], r))
    assert np.nanmax(np.abs(h)) < 0.006
    assert c.roi_floor_dev_m < 0.005 and c.roi_coverage > 0.95
    assert not c.warnings and not c.failed


def test_offset_se_suma_a_la_correccion_de_la_unidad():
    c = calibrate_mount(escena(Hv=3.0, offset=0.02), ANG, 3.0, ABAJO)
    # Rangos de entrada ya corregidos con (r − 0.0027)·0.995753: el residuo b se lleva a ese marco
    assert c.range_bias_for(0.0027, 0.995753) == pytest.approx(0.0027 + c.range_offset_m / 0.995753)


def test_pendiente_conocida_separa_giro_del_sensor_y_da_la_plomada():
    r = escena(Hv=3.0, pendiente_deg=3.0, giro_deg=1.0, n=5)
    c = calibrate_mount(r, ANG, 3.0, ABAJO, floor_slope_rad=math.radians(3.0))
    # Con giro = 0 la plomada sería el rayo 180°; con el sensor girado +1° la plomada es 179°
    assert math.degrees(c.plumb_angle_rad) == pytest.approx(180.0 - 1.0, abs=0.03)
    assert c.perp_height_m == pytest.approx(3.0 * math.cos(math.radians(3.0)))
    assert abs(c.range_offset_m) < 0.002                          # la pendiente ya no se ve como offset
    assert not c.warnings


def test_altura_de_referencia_descarta_pared_mas_cercana_que_el_piso():
    """Pared vertical a 1 m del sensor y casi «abajo» del rayo configurado: la cinta descarta la pared."""
    pared = [(1.0, 0.0, 1.0, 4.0)]
    r = escena(Hv=3.0, segmentos=pared)
    f = fit_floor(r, ANG, 3.0, math.radians(180.0 + 20.0))   # suposición inicial mala (20°)
    assert f is not None and f.height_m == pytest.approx(3.0, abs=0.002)
    # La altura dada manda: con 1 m y la suposición hacia la pared, el ajuste toma la pared
    f = fit_floor(r, ANG, 1.0, math.radians(180.0 + 70.0))
    assert f is not None and math.degrees(f.down_angle_rad) == pytest.approx(270.0, abs=0.1)


def test_cinta_muy_mal_medida_no_encuentra_el_piso():
    with pytest.raises(ValueError, match="No encontré"):
        calibrate_mount(escena(Hv=3.0), ANG, 4.0, ABAJO)


def test_objeto_en_la_roi_y_movimiento_se_avisan():
    caja = [(0.5, 0.0, 0.5, 0.6), (0.5, 0.6, 1.2, 0.6), (1.2, 0.6, 1.2, 0.0)]
    c = calibrate_mount(escena(Hv=3.0, segmentos=caja, n=5), ANG, 3.0, ABAJO)
    assert c.roi_object_x_m is not None and c.roi_object_x_m[0] == pytest.approx(0.5, abs=0.05)
    assert abs(c.range_offset_m) < 0.002                          # la caja no arrastra el ajuste
    pila = escena(Hv=3.0, n=10)
    pila[5:, 1500:1700] -= 0.5                                   # algo pasó por media captura
    c = calibrate_mount(pila, ANG, 3.0, ABAJO)
    assert c.moving_fraction > 0.05 and any("se movió" in w for w in c.warnings)


def test_abanico_quita_el_soporte_sobre_el_sensor():
    soporte = [(-0.5, 3.08, 0.5, 3.08)]                          # placa 8 cm por encima del sensor
    c = calibrate_mount(escena(Hv=3.0, segmentos=soporte), ANG, 3.0, ABAJO)
    assert c.roi_object_x_m is not None                          # sin abanico: «algo sobre el piso»
    c = calibrate_mount(escena(Hv=3.0, segmentos=soporte), ANG, 3.0, ABAJO, fan_angle_rad=math.radians(120))
    assert c.roi_object_x_m is None and not c.warnings
    assert abs(c.range_offset_m) < 0.002


def test_abanico_mas_angosto_que_la_roi_se_avisa():
    c = calibrate_mount(escena(Hv=3.0), ANG, 3.0, ABAJO, fan_angle_rad=math.radians(60))
    assert any("abanico" in w for w in c.warnings)               # llega hasta |x| = 1.73 m < 3 m
    assert c.floor_x_range_m[0] > -1.8 and c.floor_x_range_m[1] < 1.8


def test_rayos_sin_retorno_se_ignoran_no_se_rellenan():
    r = escena(Hv=3.0)
    r[::3] = np.nan
    c = calibrate_mount(r, ANG, 3.0, ABAJO)
    assert abs(c.range_offset_m) < 0.002
    assert np.isnan(r[::3]).all()


@pytest.mark.parametrize("sentido", [1, -1])
def test_sentido_con_objeto_del_lado_de_entrada(sentido):
    caja = [(-1.5, 0.0, -1.5, 0.5), (-1.5, 0.5, -1.0, 0.5), (-1.0, 0.5, -1.0, 0.0)]   # entrada: X < 0
    vacio = escena(Hv=3.0, sentido=sentido, n=3)
    con = escena(Hv=3.0, sentido=sentido, segmentos=caja, n=3)
    c = calibrate_mount(vacio, ANG, 3.0, ABAJO)
    d, xm, n = detect_direction(vacio, con, ANG, c.down_angle_rad)
    assert d == sentido and n > 20
    # Con el sentido detectado, la caja queda en x < 0 del marco mundo
    p = VelocityParams(mount_height_m=c.height_m, angle_direction=d,
                       angle_offset_rad=offset_from_down(c.down_angle_rad, d))
    h = VelocityEstimator(p).profile_from_scan(Scan(0.0, 0.0, ANG[1], con[0]))
    xc = p.grid.centers
    assert np.nanmax(np.where(xc < 0, h, np.nan)) > 0.4 and np.nanmax(np.where(xc > 0, h, np.nan)) < 0.05


def test_sentido_necesita_un_objeto_visible():
    v = escena(Hv=3.0, n=3)
    assert detect_direction(v, v, ANG, ABAJO) is None


def test_offset_nominal_de_la_flecha_al_piso():
    """Flecha 0° = rayo ROS 180° hacia abajo -> +π/2 (valor del yaml y de VelocityParams)."""
    assert offset_from_down(math.pi, 1) == pytest.approx(math.pi / 2)
    assert VelocityParams().angle_offset_rad == pytest.approx(math.pi / 2)
