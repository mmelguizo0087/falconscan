"""Pruebas unitarias de las etapas del algoritmo (sin ROS)."""
import math

import numpy as np
import pytest

from falconscan_velocity.core import (MountGeometry, ProfileGrid, Scan, VelocityEstimator, VelocityParams,
                                      fan_mask, find_shift, height_profile, median_filter, polar_to_xz,
                                      validate_ranges)


# ---------------------------------------------------------------- validación
def test_validate_marks_invalid_as_nan_never_zero():
    r = validate_ranges([1.0, 0.0, np.inf, np.nan, 0.1, 50.0, 2.0], r_min=0.2, r_max=12.0)
    assert np.isfinite(r).tolist() == [True, False, False, False, False, False, True]
    assert not np.any(r == 0.0)


# ---------------------------------------------------------------- filtrado
def test_median_removes_isolated_spike_and_keeps_invalid():
    r = np.array([2.0, 2.0, 9.0, 2.0, 2.0, np.nan, 2.0, 2.0])
    f = median_filter(r, 3)
    assert f[2] == pytest.approx(2.0)
    assert np.isnan(f[5])                       # un inválido sigue inválido


def test_median_invalidates_when_window_mostly_invalid():
    r = np.array([np.nan, 3.0, np.nan, 2.0, 2.0])
    f = median_filter(r, 3, min_valid=2)
    assert np.isnan(f[1])


def test_median_does_not_average_outlier_when_neighbor_missing():
    r = np.array([2.0, 2.0, np.nan, 7.0, 2.0, 2.0])
    f = median_filter(r, 3)
    assert np.isnan(f[3])                        # [nan, 7, 2] -> discrepan -> inválido, no 4.5
    assert f[4] == pytest.approx(2.0)


def test_median_preserves_step_edge():
    r = np.r_[np.full(10, 2.0), np.full(10, 4.0)]
    assert np.array_equal(median_filter(r, 3), r)


# ---------------------------------------------------------------- geometría
def test_polar_convention_nadir_and_forward():
    g = MountGeometry(height_m=5.0, angle_offset_rad=-math.pi / 2)
    x, z = polar_to_xz(np.array([5.0, 2.0]), np.array([0.0, math.pi / 4]), g)
    assert x[0] == pytest.approx(0.0, abs=1e-12) and z[0] == pytest.approx(0.0, abs=1e-12)
    # theta positivo -> hacia +x
    assert x[1] > 0 and z[1] == pytest.approx(5.0 - 2.0 * math.cos(math.pi / 4))


def test_fan_mask_is_measured_from_the_nadir_in_the_world_frame():
    g = MountGeometry(height_m=5.0, angle_offset_rad=-math.pi / 2)        # theta = 0 al nadir
    th = np.radians([0.0, 30.0, -30.0, 60.0, -60.0, 180.0])
    assert fan_mask(th, g, math.radians(90)).tolist() == [True, True, True, False, False, False]
    # centro corrido 30° hacia +x: acepta de 0° a 60°
    assert fan_mask(th, g, math.radians(90), math.radians(30)).tolist() == [True, True, False, True, False, False]
    # sentido invertido: theta negativo va hacia +x
    gi = MountGeometry(height_m=5.0, angle_offset_rad=-math.pi / 2, angle_direction=-1)
    assert fan_mask(th, gi, math.radians(90), math.radians(30)).tolist() == [True, False, True, False, True, False]
    assert fan_mask(th, g, 2 * math.pi).all()                           # giro completo: sin filtro


def test_estimator_only_uses_rays_inside_the_fan():
    th = np.arange(3240) * 2 * math.pi / 3240
    r = np.full(th.size, 1.0)                                            # todo a 1 m, también arriba
    scan = Scan(0.0, 0.0, 2 * math.pi / 3240, r)
    base = dict(mount_height_m=2.0, angle_offset_rad=math.pi / 2, median_window=1)   # ROS 180° al nadir
    f_todo = VelocityEstimator(VelocityParams(**base))._frame_from_scan(scan)
    f_fan = VelocityEstimator(VelocityParams(**base, fan_angle_rad=math.radians(60)))._frame_from_scan(scan)
    assert f_todo.x.size == th.size
    assert f_fan.x.size == pytest.approx(th.size / 6, abs=2)
    assert np.all(np.abs(f_fan.x) <= math.sin(math.radians(30)) + 1e-9) and np.all(f_fan.z < 2.0)


def test_height_profile_takes_max_and_leaves_empty_as_nan():
    grid = ProfileGrid(0.0, 1.0, 0.25)
    h, counts = height_profile(np.array([0.1, 0.2, 0.6]), np.array([1.0, 2.0, 0.5]), grid)
    assert h[0] == pytest.approx(2.0)
    assert np.isnan(h[1]) and counts[1] == 0     # sin puntos != piso
    assert h[2] == pytest.approx(0.5)


# ---------------------------------------------------------------- correlación
def _profile(n=300, seed=0):
    rng = np.random.default_rng(seed)
    return np.convolve(rng.standard_normal(n + 20), np.ones(9) / 9, mode="same")[10:-10]


@pytest.mark.parametrize("shift", [-3, 0, 7, 15])
def test_find_shift_integer(shift):
    a = _profile()
    b = np.roll(a, shift)
    res = find_shift(a, b, -10, 20, min_overlap=50, min_std=1e-3)
    assert res.k_int == shift
    assert res.peak > 0.99


def test_find_shift_subpixel():
    x = np.arange(400) * 0.02
    f = lambda s: np.sin(2.1 * (x - s)) + 0.6 * np.sin(5.3 * (x - s)) + 0.3 * np.cos(9.7 * (x - s))
    true_bins = 4.37
    res = find_shift(f(0.0), f(true_bins * 0.02), -2, 12, min_overlap=50, min_std=1e-3)
    assert res.shift_bins == pytest.approx(true_bins, abs=0.05)


def test_find_shift_ignores_nan_regions():
    a = _profile()
    b = np.roll(a, 5)
    a[40:90] = np.nan
    b[150:170] = np.nan
    res = find_shift(a, b, -10, 20, min_overlap=50, min_std=1e-3)
    assert res.k_int == 5


def test_periodic_profile_is_flagged_ambiguous():
    j = np.arange(300)
    a = np.sin(2 * np.pi * j / 10)                # periodo de 10 celdas
    res = find_shift(a, np.roll(a, 3), -15, 15, min_overlap=50, min_std=1e-3)
    assert res.prominence < 0.05                 # hay un rival casi igual a ±10 celdas


def test_flat_profile_gives_no_result():
    a = np.full(200, 2.0)
    res = find_shift(a, a, -5, 5, min_overlap=50, min_std=0.05)
    assert not res.found
