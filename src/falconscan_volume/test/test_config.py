"""config/falconscan.yaml (sección volume_node) debe tener exactamente los parámetros de
VolumeParams, con el tipo correcto. El archivo es compartido con velocity_node."""
from pathlib import Path

import pytest

from falconscan_volume.core import VolumeParams

yaml = pytest.importorskip("yaml")
CFG = Path(__file__).resolve().parents[2] / "falconscan_velocity" / "config" / "falconscan.yaml"


def test_yaml_matches_params():
    params = yaml.safe_load(CFG.read_text())["volume_node"]["ros__parameters"]
    params.pop("pass_frame_id")
    assert set(params) == set(VolumeParams.field_names())
    defaults = VolumeParams()
    for k, v in params.items():
        assert type(v) is type(getattr(defaults, k)), f"{k}: yaml={type(v).__name__}"
        assert v == pytest.approx(getattr(defaults, k)) if isinstance(v, float) else v == getattr(defaults, k), k


def test_shared_mount_names():
    """Los dos LiDAR usan los mismos nombres de montaje/calibración."""
    cfg = yaml.safe_load(CFG.read_text())
    vel, vol = cfg["velocity_node"]["ros__parameters"], cfg["volume_node"]["ros__parameters"]
    for k in ("mount_height_m", "angle_offset_rad", "angle_direction", "range_bias_m", "floor_z_m"):
        assert k in vel and k in vol
