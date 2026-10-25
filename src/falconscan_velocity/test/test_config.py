"""config/falconscan.yaml (sección velocity_node) debe tener exactamente los parámetros de VelocityParams, con el tipo correcto."""
from pathlib import Path

import pytest

from falconscan_velocity.core import VelocityParams

yaml = pytest.importorskip("yaml")
CFG = Path(__file__).resolve().parents[1] / "config" / "falconscan.yaml"


def test_yaml_matches_params():
    params = yaml.safe_load(CFG.read_text())["velocity_node"]["ros__parameters"]
    params.pop("publish_profile")
    assert set(params) == set(VelocityParams.field_names())
    defaults = VelocityParams()
    for k, v in params.items():
        # ROS 2 es estricto con los tipos: un float escrito como "5" fallaría al cargar
        assert type(v) is type(getattr(defaults, k)), f"{k}: yaml={type(v).__name__}"
