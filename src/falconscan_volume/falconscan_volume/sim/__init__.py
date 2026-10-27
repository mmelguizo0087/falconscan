from .portico_sim import (Box3D, DumpTruck3D, Motion3D, PorticoSim, PorticoSimConfig, lane_change,
                          stop_and_go)

# Parámetros de volume_node que el yaml fija para el sensor FÍSICO y que con el simulador del
# pórtico deben ser los del modelo (como SIM_VELOCITY_OVERRIDES): su altura y rangos exactos.
# Lo usan volume_sim.launch.py y el visor.
SIM_VOLUME_OVERRIDES = {
    "mount_height_m": PorticoSimConfig.vol_height,
    "range_bias_m": 0.0,
    "range_scale": 1.0,
}
from .scenario import ScenarioResult, run_scenario

__all__ = ["SIM_VOLUME_OVERRIDES", "Box3D", "DumpTruck3D", "Motion3D", "PorticoSim", "PorticoSimConfig", "lane_change", "stop_and_go",
           "ScenarioResult", "run_scenario"]
