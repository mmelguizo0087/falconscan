from .lidar_sim import LidarSim, LidarSimConfig, TruckMotion, make_dump_truck

# Parámetros de velocity_node que el yaml fija para el sensor FÍSICO y que con los simuladores
# (lidar_sim y el pórtico 3D) deben ser los del modelo: altura del simulador, rayos en orden de
# medición y rangos exactos (sin la inversión de sllidar_ros2 ni la calibración de la unidad 1).
# Lo usan velocity.launch.py (use_sim:=true) y falconscan_volume/volume_sim.launch.py.
SIM_VELOCITY_OVERRIDES = {
    "mount_height_m": LidarSimConfig.height_m,
    "scan_time_reversed": False,
    "range_bias_m": 0.0,
    "range_scale": 1.0,
}

__all__ = ["LidarSim", "LidarSimConfig", "TruckMotion", "make_dump_truck", "SIM_VELOCITY_OVERRIDES"]
