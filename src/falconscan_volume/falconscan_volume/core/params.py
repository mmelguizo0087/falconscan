"""Parámetros del reconstructor de volumen (1:1 con la sección volume_node de config/falconscan.yaml).

Los nombres de montaje/calibración son los mismos del LiDAR de velocidad (mount_height_m,
angle_offset_rad, angle_direction, range_bias_m, range_scale, floor_z_m), con los valores del LiDAR de
volumen. Justificación de cada valor: docs/maquina_estados_volumen.md §4.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, fields

from falconscan_velocity.core import MountGeometry, ProfileGrid


@dataclass
class VolumeParams:
    # --- Montaje del LiDAR de volumen: plano de escaneo TRANSVERSAL (y–z) ---
    mount_height_m: float = 5.0        # piso -> centro óptico del sensor            [MEDIR]
    angle_offset_rad: float = math.pi / 2    # flecha 0° del S2E al piso = rayo ROS 180° (D15, D23)  [CALIBRAR]
    angle_direction: int = 1
    y_offset_m: float = 0.0            # posición lateral del sensor respecto al eje del carril
    floor_z_m: float = 0.0
    range_bias_m: float = 0.0          # r_corr = (r − range_bias_m) · range_scale, como el LiDAR de velocidad (D11)
    range_scale: float = 1.0
    # --- Relación entre los dos LiDAR ---
    vol_plane_offset_m: float = 0.0    # centro de rotación del LiDAR de velocidad -> plano de corte, sobre x  [MEDIR]
    vel_plane_y_m: float = 0.0         # posición lateral del plano de velocidad en el marco transversal
    vol_time_offset_s: float = 0.0     # corrección de reloj/latencia de los stamps del LiDAR de volumen
    # --- Validación / filtrado ---
    range_min_m: float = 0.2
    range_max_m: float = 12.0
    median_window: int = 3
    median_max_spread_m: float = 0.15
    # --- ROI lateral y malla ---
    vol_roi_y_min_m: float = -2.0
    vol_roi_y_max_m: float = 2.0
    vol_roi_z_max_m: float = 4.5
    vol_grid_dy_m: float = 0.02
    vol_edge_margin_m: float = 0.10
    # --- Segmentación del perfil ---
    vol_h_threshold_m: float = 0.4
    vol_object_min_width_m: float = 0.5
    vol_object_min_run_m: float = 0.2
    vol_max_nan_run_m: float = 0.20
    vol_floor_drift_tol_m: float = 0.05
    vol_min_data_fraction: float = 0.3      # celdas con dato en la ROI; por debajo el escaneo es SIN DATO (no vacío)
    # --- Ciclo de captura ---
    vol_enter_scans: int = 2
    vol_preroll_scans: int = 4
    vol_empty_scans: int = 5
    vol_exit_gap_m: float = 1.5
    vol_exit_max_s: float = 3.0
    vol_max_capture_s: float = 30.0
    scan_timeout_s: float = 0.5
    # --- Velocidad ---
    vel_timeout_s: float = 0.5
    vol_velocity_margin_s: float = 0.1
    vol_close_wait_s: float = 3.0
    vol_v_max_gap_s: float = 1.0
    vol_max_v_interp_fraction: float = 0.3
    # --- Integración ---
    vol_merge_dx_m: float = 0.02
    vol_max_dx_m: float = 0.6
    vol_edge_from_velocity: bool = True     # ubicar frente y cola con el LiDAR de velocidad
    # --- Chequeos de la medición ---
    vol_min_profiles: int = 10
    vol_min_length_m: float = 3.0           # objeto más corto: no reconocible (DESCARTADO)
    vol_max_length_m: float = 20.0          # más largo: INVALIDA (p. ej. dos camiones pegados)
    vol_max_truncated_fraction: float = 0.05
    vol_max_nan_fraction: float = 0.10
    vol_max_shadow_fraction: float = 0.10   # perfiles con sombra larga sobre el total
    vol_max_center_offset_m: float = 0.5    # |centro transversal mediano| respecto al eje -> DESCENTRADO (anula)
    vol_max_lateral_dev_m: float = 0.3      # desvío del centro respecto a su mediana -> CONDUCCION_NO_PARALELA (anula)
    vol_sync_tol_s: float = 0.15
    vol_max_incoherent_fraction: float = 0.2
    # --- Salidas ---
    vol_cloud_live: bool = True
    vol_export_dir: str = ""

    @classmethod
    def field_names(cls) -> list[str]:
        return [f.name for f in fields(cls)]

    @property
    def geometry(self) -> MountGeometry:
        # El eje horizontal del plano transversal es y; se reutiliza la geometría del
        # LiDAR de velocidad (allí x) con y_offset_m como desplazamiento.
        return MountGeometry(self.mount_height_m, self.angle_offset_rad, int(self.angle_direction), self.y_offset_m)

    @property
    def grid(self) -> ProfileGrid:
        return ProfileGrid(self.vol_roi_y_min_m, self.vol_roi_y_max_m, self.vol_grid_dy_m)
