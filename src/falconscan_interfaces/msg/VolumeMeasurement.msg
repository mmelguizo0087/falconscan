# Resultado de una medición de volumen (docs/maquina_estados_volumen.md §5).
# Volumen de la ENVOLVENTE del objeto sobre el piso (cabina + volco + carga).

uint8 RESULT_VALID=0
uint8 RESULT_WARNINGS=1
uint8 RESULT_INVALID=2
uint8 RESULT_DISCARDED=3             # objeto no reconocible (más corto que vol_min_length_m)

std_msgs/Header header               # stamp = t_end; frame_id = marco del paso
uint32 measurement_id
uint32 pass_id                       # 0 si no hay paso de velocidad asociado
int8 direction
builtin_interfaces/Time t_start
builtin_interfaces/Time t_end

uint8 result
string result_text
string[] flags

float64 volume_m3
float64 end_uncertainty_m3           # cuantización de frente y cola (1 σ)
string edge_method                   # "velocidad" (bordes del LiDAR de velocidad) o "punto_medio"
float32 length_m
float32 max_width_m
float32 max_height_m
float32 mean_advance_mps
float32 yaw_rad                      # ψ de la trayectoria (informativo)
float32 center_offset_m              # mediana del centro transversal respecto al eje del carril
float32 max_lateral_dev_m            # desviación máx. del centro respecto a la mediana (paralelismo)
float32 path_rms_m
uint32 n_profiles
float32 max_dx_m
float32 nan_fraction
float32 v_interp_fraction
float32 sync_offset_s                # Δt_sync (NaN si no se pudo estimar)
float32 incoherent_fraction
uint32 n_gaps
