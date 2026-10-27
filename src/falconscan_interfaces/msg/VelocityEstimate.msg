# Estimación de velocidad longitudinal de la volqueta (LiDAR de velocidad, Falcon Scan).
#
# header.stamp = instante MEDIO del intervalo comparado (mid_stamp). Con aceleración,
# la velocidad media del intervalo corresponde a ese instante, no al último escaneo.

uint8 STATUS_OK=0
uint8 STATUS_NO_OBJECT=1
uint8 STATUS_WARMING_UP=2
uint8 STATUS_BAD_DT=3
uint8 STATUS_LOW_OVERLAP=4
uint8 STATUS_LOW_PEAK=5
uint8 STATUS_AMBIGUOUS=6
uint8 STATUS_AT_SEARCH_LIMIT=7
uint8 STATUS_IMPLAUSIBLE=8

std_msgs/Header header
builtin_interfaces/Time scan_stamp   # instante del escaneo que generó la estimación

uint8 status
string status_text
bool valid                           # status == STATUS_OK

# Velocidad con signo en el marco mundo (+x); no se aplica invert_velocity_sign (es solo para mostrar).
# Para integrar posición o volumen: velocity_mps en header.stamp (mid_stamp); no sumar displacement_m.
float64 velocity_mps                 # velocidad media del intervalo; NaN si no es válida
float64 velocity_filtered_mps        # EMA (tiene retardo; no usar para integrar posición)
float64 displacement_m               # Δs del intervalo
float64 dt_s                         # Δt del intervalo

# Diagnóstico de la correlación
float64 shift_bins
float32 peak_correlation
float32 peak_prominence
uint32 n_overlap

# Extensión del objeto en la ROI [m, eje de avance]; NaN si no hay objeto
float32 object_x_start_m
float32 object_x_end_m

# Paso de camión (PassTracker). Los escaneos de ENTRANDO salen con pass_id 0: use PassEvent.t_start
uint32 pass_id                       # 0 si no hay paso abierto
int8 direction                       # +1 / -1 del paso; 0 si no se conoce
float64 advance_mps                  # direction · velocity_mps (rapidez de avance); NaN si no es válida
