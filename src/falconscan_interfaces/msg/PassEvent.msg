# Inicio o fin de un paso de camión (PassTracker, docs/maquina_estados_paso.md §2).
# Todo en el marco mundo del LiDAR de velocidad (+x); no se aplica invert_velocity_sign.
# Velocidades = rapidez de avance (direction · v), positivas en los dos sentidos; negativa = reversa.
#
# t_start = primer escaneo ocupado (incluye ENTRANDO). t_end = último escaneo ocupado.
# Ambos en el reloj de header.stamp de /scan.

uint8 EVENT_INICIO=0
uint8 EVENT_FIN=1

std_msgs/Header header
uint32 pass_id
uint8 event
string event_text                    # "INICIO" | "FIN"
bool valid                           # FIN sin banderas = candidato a medición válida
int8 direction                       # +1 / -1; 0 si aún no se conoce
builtin_interfaces/Time t_start
builtin_interfaces/Time t_end        # solo en FIN
float64 duration_s                   # NaN en INICIO
string[] flags                       # solo en FIN; vacío = paso sin banderas
uint32 n_gaps                        # veces que pasó por SALIENDO y volvió
uint32 n_scans                       # escaneos OCUPADOS del paso (desde ENTRANDO)
uint32 n_valid                       # de ellos, con velocidad OK
float64 v_median_mps                 # NaN si no hubo velocidades OK
float64 v_min_mps
float64 v_max_mps

# Extensión del objeto [m, eje x del marco mundo] al aparecer y al desaparecer; NaN si no se conoce
float32 start_x_start_m
float32 start_x_end_m
float32 end_x_start_m
float32 end_x_end_m
