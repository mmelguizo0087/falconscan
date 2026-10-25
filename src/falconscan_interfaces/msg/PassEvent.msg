# Inicio o fin de un paso de camión (PassTracker, docs/maquina_estados_paso.md §2).
# Velocidades = rapidez de avance (direction · v), positivas en los dos sentidos.

uint8 EVENT_INICIO=0
uint8 EVENT_FIN=1

std_msgs/Header header
uint32 pass_id
uint8 event
int8 direction                       # +1 / -1; 0 si aún no se conoce
builtin_interfaces/Time t_start
builtin_interfaces/Time t_end        # solo en FIN
float64 duration_s
string[] flags                       # solo en FIN; vacío = paso sin banderas
uint32 n_gaps
uint32 n_scans
uint32 n_valid
float64 v_median_mps
float64 v_min_mps
float64 v_max_mps
