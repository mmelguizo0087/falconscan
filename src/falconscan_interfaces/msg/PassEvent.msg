# Inicio o fin de un paso de camión (docs/maquina_estados_paso.md). Solo se publica en esos dos
# momentos. En INICIO solo vienen pass_id, t_start y la extensión inicial.
#
# t_start = primer escaneo ocupado (incluye los escaneos de ENTRANDO, publicados con pass_id 0 en
# /velocity_estimate). t_end = último escaneo ocupado. Ambos en el reloj de header.stamp de /scan.

uint8 EVENT_INICIO=0
uint8 EVENT_FIN=1

# Banderas de calidad (máscara de bits; mismos valores que core.PassFlag)
uint16 FLAG_ENTRADA_FUERA_DE_BORDE=1
uint16 FLAG_SALIDA_FUERA_DE_BORDE=2
uint16 FLAG_VELOCIDAD_FUERA_DE_RANGO=4
uint16 FLAG_REVERSA=8
uint16 FLAG_POCOS_VALIDOS=16
uint16 FLAG_TIMEOUT=32
uint16 FLAG_SENSOR_PERDIDO=64
uint16 FLAG_SENTIDO_INCONSISTENTE=128   # el signo de la mediana de v no coincide con el sentido

std_msgs/Header header
uint32 pass_id
uint8 event
string event_text
bool valid                           # FIN sin banderas = candidato a medición válida

builtin_interfaces/Time t_start
builtin_interfaces/Time t_end        # cero en INICIO
float64 duration_s                   # NaN en INICIO

uint16 flags
string[] flag_names

uint32 n_gaps                        # veces que pasó por SALIENDO y volvió
uint32 n_scans                       # escaneos ocupados dentro del paso
uint32 n_valid                       # de ellos, con velocidad OK
# Velocidades y sentido en la referencia de signo del usuario (parámetro invert_velocity_sign)
float64 v_median_mps                 # NaN si no hubo velocidades OK
float64 v_min_mps
float64 v_max_mps
int8 direction                       # +1 sentido positivo, -1 negativo, 0 sin determinar
                                     # (se fija al confirmar la entrada, por el borde de entrada)

# Rapidez de avance v_avance = sentido · v: positiva en los dos sentidos, negativa si retrocede.
# No depende de la referencia de signo. Es la que se compara con 5–10 km/h y la que usa el volumen.
float64 advance_median_mps           # NaN si no hubo velocidades OK
float64 advance_min_mps
float64 advance_max_mps

# Extensión del objeto [m, eje x del marco mundo, sin la referencia de signo] al aparecer y al desaparecer; NaN si no se conoce
float32 start_x_start_m
float32 start_x_end_m
float32 end_x_start_m
float32 end_x_end_m
