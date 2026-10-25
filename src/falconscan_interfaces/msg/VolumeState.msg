# Estado del reconstructor de volumen (uno por perfil transversal y en el timer).

uint8 ARRANQUE=0
uint8 VACIO=1
uint8 DETECTANDO=2
uint8 CAPTURANDO=3
uint8 HUECO=4
uint8 CERRANDO=5
uint8 INTEGRANDO=6
uint8 OBSTRUIDO=7
uint8 SIN_DATOS=8
uint8 SIN_VELOCIDAD=9

std_msgs/Header header
string mode                          # CALIBRACION / FALLA_CAL / OPERACION
uint8 state
string state_text
bool ready
uint32 measurement_id                # medición en curso o última
uint32 pass_id
uint32 n_profiles
float64 partial_volume_m3            # provisional mientras se captura; NaN si no hay captura
