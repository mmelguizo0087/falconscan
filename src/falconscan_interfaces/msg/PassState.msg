# Estado de la máquina de operación del LiDAR de velocidad (uno por escaneo).

uint8 ARRANQUE=0
uint8 VACIO=1
uint8 ENTRANDO=2
uint8 EN_PASO=3
uint8 SALIENDO=4
uint8 OBSTRUIDO=5
uint8 SIN_DATOS=6

std_msgs/Header header
uint8 state
string state_text
uint32 pass_id                       # 0 si no hay paso abierto
int8 direction
bool ready
