# Estado de la máquina de operación del LiDAR de velocidad (uno por escaneo y, sin escaneos,
# cada vez que cambia el estado, p. ej. a SIN_DATOS).

uint8 ARRANQUE=0     # inicio o retorno de falla: un camión a medio pasar no se mide
uint8 VACIO=1        # ROI libre, esperando vehículo
uint8 ENTRANDO=2     # ocupación detectada, sin confirmar
uint8 EN_PASO=3      # paso confirmado; se mide
uint8 SALIENDO=4     # ROI aparentemente libre; confirmando salida
uint8 OBSTRUIDO=5    # ocupación demasiado larga
uint8 SIN_DATOS=6    # no llegan escaneos

std_msgs/Header header
uint8 state
string state_text
uint32 pass_id                       # 0 si no hay paso abierto
bool pass_open                       # EN_PASO o SALIENDO
int8 direction                       # marco mundo; 0 si no hay paso o aún no se conoce
bool ready                           # para la HMI: VACIO, ENTRANDO, EN_PASO o SALIENDO
