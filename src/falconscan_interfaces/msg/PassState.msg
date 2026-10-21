# Estado de la máquina de paso (docs/maquina_estados_paso.md). Se publica en cada escaneo y,
# sin escaneos, cada vez que el estado cambia (p. ej. a SIN_DATOS).
#
# header.stamp = stamp del último escaneo procesado (o reloj del nodo si el cambio vino del timer).

uint8 STATE_ARRANQUE=0     # inicio o retorno de falla: un camión a medio pasar no se mide
uint8 STATE_VACIO=1        # ROI libre, esperando vehículo
uint8 STATE_ENTRANDO=2     # ocupación detectada, sin confirmar
uint8 STATE_EN_PASO=3      # paso confirmado; se mide
uint8 STATE_SALIENDO=4     # ROI aparentemente libre; confirmando salida
uint8 STATE_OBSTRUIDO=5    # ocupación demasiado larga
uint8 STATE_SIN_DATOS=6    # no llegan escaneos

std_msgs/Header header
uint8 state
string state_text
uint32 pass_id             # paso abierto (EN_PASO o SALIENDO); 0 si no hay
bool pass_open
bool ready                 # para la HMI: VACIO, ENTRANDO, EN_PASO o SALIENDO
