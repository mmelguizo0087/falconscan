import sys
from pathlib import Path

# Permite correr `pytest` sin colcon: agrega la raíz del paquete y la de falconscan_velocity
# (núcleo compartido) al path.
root = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(root))
sys.path.insert(0, str(root.parent / "falconscan_velocity"))
