import sys
from pathlib import Path

# Permite correr `pytest` sin colcon: agrega la raíz del paquete al path.
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
