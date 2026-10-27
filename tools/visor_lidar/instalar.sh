#!/usr/bin/env bash
# Crea el acceso directo del Visor LiDAR en el escritorio y en el menú de aplicaciones.
set -e
DIR="$(cd "$(dirname "$(readlink -f "$0")")" && pwd)"
ESCRITORIO="$(xdg-user-dir DESKTOP 2>/dev/null || echo "$HOME/Desktop")"
NOMBRE=falconscan-visor-lidar.desktop
TMP="$(mktemp)"
cat > "$TMP" <<FIN
[Desktop Entry]
Type=Application
Version=1.0
Name=Visor LiDAR FalconScan
Comment=LiDAR S2E en vivo, prueba de paso, calibración, volumen 3D y rosbags
Exec=$DIR/visor_lidar.sh
Icon=$DIR/icono.svg
Terminal=false
Categories=Science;Engineering;
StartupWMClass=falconscan-visor-lidar
FIN
chmod +x "$DIR/visor_lidar.sh"
install -Dm 644 "$TMP" "$HOME/.local/share/applications/$NOMBRE"
install -Dm 755 "$TMP" "$ESCRITORIO/$NOMBRE"
gio set "$ESCRITORIO/$NOMBRE" metadata::trusted true 2>/dev/null || true
rm -f "$TMP"
echo "Acceso directo creado en $ESCRITORIO/$NOMBRE"
# Dependencias del sistema (apt): PyQt6 + pyqtgraph; la vista 3D de «Volumen 3D» necesita PyOpenGL
for mod in PyQt6:python3-pyqt6 pyqtgraph:python3-pyqtgraph OpenGL:python3-opengl; do
    python3 -c "import ${mod%%:*}" 2>/dev/null || echo "Falta ${mod#*:}: sudo apt install ${mod#*:}"
done
