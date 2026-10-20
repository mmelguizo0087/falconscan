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
Comment=Lecturas en vivo del LiDAR S2E, análisis de pared y grabación de rosbags
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
