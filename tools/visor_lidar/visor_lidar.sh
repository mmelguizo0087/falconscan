#!/usr/bin/env bash
# Lanzador del Visor LiDAR: carga el entorno ROS 2 (sistema + driver + falconscan) y abre la app.
DIR="$(cd "$(dirname "$(readlink -f "$0")")" && pwd)"
REPO="$(cd "$DIR/../.." && pwd)"
set +u
source /opt/ros/lyrical/setup.bash
[ -f "$REPO/../lidar_ws/install/setup.bash" ] && source "$REPO/../lidar_ws/install/setup.bash"
[ -f "$REPO/install/setup.bash" ] && source "$REPO/install/setup.bash"
LOG="${XDG_STATE_HOME:-$HOME/.local/state}/falconscan"
mkdir -p "$LOG"
exec python3 "$DIR/visor_lidar.py" "$@" 2>>"$LOG/visor_lidar.log"
