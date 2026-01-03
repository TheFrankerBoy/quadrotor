#!/usr/bin/env bash
set -euo pipefail

# ===============================
# CONFIGURACIÓN BASE (PORTABLE)
# ===============================
SESSION="px4_sim"

# Permiten override sin editar el script:
#   CPR_PX4_DIR=/ruta ./scripts/run_sim.sh
#   QGC_APPIMAGE=/ruta/QGC.AppImage ./scripts/run_sim.sh
CPR_PX4_DIR="${CPR_PX4_DIR:-$HOME/CPR_PX4}"
QGC_APPIMAGE="${QGC_APPIMAGE:-$HOME/imagenes_Software/QGroundControl-x86_64.AppImage}"

PX4_DIR="$CPR_PX4_DIR/PX4-Autopilot"
XRCE_DIR="$CPR_PX4_DIR/Micro-XRCE-DDS-Agent/build"
WS_SENSOR="$CPR_PX4_DIR/ws_sensor_combined"
WS_OFFBOARD="$CPR_PX4_DIR/ws_offboard_control"

ROS_SETUP="/opt/ros/humble/setup.bash"

# Comandos principales
PX4_CMD="PX4_GZ_WORLD=baylands make px4_sitl gz_x500_lidar_2d"
XRCE_CMD="MicroXRCEAgent udp4 --port 8888"
LISTENER_CMD="ros2 launch px4_ros_com sensor_combined_listener.launch.py"
OFFBOARD_CMD="ros2 run px4_ros_com offboard_control"

# ===============================
# FLAGS
# ===============================
FORCE_CLEAN=0
DO_CLEAN_ONLY=0

usage() {
  cat <<EOF
Uso: $(basename "$0") [opciones]

Opciones:
  --kill | --cleanup    Cierra TODO (tmux + PX4 + Gazebo + XRCE + QGC) y sale.
  --force               Limpia restos antes de lanzar la simulación.
  -h | --help           Muestra esta ayuda.

Variables de entorno (para no editar el script):
  CPR_PX4_DIR=...        Ruta base donde están PX4-Autopilot, Micro-XRCE..., ws_...
  QGC_APPIMAGE=...       Ruta al AppImage de QGroundControl

Ejemplos:
  ./scripts/run_sim.sh
  ./scripts/run_sim.sh --force
  ./scripts/run_sim.sh --kill
  CPR_PX4_DIR=\$HOME/CPR_PX4 QGC_APPIMAGE=\$HOME/QGroundControl.AppImage ./scripts/run_sim.sh
EOF
}

# Parseo simple de argumentos
while [[ $# -gt 0 ]]; do
  case "$1" in
    --kill|--cleanup) DO_CLEAN_ONLY=1; shift ;;
    --force) FORCE_CLEAN=1; shift ;;
    -h|--help) usage; exit 0 ;;
    *) echo "Opción desconocida: $1"; usage; exit 1 ;;
  esac
done

# ===============================
# HELPERS
# ===============================
have() { command -v "$1" >/dev/null 2>&1; }

cleanup() {
  echo "[cleanup] Cerrando sesión tmux y procesos relacionados..."

  # 1) Matar tmux session si existe
  tmux kill-session -t "$SESSION" 2>/dev/null || true

  # 2) Matar PX4 (SITL)
  pkill -f "/bin/px4" 2>/dev/null || true
  pkill -f "px4.*px4_sitl" 2>/dev/null || true
  pkill -f "px4" 2>/dev/null || true

  # 3) Matar MicroXRCEAgent
  pkill -f "MicroXRCEAgent" 2>/dev/null || true

  # 4) Matar Gazebo (gz / ignition / classic por si acaso)
  pkill -f "gz sim" 2>/dev/null || true
  pkill -f "gz-gui" 2>/dev/null || true
  pkill -f "gzclient" 2>/dev/null || true
  pkill -f "gzserver" 2>/dev/null || true
  pkill -f "ign gazebo" 2>/dev/null || true
  pkill -f "gazebo" 2>/dev/null || true

  # 5) (Opcional) Matar QGC si lo lanzas desde aquí
  pkill -x "QGroundControl" 2>/dev/null || true
  pkill -f "QGroundControl" 2>/dev/null || true

  echo "[cleanup] OK"
}

safe_source_ws() {
  local ws="$1"
  if [[ -f "$ws/install/local_setup.bash" ]]; then
    echo "source \"$ws/install/local_setup.bash\""
  else
    echo "echo \"[WARN] No existe $ws/install/local_setup.bash -> ejecuta: cd $ws && colcon build\""
  fi
}

# ===============================
# CHEQUEOS BÁSICOS
# ===============================
have tmux || { echo "tmux no está instalado"; exit 1; }

# Si solo quieres limpiar y salir
if [[ $DO_CLEAN_ONLY -eq 1 ]]; then
  cleanup
  exit 0
fi

# Limpieza previa si se pidió
if [[ $FORCE_CLEAN -eq 1 ]]; then
  cleanup
fi

# Rutas necesarias
[[ -d "$PX4_DIR" ]] || { echo "No existe $PX4_DIR"; exit 1; }
[[ -d "$XRCE_DIR" ]] || { echo "No existe $XRCE_DIR"; exit 1; }
[[ -d "$WS_SENSOR" ]] || { echo "No existe $WS_SENSOR"; exit 1; }
[[ -d "$WS_OFFBOARD" ]] || { echo "No existe $WS_OFFBOARD"; exit 1; }
[[ -f "$ROS_SETUP" ]] || { echo "No existe $ROS_SETUP"; exit 1; }

# QGC (si no está, no matamos el script; solo avisamos)
if [[ ! -x "$QGC_APPIMAGE" ]]; then
  echo "[WARN] QGC AppImage no encontrado/ejecutable en: $QGC_APPIMAGE"
  echo "       Puedes definir QGC_APPIMAGE=/ruta/a/QGroundControl.AppImage"
fi

# ===============================
# QGROUND CONTROL (primero)
# ===============================
if [[ -x "$QGC_APPIMAGE" ]]; then
  if ! pgrep -x QGroundControl >/dev/null 2>&1; then
    echo "[QGC] Lanzando QGroundControl..."
    nohup "$QGC_APPIMAGE" >/tmp/qgc.log 2>&1 &
  else
    echo "[QGC] Ya está ejecutándose."
  fi
fi

# ===============================
# TMUX
# ===============================
# Si ya existe la sesión, engancha
if tmux has-session -t "$SESSION" 2>/dev/null; then
  tmux attach -t "$SESSION"
  exit 0
fi

tmux new-session -d -s "$SESSION" -n sim

# Layout:
# ┌──────── PX4 ────────┐
# ├──── XRCE ──┬─ LIST ─┤
# └──────── OFFBOARD ───┘
tmux split-window -v -t "$SESSION:0" -p 55
tmux split-window -h -t "$SESSION:0.1" -p 50
tmux split-window -v -t "$SESSION:0.2" -p 50

send() { tmux send-keys -t "$1" "$2" C-m; }

P_PX4="$SESSION:0.0"
P_XRCE="$SESSION:0.1"
P_LIST="$SESSION:0.2"
P_OFFB="$SESSION:0.3"

# ===============================
# COMANDOS (en panes)
# ===============================

send "$P_PX4" "cd \"$PX4_DIR\" && $PX4_CMD"
send "$P_XRCE" "cd \"$XRCE_DIR\" && $XRCE_CMD"

send "$P_LIST" "source \"$ROS_SETUP\""
send "$P_LIST" "cd \"$WS_SENSOR\""
send "$P_LIST" "$(safe_source_ws "$WS_SENSOR")"
send "$P_LIST" "$LISTENER_CMD"

send "$P_OFFB" "source \"$ROS_SETUP\""
send "$P_OFFB" "cd \"$WS_OFFBOARD\""
send "$P_OFFB" "$(safe_source_ws "$WS_OFFBOARD")"
send "$P_OFFB" "$OFFBOARD_CMD"

tmux select-pane -t "$P_PX4"
tmux attach -t "$SESSION"

