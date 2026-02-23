#!/bin/bash
# Terminal 2 con Web Dashboard: contenedor + QCar + Cartographer + Pure Pursuit + Dashboard Web.
# Abre http://localhost:8085 en el navegador para ver cámaras y poner waypoints.
#
# Uso:
#   Terminal 1:  ./run_t1.sh      (escenario QLabs)
#   Terminal 2:  ./run_t2_web.sh  (este script)
#   Navegador:   http://localhost:8085

set -e
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
DEV_DIR="$SCRIPT_DIR/Development"
RUN_DEV="$SCRIPT_DIR/isaac_ros_common/scripts/run_dev.sh"

if [[ ! -d "$DEV_DIR" ]] || [[ ! -f "$RUN_DEV" ]]; then
    echo "Error: no se encuentra $RUN_DEV o $DEV_DIR"
    exit 1
fi

"$RUN_DEV" "$DEV_DIR" -- bash -c "cd /workspaces/isaac_ros-dev/ros2 && ./run_qcar2_with_web_dashboard.sh"
