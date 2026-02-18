#!/bin/bash
# Terminal 2: contenedor de desarrollo + QCar con mapa (build, launch). Un solo comando.
# Ejecutar desde la raíz del proyecto: ./run_t2.sh

set -e
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
DEV_DIR="$SCRIPT_DIR/Development"
RUN_DEV="$SCRIPT_DIR/isaac_ros_common/scripts/run_dev.sh"

if [[ ! -d "$DEV_DIR" ]] || [[ ! -f "$RUN_DEV" ]]; then
    echo "Error: no se encuentra $RUN_DEV o $DEV_DIR"
    exit 1
fi

"$RUN_DEV" "$DEV_DIR" -- bash -c "cd /workspaces/isaac_ros-dev/ros2 && ./run_qcar2_with_map.sh"
