#!/bin/bash
# Un solo comando: abre el contenedor y lanza los nodos QCar2 virtual.
# Uso: ./run_qcar2.sh

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
DEV_DIR="$SCRIPT_DIR/Development"
RUN_DEV="$SCRIPT_DIR/isaac_ros_common/scripts/run_dev.sh"

if [[ ! -d "$DEV_DIR" ]] || [[ ! -x "$RUN_DEV" ]]; then
    echo "Error: no se encuentra $RUN_DEV o $DEV_DIR"
    exit 1
fi

"$RUN_DEV" "$DEV_DIR" -- bash -c "cd /workspaces/isaac_ros-dev/ros2 && ./run_qcar2_virtual.sh"
