#!/bin/bash
# Igual que run_qcar2_with_map_and_gui.sh pero lanza el Web Dashboard en vez del GUI matplotlib.
# Abre http://localhost:8085 para controlar el carro.

set -e
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

if ! dpkg -l quanser-sdk &>/dev/null && [ ! -d /usr/include/quanser ]; then
    echo ""
    echo "  ERROR: Ejecuta este script DENTRO del contenedor Docker de desarrollo."
    echo "  Host: ./run_t2_web.sh  (desde la raíz del proyecto)"
    echo ""
    exit 1
fi

if [ -d build ] && grep -rq "Documents/ACC_Development" build/*/CMakeCache.txt 2>/dev/null; then
    echo ">>> Limpiando build anterior (hecho en el host)..."
    rm -rf build install log
fi

echo ">>> 1/3 Compilando (colcon build)..."
colcon build

echo ""
echo ">>> 2/3 Cargando entornos (Cartographer + ros2)..."
if [ -f /workspace/cartographer_ws/install/setup.bash ]; then
    source /workspace/cartographer_ws/install/setup.bash
fi
source install/setup.bash

echo ""
echo ">>> 3/3 Lanzando QCar + Cartographer + Pure Pursuit + Web Dashboard..."
echo ">>>     Abre en el navegador: http://localhost:8085"
echo ""
trap 'echo ""; echo ">>> Saliendo (Ctrl+C)."; exit 0' INT
while true; do
    ros2 launch qcar2_autonomy waypoint_web_launch.py || true
    echo ""
    echo ">>> Launch terminó. Reintentando en 3 s..."
    sleep 3
done
