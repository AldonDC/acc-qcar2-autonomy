#!/bin/bash
# Mismo uso que run_qcar2_virtual.sh pero lanza Cartographer para tener MAPA (y poder usar RViz).
# Terminal 2 SIEMPRE este script: sirve para demo, teleop, waypoints por parámetro, waypoints en RViz, etc.
#
# Contenedor virtual QCar + mapa (en otra terminal, ANTES):
#   sudo docker run --rm -it --network host quanser/virtual-qcar2
#   Dentro: cd /home/qcar2_scripts/python && python3 Base_Scenarios_Python/Setup_Competition_Map.py

set -e
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

if ! dpkg -l quanser-sdk &>/dev/null && [ ! -d /usr/include/quanser ]; then
    echo ""
    echo "  ERROR: Ejecuta este script DENTRO del contenedor Docker de desarrollo."
    echo "  Host: ./isaac_ros_common/scripts/run_dev.sh $HOME/Documents/ACC_Development/Development"
    echo "  Luego: cd /workspaces/isaac_ros-dev/ros2 && ./run_qcar2_with_map.sh"
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
echo ">>> 3/3 Lanzando QCar virtual + Cartographer (mapa para RViz; se reinicia solo si se cae; Ctrl+C para salir)..."
trap 'echo ""; echo ">>> Saliendo (Ctrl+C)."; exit 0' INT
while true; do
    ros2 launch qcar2_nodes qcar2_cartographer_virtual_launch.py || true
    echo ""
    echo ">>> Launch terminó. Reintentando en 3 s..."
    sleep 3
done
