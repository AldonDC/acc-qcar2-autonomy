#!/bin/bash
# Launch SIN mapa (solo nodos base). Para tener MAPA y usar RViz usa: ./run_qcar2_with_map.sh
# Ejecutar DENTRO del contenedor: cd /workspaces/isaac_ros-dev/ros2 && ./run_qcar2_virtual.sh
#
# Contenedor VIRTUAL QCar + mapa (en otra terminal, ANTES de este script):
#   sudo docker run --rm -it --network host quanser/virtual-qcar2
#   Dentro del contenedor:
#   cd /home/qcar2_scripts/python && python3 Base_Scenarios_Python/Setup_Competition_Map.py

set -e
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

# Comprobar si tenemos las librerías Quanser (solo en el contenedor Docker)
if ! dpkg -l quanser-sdk &>/dev/null && [ ! -d /usr/include/quanser ]; then
    echo ""
    echo "  ERROR: Faltan las librerías de Quanser (quanser-sdk)."
    echo "  Este script debe ejecutarse DENTRO del contenedor Docker de desarrollo."
    echo ""
    echo "  Pasos:"
    echo "  1. En esta máquina (fuera del contenedor), ejecuta:"
    echo "     cd $HOME/Documents/ACC_Development/isaac_ros_common/scripts"
    echo "     ./run_dev.sh $HOME/Documents/ACC_Development/Development"
    echo "  2. Cuando estés dentro del contenedor (prompt admin@...), ejecuta:"
    echo "     cd /workspaces/isaac_ros-dev/ros2"
    echo "     ./run_qcar2_virtual.sh"
    echo ""
    exit 1
fi
    
# Si la build se hizo en el host (ruta con Documents/...) y ahora estamos en el contenedor, limpiar
if [ -d build ] && grep -rq "Documents/ACC_Development" build/*/CMakeCache.txt 2>/dev/null; then
    echo ">>> Limpiando build anterior (hecho en el host) para recompilar en el contenedor..."
    rm -rf build install log
fi

echo ">>> 1/3 Compilando nodos ROS2 de QCar2 (colcon build)..."
colcon build

echo ""
echo ">>> 2/3 Cargando entorno (source install/setup.bash)..."
source install/setup.bash

echo ""
echo ">>> 3/3 Lanzando nodos QCar virtual (se reinicia solo si se cae; Ctrl+C para salir)..."
# Salir limpio con Ctrl+C en lugar de reiniciar
trap 'echo ""; echo ">>> Saliendo (Ctrl+C)."; exit 0' INT
while true; do
    ros2 launch qcar2_nodes qcar2_virtual_launch.py || true
    echo ""
    echo ">>> Launch terminó. Reintentando en 3 s..."
    sleep 3
done
