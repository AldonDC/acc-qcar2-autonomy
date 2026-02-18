#!/bin/bash
# ================================================================
# GRABAR RUTA — Graba la posición del QCar mientras lo conduces
# ================================================================
# Prerequisitos:
#   T1: QLabs corriendo
#   T2: ros2 launch qcar2_nodes qcar2_cartographer_virtual_launch.py
#   T4: ros2 run qcar2_autonomy simple_teleop_keyboard  ← en otra terminal
#
# Uso: ./record.sh [nombre_ruta]
# Ejemplo: ./record.sh vuelta_rapida
# ================================================================

ROUTE_NAME="${1:-ruta_$(date +%Y%m%d_%H%M%S)}"
OUTPUT_DIR="/workspaces/isaac_ros-dev/ros2/routes"
OUTPUT_FILE="${OUTPUT_DIR}/${ROUTE_NAME}.json"

echo "============================================"
echo "🎥 GRABACIÓN DE RUTA: ${ROUTE_NAME}"
echo "============================================"
echo "Archivo de salida: ${OUTPUT_FILE}"
echo ""
echo "⚠️  Abre OTRA terminal y ejecuta:"
echo "    ros2 run qcar2_autonomy simple_teleop_keyboard"
echo ""
echo "Controles: W=adelante S=atrás A/D=girar X=parar Q=salir"
echo ""
echo "Pulsa Ctrl+C aquí para terminar y guardar el JSON."
echo "============================================"

# Crear directorio de rutas si no existe
mkdir -p "${OUTPUT_DIR}"

# Lanzar solo el recorder
ros2 run qcar2_autonomy route_recorder --ros-args \
  -p output_file:="${OUTPUT_FILE}" \
  -p route_name:="${ROUTE_NAME}" \
  -p min_distance:=0.10

echo ""
echo "✅ Ruta guardada en: ${OUTPUT_FILE}"
echo "Para navegar: ./navigate.sh ${OUTPUT_FILE}"
