#!/bin/bash
# ================================================================
# NAVEGAR RUTA — El QCar sigue una ruta grabada automáticamente
# ================================================================
# Prerequisitos:
#   T1: QLabs corriendo
#   T2: ros2 launch qcar2_nodes qcar2_cartographer_virtual_launch.py
#
# Uso: ./navigate.sh ruta.json [skip] [loop]
# Ejemplos:
#   ./navigate.sh routes/mi_ruta.json           → Todos los puntos, sin loop
#   ./navigate.sh routes/mi_ruta.json 3         → Cada 3er punto
#   ./navigate.sh routes/mi_ruta.json 1 true    → Todos los puntos, en loop
# ================================================================

ROUTE_FILE="${1}"
SKIP="${2:-1}"
LOOP="${3:-false}"

if [ -z "$ROUTE_FILE" ]; then
    echo "❌ Uso: ./navigate.sh <archivo_ruta.json> [skip_every] [loop]"
    echo ""
    echo "Rutas disponibles:"
    ls -la routes/*.json 2>/dev/null || echo "  (ninguna — primero graba con ./record.sh)"
    exit 1
fi

if [ ! -f "$ROUTE_FILE" ]; then
    echo "❌ Archivo no encontrado: $ROUTE_FILE"
    echo ""
    echo "Rutas disponibles:"
    ls -la routes/*.json 2>/dev/null || echo "  (ninguna — primero graba con ./record.sh)"
    exit 1
fi

# Mostrar info de la ruta
echo "============================================"
echo "🏎️  NAVEGACIÓN AUTÓNOMA"
echo "============================================"
echo "Ruta:     ${ROUTE_FILE}"
echo "Skip:     ${SKIP} (1=todos los puntos)"
echo "Loop:     ${LOOP}"
echo ""
python3 -c "
import json
with open('${ROUTE_FILE}') as f:
    d = json.load(f)
print(f'   Nombre:    {d.get(\"route_name\", \"?\")}')
print(f'   Puntos:    {d.get(\"total_points\", \"?\")}')
print(f'   Distancia: {d.get(\"total_distance_m\", \"?\")}m')
print(f'   Grabada:   {d.get(\"recorded_at\", \"?\")}')
" 2>/dev/null
echo ""
echo "Ctrl+C para detener."
echo "============================================"

LIBGL_ALWAYS_SOFTWARE=1 ros2 launch qcar2_autonomy navigate_route_launch.py \
    route_file:="${ROUTE_FILE}" \
    skip_every:="${SKIP}" \
    loop:="${LOOP}"
