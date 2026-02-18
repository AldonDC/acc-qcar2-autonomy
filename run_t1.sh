#!/bin/bash
# Terminal 1: contenedor virtual QCar + mapa. Un solo comando.
# Ejecutar desde la raíz del proyecto: ./run_t1.sh
#
# IMPORTANTE: Antes de que el script del mapa funcione, debes tener QLabs abierto:
#   1. Abre Quanser Interactive Labs (QLabs).
#   2. Entra en "ACC Self Driving Car Competition" y abre el plano/escenario.
#   3. Luego ejecuta este script (o pulsa Enter si el script te lo pide).

set -e
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

echo ""
echo "  Antes de continuar: ¿tienes QLabs abierto con el escenario ACC cargado?"
echo "  (Quanser Interactive Labs → ACC Self Driving Car Competition → abrir plano)"
echo ""
read -p "  Cuando esté listo, pulsa ENTER para arrancar el contenedor... " dummy
echo ""

echo ">>> Arrancando contenedor virtual QCar y script del mapa..."
sudo docker run --rm -it --network host quanser/virtual-qcar2 \
  bash -c "cd /home/qcar2_scripts/python && python3 Base_Scenarios_Python/Setup_Competition_Map.py; echo ''; echo '>>> Mapa listo. Contenedor activo. Ctrl+C para salir.'; exec bash"
