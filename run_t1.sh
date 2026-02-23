#!/bin/bash
# Terminal 1: contenedor virtual QCar + escenario QLabs.
# Ejecutar desde la raíz del proyecto: ./run_t1.sh
#
# IMPORTANTE: Antes de que el script funcione, debes tener QLabs abierto:
#   1. Abre Quanser Interactive Labs (QLabs).
#   2. Entra en "ACC Self Driving Car Competition" y abre el plano/escenario.
#   3. Luego ejecuta este script.
#
# Uso:
#   ./run_t1.sh          → menú interactivo
#   ./run_t1.sh 1        → lanza directo el escenario 1
#   ./run_t1.sh 2        → lanza directo el escenario 2

set -e
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

SCENARIOS_DIR="Base_Scenarios_Python"

# ── Colores ──
R='\033[0;31m'    # rojo
G='\033[0;32m'    # verde
Y='\033[1;33m'    # amarillo
C='\033[0;36m'    # cyan
B='\033[1;37m'    # blanco bold
D='\033[0;37m'    # dim
N='\033[0m'       # reset

# ── Escenarios ──
declare -A SCENARIOS DESCRIPTIONS ICONS
SCENARIOS[1]="Setup_Competition_Map.py";             DESCRIPTIONS[1]="Mapa vacío (navegación, sin señales)";                  ICONS[1]="🗺️ "
SCENARIOS[2]="Setup_Real_Scenario.py";               DESCRIPTIONS[2]="Mapa completo (semáforos+stops+yields+roundabout)";     ICONS[2]="🏁"
SCENARIOS[3]="Setup_Competition_Map_Interleaved.py";  DESCRIPTIONS[3]="Mapa vacío — Interleaved (mejor FPS)";                ICONS[3]="⚡"
SCENARIOS[4]="Setup_Real_Scenario_Interleaved.py";    DESCRIPTIONS[4]="Mapa completo — Interleaved (mejor FPS)";             ICONS[4]="🚀"

show_menu() {
    echo ""
    echo -e "  ${C}┌─────────────────────────────────────────────────────────────┐${N}"
    echo -e "  ${C}│${N}        ${B}🏎️  ACC Competition — Escenarios QLabs${N}              ${C}│${N}"
    echo -e "  ${C}├─────────────────────────────────────────────────────────────┤${N}"
    echo -e "  ${C}│${N}                                                             ${C}│${N}"
    echo -e "  ${C}│${N}  ${G}1)${N} ${ICONS[1]} ${DESCRIPTIONS[1]}            ${C}│${N}"
    echo -e "  ${C}│${N}  ${G}2)${N} ${ICONS[2]} ${DESCRIPTIONS[2]}   ${C}│${N}"
    echo -e "  ${C}│${N}  ${D}3)${N} ${ICONS[3]} ${DESCRIPTIONS[3]}              ${C}│${N}"
    echo -e "  ${C}│${N}  ${D}4)${N} ${ICONS[4]} ${DESCRIPTIONS[4]}           ${C}│${N}"
    echo -e "  ${C}│${N}                                                             ${C}│${N}"
    echo -e "  ${C}├─────────────────────────────────────────────────────────────┤${N}"
    echo -e "  ${C}│${N}  ${R}5)${N} 🛑 Detener QCar                                       ${C}│${N}"
    echo -e "  ${C}│${N}  ${Y}6)${N} 🔄 Reiniciar QCar                                     ${C}│${N}"
    echo -e "  ${C}└─────────────────────────────────────────────────────────────┘${N}"
    echo ""
}

# ── Si se pasó argumento, usar directo ──
CHOICE="$1"

if [[ -z "$CHOICE" ]]; then
    echo ""
    echo -e "  ${Y}⚠  Antes de continuar:${N} ¿tienes QLabs abierto?"
    echo -e "  ${D}(Quanser Interactive Labs → ACC Self Driving Car Competition → abrir plano)${N}"
    echo ""
    read -p "  Pulsa ENTER cuando esté listo... " dummy

    show_menu
    echo -ne "  ${B}Elige escenario [1-6]:${N} "
    read CHOICE
fi

# ── Ejecutar ──
case "$CHOICE" in
    1|2|3|4)
        SCRIPT="${SCENARIOS[$CHOICE]}"
        echo ""
        echo -e "  ${G}▶${N} ${B}${DESCRIPTIONS[$CHOICE]}${N}"
        echo -e "  ${D}Script: ${SCENARIOS_DIR}/${SCRIPT}${N}"
        echo ""
        sudo docker run --rm -it --network host quanser/virtual-qcar2 \
          bash -c "cd /home/qcar2_scripts/python && python3 ${SCENARIOS_DIR}/${SCRIPT}; echo ''; echo '>>> Escenario listo. Contenedor activo. Ctrl+C para salir.'; exec bash"
        ;;
    5)
        echo ""
        echo -e "  ${R}■${N} Deteniendo QCar..."
        sudo docker run --rm -it --network host quanser/virtual-qcar2 \
          bash -c "cd /home/qcar2_scripts/python && python3 qcar2_stop.py"
        echo -e "  ${G}✔${N} QCar detenido."
        ;;
    6)
        echo ""
        echo -e "  ${Y}↻${N} Reiniciando QCar..."
        sudo docker run --rm -it --network host quanser/virtual-qcar2 \
          bash -c "cd /home/qcar2_scripts/python && python3 qcar2_restart.py; echo ''; echo '>>> QCar reiniciado. Ctrl+C para salir.'; exec bash"
        ;;
    *)
        echo ""
        echo -e "  ${R}✗${N} Opción inválida: '${CHOICE}'. Usa 1-6."
        exit 1
        ;;
esac
