#!/bin/bash
# Aumenta la memoria swap a 8 GB.
# Ejecutar en el HOST (no dentro del contenedor): sudo ./setup_swap_8gb.sh

set -e
TARGET_GB=8
SWAPFILE="${SWAPFILE:-/swapfile}"

if [[ $(id -u) -ne 0 ]]; then
    echo "Ejecuta con sudo: sudo $0"
    exit 1
fi

echo ">>> Swap actual:"
swapon --show 2>/dev/null || true
free -h | grep -i swap || true

echo ""
echo ">>> Desactivando swap actual..."
swapoff -a 2>/dev/null || true

if [[ -f "$SWAPFILE" ]]; then
    echo ">>> Eliminando archivo de swap anterior ($SWAPFILE)..."
    rm -f "$SWAPFILE"
fi

echo ">>> Creando archivo de swap de ${TARGET_GB} GB en $SWAPFILE..."
fallocate -l "${TARGET_GB}G" "$SWAPFILE" 2>/dev/null || dd if=/dev/zero of="$SWAPFILE" bs=1M count=$((TARGET_GB * 1024)) status=progress
chmod 600 "$SWAPFILE"

echo ">>> Configurando swap..."
mkswap "$SWAPFILE"
swapon "$SWAPFILE"

echo ""
echo ">>> Swap después del cambio:"
swapon --show
free -h | grep -i swap

echo ""
echo ">>> Para que el swap de 8 GB se mantenga tras reiniciar, añade esta línea a /etc/fstab si no existe:"
echo "    $SWAPFILE none swap sw 0 0"
echo "Comprueba con: grep swap /etc/fstab"
if ! grep -q "^${SWAPFILE} " /etc/fstab 2>/dev/null; then
    echo ""
    read -p "¿Añadir $SWAPFILE a /etc/fstab ahora? (s/n) " -n 1 r
    echo
    if [[ "$r" =~ ^[sSyY]$ ]]; then
        echo "$SWAPFILE none swap sw 0 0" >> /etc/fstab
        echo "Añadido a /etc/fstab."
    fi
fi

echo ""
echo ">>> Listo. Swap configurado a ${TARGET_GB} GB."
