#!/bin/bash
# Script para enviar la ruta de waypoints al QCar
# Shortcut: ./send.sh

# Cargar entorno de ROS 2 si no está cargado
source /workspaces/isaac_ros-dev/ros2/install/setup.bash 2>/dev/null

echo "Enviando ruta al QCar..."
ros2 service call /send_path std_srvs/srv/Empty
