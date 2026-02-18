#!/bin/bash
# Script para limpiar los waypoints de RViz
# Shortcut: ./clear.sh

# Cargar entorno de ROS 2 si no está cargado
source /workspaces/isaac_ros-dev/ros2/install/setup.bash 2>/dev/null

echo "Limpiando waypoints..."
ros2 service call /clear_waypoints std_srvs/srv/Empty
