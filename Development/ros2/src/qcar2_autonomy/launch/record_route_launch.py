# Launch para la FASE DE GRABACIÓN: route_recorder
#
# Uso:
#   ros2 launch qcar2_autonomy record_route_launch.py
#   (En OTRA terminal: ros2 run qcar2_autonomy simple_teleop_keyboard)
#
# Al pulsar Ctrl+C, se guarda el JSON automáticamente.

import os
import time
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Nombre por defecto: ruta_YYYYMMDD_HHMMSS.json
    timestamp = time.strftime("%Y%m%d_%H%M%S")
    default_output = f"/workspaces/isaac_ros-dev/ros2/routes/ruta_{timestamp}.json"
    default_name = f"ruta_{timestamp}"

    output_arg = DeclareLaunchArgument(
        "output_file",
        default_value=default_output,
        description="Ruta al archivo JSON de salida",
    )
    name_arg = DeclareLaunchArgument(
        "route_name",
        default_value=default_name,
        description="Nombre descriptivo de la ruta",
    )
    dist_arg = DeclareLaunchArgument(
        "min_distance",
        default_value="0.10",
        description="Metros mínimos entre waypoints",
    )

    recorder = Node(
        package="qcar2_autonomy",
        executable="route_recorder",
        name="route_recorder",
        output="screen",
        parameters=[{
            "output_file": LaunchConfiguration("output_file"),
            "route_name": LaunchConfiguration("route_name"),
            "min_distance": LaunchConfiguration("min_distance"),
            "min_angle": 0.15,
            "sample_hz": 20.0,
        }],
    )

    return LaunchDescription([
        output_arg,
        name_arg,
        dist_arg,
        recorder,
    ])
