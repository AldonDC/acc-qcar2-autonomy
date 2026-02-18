# Launch para la FASE DE NAVEGACIÓN: route_player + pure_pursuit + RViz
#
# Uso:
#   ros2 launch qcar2_autonomy navigate_route_launch.py route_file:=/ruta/al/archivo.json
#
# El coche sigue la ruta grabada automáticamente.
# En RViz verás: mapa de Cartographer + ruta (línea azul + esferas) + coche (flecha amarilla)

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg = get_package_share_directory("qcar2_autonomy")
    rviz_config = os.path.join(pkg, "config", "qcar2_navigate.rviz")

    # Argumento para el archivo de ruta
    route_file_arg = DeclareLaunchArgument(
        "route_file",
        default_value="",
        description="Ruta al archivo JSON grabado con route_recorder",
    )
    skip_arg = DeclareLaunchArgument(
        "skip_every",
        default_value="1",
        description="Usar 1 de cada N puntos (1=todos, 5=cada 5)",
    )
    loop_arg = DeclareLaunchArgument(
        "loop",
        default_value="false",
        description="Repetir la ruta en bucle",
    )

    route_player = Node(
        package="qcar2_autonomy",
        executable="route_player",
        name="route_player",
        output="screen",
        parameters=[{
            "route_file": LaunchConfiguration("route_file"),
            "skip_every": LaunchConfiguration("skip_every"),
            "loop": LaunchConfiguration("loop"),
            "auto_start": True,
        }],
    )

    pure_pursuit = Node(
        package="qcar2_autonomy",
        executable="waypoint_follower_pure_pursuit",
        name="waypoint_follower_pure_pursuit",
        output="screen",
    )

    robot_marker = Node(
        package="qcar2_autonomy",
        executable="robot_pose_marker_node",
        name="robot_pose_marker_node",
        parameters=[{
            "pose_offset_x": 0.0,
            "pose_offset_y": 0.0,
            "pose_offset_theta": 0.0,
            "enable_click_calibration": False,
        }],
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config],
    )

    return LaunchDescription([
        route_file_arg,
        skip_arg,
        loop_arg,
        route_player,
        pure_pursuit,
        robot_marker,
        rviz_node,
    ])
