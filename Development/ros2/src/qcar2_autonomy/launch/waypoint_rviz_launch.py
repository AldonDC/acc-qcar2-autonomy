# Waypoints en RViz: fondo = imagen de la pista (/map_circuit) con dimensiones del mapa.
# Más robusto que la GUI Python. Requiere que /map esté publicado (Cartographer).
#
# Uso:
#   1. Arranca este launch (T1+T2 ya con simulación y Cartographer).
#   2. Abre RViz con el config que usa /map_circuit (o se lanza RViz aquí).
#   3. En RViz: herramienta "Publish Point" → clic en el mapa para añadir waypoints.
#   4. Enviar ruta al coche: ros2 service call /waypoint_rviz_node/send_path std_srvs/srv/Empty
#   5. Borrar waypoints: ros2 service call /waypoint_rviz_node/clear_waypoints std_srvs/srv/Empty

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory("qcar2_autonomy")
    rviz_config = os.path.join(pkg, "config", "qcar2_waypoints_circuit.rviz")

    # Argumento para cargar JSON al inicio (amarillos)
    load_file_arg = DeclareLaunchArgument(
        "load_file",
        default_value="",
        description="Archivo JSON a cargar en el editor de waypoints (AMARILLOS)"
    )

    # Argumento para cargar GUÍA de referencia (verdes)
    reference_file_arg = DeclareLaunchArgument(
        "reference_file",
        default_value="",
        description="Archivo JSON de guía/referencia (VERDES)"
    )

    # Eliminamos el circuit_map_pub para quitar la imagen de fondo
    # y centrarnos solo en los waypoints y el SLAM.

    waypoint_rviz = Node(
        package="qcar2_autonomy",
        executable="waypoint_rviz_node",
        name="waypoint_rviz_node",
        parameters=[{
            "load_file": LaunchConfiguration("load_file"),
            "reference_file": LaunchConfiguration("reference_file")
        }]
    )
    pure_pursuit = Node(
        package="qcar2_autonomy",
        executable="waypoint_follower_pure_pursuit",
        name="waypoint_follower_pure_pursuit",
    )
    robot_marker = Node(
        package="qcar2_autonomy",
        executable="robot_pose_marker_node",
        name="robot_pose_marker_node",
        parameters=[{
            "pose_offset_x": 0.0,
            "pose_offset_y": 0.0,
            "pose_offset_theta": 0.22,
            "calibration_file": os.path.join(pkg, "config", "qcar_pose_calibration.yaml")
        }],
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config],
    )

    return LaunchDescription([
        load_file_arg,
        reference_file_arg,
        waypoint_rviz,
        pure_pursuit,
        robot_marker,
        rviz_node,
    ])
