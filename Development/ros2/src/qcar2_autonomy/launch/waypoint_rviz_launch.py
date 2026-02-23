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
    # Para que RViz pueda publicar en /clicked_point, el Fixed Frame "map" debe existir en el TF.
    # Si T2 (Cartographer) no está corriendo, "map" no existe y Publish Point no publica nada.
    # Publicamos world -> map (identidad) para que "map" exista siempre.
    static_map = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="world_to_map",
        arguments=["0", "0", "0", "0", "0", "0", "world", "map"],
    )
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
    use_pose_gt_arg = DeclareLaunchArgument(
        "use_pose_gt_topic",
        default_value="false",
        description="Si true, el marcador usa /qcar_pose_gt (pose desde QLabs) para ir a la par sin desfase."
    )

    # Imagen de fondo: pista en /map_circuit. Si existe circuit_extent_from_map.yaml (generado por
    # sync_circuit_from_map_node leyendo /map), se usan esas medidas para coincidir con QLabs/Cartographer.
    extent_file = os.path.join(pkg, "config", "circuit_extent_from_map.yaml")
    default_circuit_params = [{
        "circuit_image_path": "pista_qcar2.png",
        "use_fixed_extent": True,
        "fixed_extent": [-4.1, 4.1, -1.0, 5.38],
        "fixed_resolution": 8.2 / 1152,
    }]
    circuit_map = Node(
        package="qcar2_autonomy",
        executable="circuit_map_publisher",
        name="circuit_map_publisher",
        parameters=[extent_file] if os.path.isfile(extent_file) else default_circuit_params,
    )

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
    # Marcador QCar: primer clic en RViz (Publish Point) donde está el QCar en QLabs = calibración automática.
    # use_pose_gt_topic:=true para usar /qcar_pose_gt (pose desde QLabs) y reducir desfase.
    robot_marker = Node(
        package="qcar2_autonomy",
        executable="robot_pose_marker_node",
        name="robot_pose_marker_node",
        parameters=[{
            "pose_offset_x": 0.0,
            "pose_offset_y": 0.0,
            "pose_offset_theta": 0.22,
            "enable_click_calibration": True,
            "calibration_file": os.path.join(pkg, "config", "qcar_pose_calibration.yaml"),
            "use_pose_gt_topic": LaunchConfiguration("use_pose_gt_topic"),
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
        use_pose_gt_arg,
        static_map,
        circuit_map,
        waypoint_rviz,
        pure_pursuit,
        robot_marker,
        rviz_node,
    ])
