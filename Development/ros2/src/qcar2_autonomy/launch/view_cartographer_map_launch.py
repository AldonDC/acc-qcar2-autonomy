# Solo RViz para ver el mapa que Cartographer genera con el LiDAR.
# Útil para: T1 (QLabs) + T2 (Cartographer) + este launch + teleop; conduces y ves cómo se construye /map.
#
# Uso:
#   T1: ./run_t1.sh
#   T2: ./run_t2.sh
#   T3 (contenedor): ros2 launch qcar2_autonomy view_cartographer_map_launch.py
#   T4 (contenedor): ros2 run qcar2_autonomy simple_teleop_keyboard
# Conduce con W/S/A/D y mira en RViz el mapa y el escaneo LiDAR.

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory("qcar2_autonomy")
    rviz_config = os.path.join(pkg, "config", "qcar2_navigate.rviz")

    # Frame "map" lo publica Cartographer (T2). Si T2 no está aún, publicamos world->map para que RViz no se queje.
    static_map = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="world_to_map",
        arguments=["0", "0", "0", "0", "0", "0", "world", "map"],
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
        static_map,
        robot_marker,
        rviz_node,
    ])
