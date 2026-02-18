# Launch demo de entrega: ruta fija en cuadrado + seguidor.
# El nodo demo_entrega_node publica la ruta; el follower la sigue.
# En RViz se ve: mapa, Path (línea verde), WaypointMarkers (esferas), TF, LaserScan.
# Uso: T1 run_t1.sh, T2 run_t2.sh, T3 este launch + RViz, T4 waypoint_send_gui (opcional).

from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node


def generate_launch_description():
    demo_node = Node(
        package="qcar2_autonomy",
        executable="demo_entrega_node",
        name="demo_entrega_node",
    )
    waypoint_follower = ExecuteProcess(
        cmd=[
            "ros2", "run", "qcar2_autonomy", "waypoint_follower_controller",
            "--ros-args", "-p", "use_rviz_only:=true",
        ],
        shell=False,
    )
    return LaunchDescription([demo_node, waypoint_follower])
