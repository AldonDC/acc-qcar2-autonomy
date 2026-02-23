# Launch del Web Dashboard: dashboard profesional en navegador web.
# Arranca el nodo QCar Web Dashboard que abre un servidor HTTP
# en http://localhost:8085 con streaming MJPEG de cámaras,
# vista overhead de QLabs, telemetría, y manejo de waypoints.
#
# Requiere que estén corriendo los nodos del QCar (csi, rgbd, pose).
# Uso:
#   ros2 launch qcar2_autonomy qcar_web_dashboard_launch.py
#   Luego abrir http://localhost:8085 en el navegador.

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "port", default_value="8085",
            description="HTTP port for the web dashboard"),
        DeclareLaunchArgument(
            "use_qlabs_view", default_value="true",
            description="Connect to QLabs for overhead live view"),
        Node(
            package="qcar2_autonomy",
            executable="qcar_web_dashboard",
            name="qcar_web_dashboard",
            parameters=[{
                "port": LaunchConfiguration("port"),
                "use_qlabs_view": LaunchConfiguration("use_qlabs_view"),
            }],
            output="screen",
        ),
    ])
