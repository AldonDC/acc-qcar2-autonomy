# Launch del dashboard: pista + QCar + ruta + gráficas.
# Requiere que estén corriendo: TF map->base_link (p. ej. qcar_pose_from_qlabs_node),
# y opcionalmente /waypoints_path y /cmd_vel_nav (teleop o controlador).
#
# Por defecto usa vista 2D directa desde QLabs (mismo template que la ventana del simulador).
# Para forzar imagen PNG: use_qlabs_view:=false

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("use_qlabs_view", default_value="true", description="Vista 2D directa desde QLabs (true) o PNG (false)"),
        Node(
            package="qcar2_autonomy",
            executable="qcar_dashboard_gui",
            name="qcar_dashboard_gui",
            parameters=[{"use_qlabs_view": LaunchConfiguration("use_qlabs_view", default="true")}],
        ),
    ])
