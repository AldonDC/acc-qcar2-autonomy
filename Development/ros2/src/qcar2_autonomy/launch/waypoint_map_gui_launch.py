# Launch GUI del mapa + seguidor Pure Pursuit (estilo smart_mobility_2025).
# Así el coche se mueve al pulsar "Enviar ruta al coche" sin abrir otro terminal.
# Para usar Stanley: cambia executable a waypoint_follower_stanley.

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    waypoint_follower = Node(
        package="qcar2_autonomy",
        executable="waypoint_follower_pure_pursuit",
        name="waypoint_follower_pure_pursuit",
    )
    waypoint_map_gui = Node(
        package="qcar2_autonomy",
        executable="waypoint_map_gui",
        name="waypoint_map_gui",
    )
    return LaunchDescription([waypoint_follower, waypoint_map_gui])
