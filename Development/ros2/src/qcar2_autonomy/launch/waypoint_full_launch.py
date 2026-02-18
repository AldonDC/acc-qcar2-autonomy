# Todo en uno: QCar virtual + Cartographer + seguidor Stanley + GUI del mapa.
# Un solo launch en el mismo proceso para que /waypoints_path y /cmd_vel_nav se vean y el coche se mueva.

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    # Incluir el launch de QCar + Cartographer (mismo que run_qcar2_with_map.sh)
    qcar2_cartographer_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                get_package_share_directory("qcar2_nodes"),
                "launch",
                "qcar2_cartographer_virtual_launch.py",
            )
        ])
    )

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

    return LaunchDescription([
        qcar2_cartographer_launch,
        waypoint_follower,
        waypoint_map_gui,
    ])
