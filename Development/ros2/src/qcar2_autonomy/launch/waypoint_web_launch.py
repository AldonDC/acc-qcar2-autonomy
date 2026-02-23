# Todo en uno: QCar virtual + Cartographer + Pure Pursuit + Web Dashboard.
# Un solo launch que arranca todo lo necesario.
#
# Terminal 1: ./run_t1.sh (arranca QLabs + escenario)
# Terminal 2: ros2 launch qcar2_autonomy waypoint_web_launch.py
# Luego abrir http://localhost:8085 en el navegador para controlar.

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    # QCar + Cartographer (nodos base)
    qcar2_cartographer_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                get_package_share_directory("qcar2_nodes"),
                "launch",
                "qcar2_cartographer_virtual_launch.py",
            )
        ])
    )

    # Pure Pursuit waypoint follower
    waypoint_follower = Node(
        package="qcar2_autonomy",
        executable="waypoint_follower_pure_pursuit",
        name="waypoint_follower_pure_pursuit",
    )

    # Web Dashboard (browser UI at http://localhost:8085)
    web_dashboard = Node(
        package="qcar2_autonomy",
        executable="qcar_web_dashboard",
        name="qcar_web_dashboard",
        parameters=[{
            "use_qlabs_view": True,
            "port": 8085,
        }],
        output="screen",
    )

    return LaunchDescription([
        qcar2_cartographer_launch,
        waypoint_follower,
        web_dashboard,
    ])
