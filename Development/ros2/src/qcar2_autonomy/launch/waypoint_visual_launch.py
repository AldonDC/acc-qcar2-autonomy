# Launch para waypoints visuales: nodo visual + seguidor Stanley (pose desde TF).
# Stanley sigue mejor la trayectoria que el controlador P con dead-reckoning.
# RViz o waypoint_map_gui por separado.

from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node


def generate_launch_description():
    waypoint_visual = Node(
        package="qcar2_autonomy",
        executable="waypoint_visual_node",
        name="waypoint_visual_node",
    )
    waypoint_follower = ExecuteProcess(
        cmd=[
            "ros2", "run", "qcar2_autonomy", "waypoint_follower_stanley",
        ],
        shell=False,
    )
    return LaunchDescription([waypoint_visual, waypoint_follower])
