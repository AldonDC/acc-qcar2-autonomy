# Launch del seguidor de waypoints (Pure Pursuit o Stanley).
# Requiere: TF map->base_link (p. ej. qcar_dashboard_only_launch + pose_gt_to_tf).
# Los waypoints del GUI están en el mismo frame (map) para que el QCar vaya hacia los puntos.
#
# Uso:
#   Pure Pursuit (por defecto): ros2 launch qcar2_autonomy qcar_follower_launch.py
#   Stanley:                   ros2 launch qcar2_autonomy qcar_follower_launch.py use_stanley:=true

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_stanley_arg = DeclareLaunchArgument(
        "use_stanley",
        default_value="false",
        description="Si true, usa controlador Stanley; si false, Pure Pursuit.",
    )
    pure_pursuit = Node(
        package="qcar2_autonomy",
        executable="waypoint_follower_pure_pursuit",
        name="waypoint_follower_pure_pursuit",
        condition=UnlessCondition(LaunchConfiguration("use_stanley")),
    )
    stanley = Node(
        package="qcar2_autonomy",
        executable="waypoint_follower_stanley",
        name="waypoint_follower_stanley",
        condition=IfCondition(LaunchConfiguration("use_stanley")),
    )
    return LaunchDescription([use_stanley_arg, pure_pursuit, stanley])
