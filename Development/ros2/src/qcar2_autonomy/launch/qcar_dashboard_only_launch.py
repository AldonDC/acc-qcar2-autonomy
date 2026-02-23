# Solo lo necesario para el dashboard GUI: sin RViz.
# Incluye: frame "map" (static) + pose del QCar desde QLabs (/qcar_pose_gt) + TF map->base_link.
# El Pure Pursuit necesita TF map->base_link para saber dónde está el coche; pose_gt_to_tf lo publica.
#
# Uso:
#   T1: ./run_t1.sh
#   T2: ./run_t2.sh  (opcional)
#   T3 (contenedor): ros2 launch qcar2_autonomy qcar_dashboard_only_launch.py
#   T4 (contenedor): teleop, T5: dashboard, T6: ros2 launch qcar2_autonomy qcar_follower_launch.py
#   Para Stanley en T6: ros2 launch qcar2_autonomy qcar_follower_launch.py use_stanley:=true

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    static_map = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="world_to_map",
        arguments=["0", "0", "0", "0", "0", "0", "world", "map"],
    )
    qcar_pose_qlabs = Node(
        package="qcar2_autonomy",
        executable="qcar_pose_from_qlabs_node",
        name="qcar_pose_from_qlabs",
    )
    pose_gt_to_tf = Node(
        package="qcar2_autonomy",
        executable="pose_gt_to_tf_node",
        name="pose_gt_to_tf",
    )
    return LaunchDescription([static_map, qcar_pose_qlabs, pose_gt_to_tf])
