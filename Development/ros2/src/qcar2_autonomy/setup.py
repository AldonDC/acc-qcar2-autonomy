from setuptools import find_packages, setup
import os 
from glob import glob

package_name = 'qcar2_autonomy'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name,'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.rviz') + glob('config/*.yaml') + glob('config/*.png') + glob('config/*.jpg') + glob('config/*.txt'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ubuntu2404',
    maintainer_email='ubuntu2404@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'path_follower = autonomy.nav_to_pose:main',
            'traffic_system_detector=autonomy.traffic_system_detector:main',
            'lane_detector=autonomy.lane_detector:main',
            'yolo_detector=autonomy.yolo_detector:main',
            'trip_planner=autonomy.trip_planner:main',
            'simple_demo_nav=autonomy.simple_demo_nav:main',
            'simple_teleop_keyboard=autonomy.simple_teleop_keyboard:main',
            'waypoint_follower_controller=autonomy.waypoint_follower_controller:main',
            'waypoint_follower_stanley=autonomy.waypoint_follower_stanley:main',
            'waypoint_follower_pure_pursuit=autonomy.waypoint_follower_pure_pursuit:main',
            'obstacle_avoider=autonomy.obstacle_avoider:main',
            'waypoint_visual_node=autonomy.waypoint_visual_node:main',
            'waypoint_send_gui=autonomy.waypoint_send_gui:main',
            'demo_entrega_node=autonomy.demo_entrega_node:main',
            'waypoint_publish_from_file=autonomy.waypoint_publish_from_file:main',
            'waypoint_map_gui=autonomy.waypoint_map_gui:main',
            'circuit_map_publisher=autonomy.circuit_map_publisher:main',
            'waypoint_rviz_node=autonomy.waypoint_rviz_node:main',
            'robot_pose_marker_node=autonomy.robot_pose_marker_node:main',
            'route_recorder=autonomy.route_recorder:main',
            'route_player=autonomy.route_player:main'
        ],
    },
)
