# articubot_one/launch/ldlidar.launch.py
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    # Find the ldlidar_node package
    ldlidar_pkg = get_package_share_directory('ldlidar_node')

    # Include the official bringup launch file
    ldlidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(ldlidar_pkg, 'launch', 'ldlidar_bringup.launch.py')
        ]),
        launch_arguments={
            'node_namespace': '',
            'node_name': 'ldlidar_node',
        }.items()
    )

    return LaunchDescription([
        ldlidar_launch
    ])

