import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg = 'articubot_one'

    # Args
    use_sim_time = LaunchConfiguration('use_sim_time')
    odom_pkg     = LaunchConfiguration('odom_pkg')
    odom_launch  = LaunchConfiguration('odom_launch')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false',
                              description='Use simulated clock'),
        # Point these to your *actual* odometry package + launch file
        DeclareLaunchArgument('odom_pkg', default_value='encoder_odometry',
                              description='Package that contains the odom launch'),
        DeclareLaunchArgument('odom_launch', default_value='encoder_odometry.launch.py',
                              description='Launch file that starts the odom node(s)'),

        # URDF + robot_state_publisher (handled inside rsp.launch.py)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(get_package_share_directory(pkg), 'launch', 'rsp.launch.py')
            ]),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'use_ros2_control': 'false'
            }.items()
        ),

        # Your external odom bringup (encoder → /odom + TF)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                FindPackageShare(odom_pkg), '/launch/', odom_launch
            ]),
            launch_arguments={'use_sim_time': use_sim_time}.items()
        ),

        # Twist mux → publish plain /cmd_vel (no ros2_control topics)
        Node(
            package='twist_mux',
            executable='twist_mux',
            output='screen',
            parameters=[
                os.path.join(get_package_share_directory(pkg), 'config', 'twist_mux.yaml'),
                {'use_sim_time': use_sim_time}
            ],
            remappings=[('/cmd_vel_out', '/cmd_vel')],
        ),
    ])
