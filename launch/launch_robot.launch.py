import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, RegisterEventHandler
from launch.event_handlers import OnProcessStart
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import Command


def generate_launch_description():

    package_name = 'articubot_one'

    # --- Robot State Publisher (loads your URDF)
    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory(package_name), 'launch', 'rsp.launch.py')
        ]),
        launch_arguments={
            'use_sim_time': 'false',
            'use_ros2_control': 'false'
        }.items()
    )

    # --- Twist Mux (optional)
    twist_mux_params = os.path.join(
        get_package_share_directory(package_name), 'config', 'twist_mux.yaml'
    )

    twist_mux = Node(
        package="twist_mux",
        executable="twist_mux",
        parameters=[twist_mux_params],
        remappings=[('/cmd_vel_out', '/diff_cont/cmd_vel_unstamped')]
    )

    encoder_odom = Node(
        package='articubot_one',
        executable='encoder_odometry',
        name='encoder_odometry',
        # output='screen',
        parameters=[{
            'wheel_radius': 0.033,
            'wheel_base': 0.297,
            'encoder_cpr': 360
        }]
    )

    # --- Static transforms for your wheels
    left_wheel_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='left_wheel_tf',
        arguments=['0', '0.1485', '0', '0', '0', '0', 'base_link', 'left_wheel']
    )

    right_wheel_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='right_wheel_tf',
        arguments=['0', '-0.1485', '0', '0', '0', '0', 'base_link', 'right_wheel']
    )

    ldlidar_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='ldlidar_tf',
        arguments=['0', '0', '0.1', '0', '0', '0', 'base_link', 'ldlidar_base']
    )

    ldlidar_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                get_package_share_directory('ldlidar_node'),
                'launch',
                'ldlidar_bringup.launch.py'
            )
        ]),
        launch_arguments={'node_namespace': '', 'node_name': 'ldlidar_node'}.items()
    )

    # --- (Optional) Controller setup (commented out since you’re not using ros2_control)
    # robot_description = Command(['ros2 param get --hide-type /robot_state_publisher robot_description'])
    # controller_params_file = os.path.join(
    #     get_package_share_directory(package_name), 'config', 'my_controllers.yaml'
    # )
    # controller_manager = Node(
    #     package="controller_manager",
    #     executable="ros2_control_node",
    #     parameters=[{'robot_description': robot_description}, controller_params_file]
    # )
    # delayed_controller_manager = TimerAction(period=3.0, actions=[controller_manager])

    # --- Launch them all!
    return LaunchDescription([
        rsp,
        twist_mux,
        left_wheel_tf,
        right_wheel_tf,
        ldlidar_bringup,
        ldlidar_tf,
        encoder_odom
        # delayed_controller_manager,
    ])
