import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('rov_physics_sim')
    gz_pkg = get_package_share_directory('ros_gz_sim')
    world = os.path.join(pkg, 'worlds', 'empty.sdf')

    headless = LaunchConfiguration('headless')

    gz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gz_pkg, 'launch', 'gz_sim.launch.py')),
        launch_arguments={
            'gz_args': PythonExpression([
                "('-r -s ' if '", headless, "' == 'true' else '-r ') + '", world, "'"
            ]),
        }.items())

    clock_bridge = Node(
        package='ros_gz_bridge', executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        output='screen')

    foxglove = Node(
        package='foxglove_bridge', executable='foxglove_bridge',
        parameters=[{'port': 8765}], output='screen')

    return LaunchDescription([
        DeclareLaunchArgument('headless', default_value='false'),
        gz, clock_bridge, foxglove,
    ])
