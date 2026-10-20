"""Local LIO with an optional independent loop-closure backend."""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory('super_lio')

    def setup(context):
        enabled = LaunchConfiguration('loop_closure').perform(context).lower() == 'true'
        sensor = LaunchConfiguration('config').perform(context)
        rviz = LaunchConfiguration('rviz').perform(context).lower() == 'true'
        sim = LaunchConfiguration('use_sim_time').perform(context).lower() == 'true'
        loop_config = LaunchConfiguration('loop_config').perform(context)
        nodes = [Node(package='super_lio', executable='super_lio_node', output='screen',
                      parameters=[sensor, loop_config, {'lio.loop.enabled': enabled, 'use_sim_time': sim}])]
        if rviz:
            nodes.append(Node(package='rviz2', executable='rviz2',
                              arguments=['-d', os.path.join(share, 'rviz', 'loop.rviz' if enabled else 'lio.rviz')],
                              parameters=[{'use_sim_time': sim}]))
        return nodes

    return LaunchDescription([
        DeclareLaunchArgument('config', default_value=os.path.join(share, 'config', 'livox_360.yaml')),
        DeclareLaunchArgument('loop_config', default_value=os.path.join(share, 'config', 'loop_closure.yaml')),
        DeclareLaunchArgument('loop_closure', default_value='false', choices=['true', 'false']),
        DeclareLaunchArgument('rviz', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('use_sim_time', default_value='false', choices=['true', 'false']),
        OpaqueFunction(function=setup),
    ])
