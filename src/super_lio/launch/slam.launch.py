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
        localization = LaunchConfiguration('localization').perform(context).lower() == 'true'
        if enabled and localization:
            raise RuntimeError('loop_closure and localization cannot both be true')
        sensor = LaunchConfiguration('config').perform(context)
        rviz = LaunchConfiguration('rviz').perform(context).lower() == 'true'
        sim = LaunchConfiguration('use_sim_time').perform(context).lower() == 'true'
        loop_config = LaunchConfiguration('loop_config').perform(context)
        localization_config = LaunchConfiguration('localization_config').perform(context)
        overrides = {'lio.loop.enabled': enabled, 'lio.localization.enabled': localization, 'use_sim_time': sim}
        index_reference = LaunchConfiguration('index_reference_map').perform(context)
        index_output = LaunchConfiguration('index_output').perform(context)
        if index_output and not index_reference:
            raise RuntimeError('index_output requires index_reference_map')
        if index_reference:
            if not enabled or localization:
                raise RuntimeError('Index generation requires loop_closure:=true and localization:=false')
            overrides['lio.map.save_map'] = False
            overrides['lio.loop.place_index_reference_map'] = os.path.expanduser(index_reference)
            overrides['lio.loop.place_index_path'] = os.path.expanduser(index_output or index_reference + '.index')
        if localization:
            overrides['lio.map.save_map'] = False
            map_path = LaunchConfiguration('map_path').perform(context)
            if map_path:
                overrides['lio.localization.map_path'] = map_path
        nodes = [Node(package='super_lio', executable='super_lio_node', output='screen',
                      sigterm_timeout='120', sigkill_timeout='10',
                      parameters=[sensor, loop_config, localization_config, overrides])]
        if rviz:
            nodes.append(Node(package='rviz2', executable='rviz2',
                              arguments=['-d', os.path.join(share, 'rviz', 'localization.rviz' if localization else ('loop.rviz' if enabled else 'lio.rviz'))],
                              parameters=[{'use_sim_time': sim}]))
        return nodes

    return LaunchDescription([
        DeclareLaunchArgument('config', default_value=os.path.join(share, 'config', 'livox_360.yaml')),
        DeclareLaunchArgument('loop_config', default_value=os.path.join(share, 'config', 'loop_closure.yaml')),
        DeclareLaunchArgument('index_reference_map', default_value=''),
        DeclareLaunchArgument('index_output', default_value=''),
        DeclareLaunchArgument('localization_config', default_value=os.path.join(share, 'config', 'localization.yaml')),
        DeclareLaunchArgument('localization', default_value='false', choices=['true', 'false']),
        DeclareLaunchArgument('map_path', default_value=''),
        DeclareLaunchArgument('loop_closure', default_value='false', choices=['true', 'false']),
        DeclareLaunchArgument('rviz', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('use_sim_time', default_value='false', choices=['true', 'false']),
        OpaqueFunction(function=setup),
    ])
