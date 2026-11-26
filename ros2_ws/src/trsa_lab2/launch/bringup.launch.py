"""Integrate simulation, optional observer/active EKF, SLAM or AMCL, and Nav2.

Three supported modes:
- odometry_source=wheel, use_ekf=false: wheel-only baseline
- odometry_source=wheel, use_ekf=true: wheel control + observer EKF
- odometry_source=ekf, use_ekf=true: EKF publishes odom TF and /odom
The simulator must never launch twice. An optional, provided launch_support.after_navigation_ready helper gates Nav2.
If navigation is enabled, only start Nav2 after required map, odometry, and
TF are ready (e.g. using launch OnProcessExit and trsa_warehouse/nav_check).
"""
from pathlib import Path
from ament_index_python.packages import get_package_share_directory as pkg

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import DeclareLaunchArgument, OpaqueFunction, IncludeLaunchDescription, GroupAction


def inc(pkgname, rel, args):
    return  GroupAction(scoped=True, forwarding=False, actions=[
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(Path(pkg(pkgname)) / rel)),
            launch_arguments={k: str(v) for k, v in args.items()}.items()
        )
    ])

def setup(context):
    # Implement the three modes above, plus mapping vs localisation.
    # Simulator argument: publish_wheel_tf true only when odometry_source == 'wheel'.
    # Filter: publish_tf true and output /odom only when odometry_source == 'ekf'.
    # Otherwise observer filter: publish_tf false, output /odometry/filtered.
    # Optionally use trsa_lab2.launch_support.after_navigation_ready on your
    # own navigation include to wait for map/scan/odom/TF readiness.
    # Pass --without-ekf only when use_ekf is false. Avoid a fixed sleep.
    g = lambda k: LaunchConfiguration(k).perform(context)

    mode, src = g('mode'), g('odometry_source')
    use_ekf = g('use_ekf').lower() == 'true'
    navigation = g('navigation').lower() == 'true'

    if mode not in ('mapping', 'localization'):
        raise ValueError("mode must be 'mapping' or 'localization'")
    if src not in ('wheel', 'ekf'):
        raise ValueError("odometry_source must be 'wheel' or 'ekf'")
    if src == 'ekf' and not use_ekf:
        raise ValueError('odometry_source=ekf requires use_ekf=true')
    if mode == 'localization' and not g('map'):
        raise ValueError('mode:=localization requires map:=/path/to/map.yaml')

    active = (src == 'ekf')
    nav2_yaml = str(Path(pkg('trsa_lab2')) / 'config' / 'nav2.yaml')
    rviz_cfg = str(Path(pkg('trsa_warehouse')) / 'config' / 'navigation.rviz')

    actions = [inc('trsa_warehouse', 'launch/simulation.launch.py', {
        'publish_wheel_tf': 'false' if active else 'true',
        'rviz': g('rviz'),
        'sensor_profile': g('sensor_profile'),
        'rviz_config': rviz_cfg,
    })]  # simulator launched ONCE

    if use_ekf:
        actions.append(inc('trsa_lab2', 'launch/fusion.launch.py', {
            'publish_tf': 'true' if active else 'false',
            'output_odom_topic': '/odom' if active else '/odometry/filtered',
        }))  # EKF launched ONCE

    if mode == 'mapping':
        actions.append(inc('trsa_lab2', 'launch/mapping.launch.py', {}))
    else:
        actions.append(inc('trsa_lab2', 'launch/localization.launch.py', {'map': g('map')}))

    if navigation:
        actions.append(inc('trsa_lab2', 'launch/navigation.launch.py', {'params_file': nav2_yaml, 'use_sim_time': 'true'}))

    return actions

def generate_launch_description():
    defaults = {'mode': 'mapping', 'map': '', 'use_ekf': 'false',
                'odometry_source': 'wheel', 'navigation': 'true',
                'gui': 'true', 'rviz': 'true', 'sensor_profile': 'nominal'}
    return LaunchDescription([DeclareLaunchArgument(k, default_value=v) for k,v in defaults.items()] + [OpaqueFunction(function=setup)])
