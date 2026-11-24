"""TODO LAB 2: integrate simulation, optional observer/active EKF, SLAM or AMCL, and Nav2.

Three supported modes:
- odometry_source=wheel, use_ekf=false: wheel-only baseline
- odometry_source=wheel, use_ekf=true: wheel control + observer EKF
- odometry_source=ekf, use_ekf=true: EKF publishes odom TF and /odom
The simulator must never launch twice. An optional, provided launch_support.after_navigation_ready helper gates Nav2.
If navigation is enabled, only start Nav2 after required map, odometry, and
TF are ready (e.g. using launch OnProcessExit and trsa_warehouse/nav_check).
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction


def setup(context):
    # TODO: Implement the three modes above, plus mapping vs localisation.
    # Simulator argument: publish_wheel_tf true only when odometry_source == 'wheel'.
    # Filter: publish_tf true and output /odom only when odometry_source == 'ekf'.
    # Otherwise observer filter: publish_tf false, output /odometry/filtered.
    # Optionally use trsa_lab2.launch_support.after_navigation_ready on your
    # own navigation include to wait for map/scan/odom/TF readiness.
    # Pass --without-ekf only when use_ekf is false. Avoid a fixed sleep.
    raise NotImplementedError('LAB2: implement integrated bringup')


def generate_launch_description():
    defaults = {'mode':'mapping', 'map':'', 'use_ekf':'false', 'odometry_source':'wheel',
                'navigation':'true', 'gui':'true', 'rviz':'true', 'sensor_profile':'nominal'}
    return LaunchDescription([DeclareLaunchArgument(k, default_value=v) for k,v in defaults.items()]
                             + [OpaqueFunction(function=setup)])
