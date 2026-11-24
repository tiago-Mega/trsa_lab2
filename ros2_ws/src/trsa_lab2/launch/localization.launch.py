"""TODO LAB 2: Require map:=... and include nav2_bringup localization_launch.py using config/amcl.yaml; do not start SLAM."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction


def setup(context):
    # TODO: Require map:=... and include nav2_bringup localization_launch.py using config/amcl.yaml; do not start SLAM.
    raise NotImplementedError('LAB2: implement localization.launch.py')


def generate_launch_description():
    defaults = {'use_sim_time': 'true', 'map': ''}
    return LaunchDescription([DeclareLaunchArgument(k, default_value=v) for k,v in defaults.items()]
                             + [OpaqueFunction(function=setup)])
