"""TODO LAB 2: Include the installed nav2_bringup navigation_launch.py, use_composition:=False, config/nav2.yaml and the Nav2 Collision Monitor as final velocity publisher on /cmd_vel."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction


def setup(context):
    # TODO: Include the installed nav2_bringup navigation_launch.py, use_composition:=False, config/nav2.yaml and the Nav2 Collision Monitor as final velocity publisher on /cmd_vel.
    raise NotImplementedError('LAB2: implement navigation.launch.py')


def generate_launch_description():
    defaults = {'use_sim_time': 'true'}
    return LaunchDescription([DeclareLaunchArgument(k, default_value=v) for k,v in defaults.items()]
                             + [OpaqueFunction(function=setup)])
