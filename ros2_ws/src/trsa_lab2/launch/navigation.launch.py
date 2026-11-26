"""TODO LAB 2: Include the installed nav2_bringup navigation_launch.py, use_composition:=False, config/nav2.yaml and the Nav2 Collision Monitor as final velocity publisher on /cmd_vel."""
"""Nav2 navigation stack (installed navigation_launch.py) with config/nav2.yaml. Velocity chain: controller -> cmd_vel_nav -> velocity_smoother -> cmd_vel_smoothed
-> collision_monitor -> /cmd_vel (unstamped Twist)."""
from pathlib import Path
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import DeclareLaunchArgument, OpaqueFunction, IncludeLaunchDescription


def setup(context):
    get = lambda k: LaunchConfiguration(k).perform(context)
    default = Path(get_package_share_directory('trsa_lab2')) / 'config' / 'nav2.yaml'
    params = Path(get('params_file')).expanduser() if get('params_file') else default
    if not params.is_file():
        raise FileNotFoundError(f'Missing Nav2 params: {params}')
    nav2 = Path(get_package_share_directory('nav2_bringup'))
    return [IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(nav2 / 'launch' / 'navigation_launch.py')),
        launch_arguments={
            'params_file': str(params.resolve()),
            'use_sim_time': get('use_sim_time'),
            'autostart': 'true',
            'use_composition': 'False',
        }.items())]



def generate_launch_description():
    defaults = {'use_sim_time': 'true', 'params_file': ''}
    return LaunchDescription([DeclareLaunchArgument(k, default_value=v) for k,v in defaults.items()] + [OpaqueFunction(function=setup)])
