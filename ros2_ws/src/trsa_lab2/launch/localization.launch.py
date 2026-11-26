"""LAB 2: Require map:=... and include nav2_bringup localization_launch.py using config/amcl.yaml; do not start SLAM."""
from pathlib import Path
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import DeclareLaunchArgument, OpaqueFunction, IncludeLaunchDescription


def setup(context):
    get = lambda k: LaunchConfiguration(k).perform(context)
    map_file = Path(get('map')).expanduser()
    if not get('map') or not map_file.is_file():
        raise FileNotFoundError(f'Pass an existing map:=/path/to/map.yaml (got "{get("map")}")')
    params = Path(get_package_share_directory('trsa_lab2')) / 'config' / 'amcl.yaml'
    if not params.is_file():
        raise FileNotFoundError(f'Missing {params}')
    nav2 = Path(get_package_share_directory('nav2_bringup'))
    return [IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(nav2 / 'launch' / 'localization_launch.py')),
        launch_arguments={
            'map': str(map_file.resolve()),
            'params_file': str(params),
            'use_sim_time': get('use_sim_time'),
            'autostart': 'true',
            'use_composition': 'False',
        }.items())]

def generate_launch_description():
    defaults = {'use_sim_time': 'true', 'map': ''}
    return LaunchDescription([DeclareLaunchArgument(k, default_value=v) for k,v in defaults.items()] + [OpaqueFunction(function=setup)])
