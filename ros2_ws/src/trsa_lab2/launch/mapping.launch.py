"""Online async SLAM Toolbox mapping using this package's config/slam.yaml."""
from pathlib import Path
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import DeclareLaunchArgument, OpaqueFunction, IncludeLaunchDescription


def setup(context):
    lab = Path(get_package_share_directory('trsa_lab2'))
    slam = Path(get_package_share_directory('slam_toolbox'))
    params = lab / 'config' / 'slam.yaml'
    if not params.is_file():
        raise FileNotFoundError(f'Missing {params}')
    use_sim_time = LaunchConfiguration('use_sim_time').perform(context)
    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(slam / 'launch' / 'online_async_launch.py')),
            launch_arguments={
                'slam_params_file': str(params),
                'use_sim_time': use_sim_time,
            }.items()),
    ]


def generate_launch_description():
    defaults = {'use_sim_time': 'true'}
    return LaunchDescription([DeclareLaunchArgument(k, default_value=v) for k,v in defaults.items()] + [OpaqueFunction(function=setup)])
