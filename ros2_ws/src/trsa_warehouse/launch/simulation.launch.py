"""Simulation and robot interfaces only. No SLAM, AMCL, Nav2 or EKF."""
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, IncludeLaunchDescription, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from trsa_warehouse.model import prepare_files, resource_environment


def setup(context):
    share = Path(get_package_share_directory('trsa_warehouse'))
    get = lambda k: LaunchConfiguration(k).perform(context)
    world, profile = get('world'), get('sensor_profile')
    if world not in ('1', '2'):
        raise ValueError('world must be 1/2')
    if profile not in ('nominal', 'wheel_drift', 'imu_bias', 'custom'):
        raise ValueError('Unknown sensor profile')
    description, world_file = prepare_files(share, world, **{k:float(get(k)) for k in ('x','y','z','yaw')})
    sensor_path = Path(get('sensor_params')).expanduser() if get('sensor_params') else share/'config/sensors.yaml'
    rviz_path = Path(get('rviz_config')).expanduser() if get('rviz_config') else share/'config/warehouse.rviz'
    if not sensor_path.is_file() or not rviz_path.is_file():
        raise FileNotFoundError('Missing sensor configuration or RViz display file')
    env = resource_environment(share)
    gz = Path(get_package_share_directory('ros_gz_sim'))
    args = f'-r -v 3 {world_file}'
    args += f' --gui-config {share / "config/gazebo.config"}' if get('gui').lower() == 'true' else ' -s'
    return [
        SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', env['GZ_SIM_RESOURCE_PATH']),
        SetEnvironmentVariable('SDF_PATH', env['SDF_PATH']),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(gz/'launch/gz_sim.launch.py')),
            launch_arguments={'gz_args': args, 'on_exit_shutdown': 'true'}.items()),
        Node(package='robot_state_publisher', executable='robot_state_publisher', output='screen',
            parameters=[{'use_sim_time': True, 'robot_description': ParameterValue(description, value_type=str)}]),
        Node(package='ros_gz_bridge', executable='parameter_bridge', name='warehouse_bridge', output='screen',
            parameters=[{'use_sim_time': True, 'config_file': str(share/'config/bridge.yaml')}]),
        Node(package='trsa_warehouse', executable='sensor_adapter', name='sensor_adapter', output='screen',
            parameters=[str(sensor_path), {'use_sim_time': True, 'profile': profile}]),
        Node(package='trsa_warehouse', executable='wheel_odom_interface', name='wheel_odom_interface', output='screen',
            parameters=[{'use_sim_time': True}], condition=IfCondition(LaunchConfiguration('publish_wheel_tf'))),
        Node(package='rviz2', executable='rviz2', output='screen', arguments=['-d', str(rviz_path)],
            parameters=[{'use_sim_time': True}], condition=IfCondition(LaunchConfiguration('rviz'))),
    ]


def generate_launch_description():
    defaults = {'world':'1','gui':'true','rviz':'true','publish_wheel_tf':'true','sensor_profile':'nominal',
                'sensor_params':'','rviz_config':'','x':'0.0','y':'0.0','z':'0.02','yaw':'0.0'}
    return LaunchDescription([DeclareLaunchArgument(k, default_value=v) for k,v in defaults.items()]
                             + [OpaqueFunction(function=setup)])
