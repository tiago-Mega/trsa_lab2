"""Start robot_localization ekf_node as ekf_filter_node using config/ekf.yaml. Support launch arguments `publish_tf` (default false) and `output_odom_topic` (default /odometry/filtered) so the EKF can run first as an observer and later as the active local odometry source."""

from pathlib import Path
from ament_index_python.packages import get_package_share_directory

from launch_ros.actions import Node
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, OpaqueFunction


def setup(context):
    get = lambda k: LaunchConfiguration(k).perform(context)
    cfg = Path(get_package_share_directory('trsa_lab2')) / 'config/ekf.yaml'
    return [Node(
        package='robot_localization', executable='ekf_node',
        name='ekf_filter_node', output='screen',
        parameters=[str(cfg), {
            'use_sim_time': get('use_sim_time').lower() == 'true',
            'publish_tf': get('publish_tf').lower() == 'true'}],
        remappings=[('odometry/filtered', get('output_odom_topic'))])]


def generate_launch_description():
    defaults = {'use_sim_time': 'true', 'publish_tf': 'false', 'output_odom_topic': '/odometry/filtered'}
    return LaunchDescription([DeclareLaunchArgument(k, default_value=v) for k, v in defaults.items()] + [OpaqueFunction(function=setup)])
