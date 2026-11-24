"""TODO LAB 2: Start robot_localization ekf_node as ekf_filter_node using config/ekf.yaml. Support launch arguments `publish_tf` (default false) and `output_odom_topic` (default /odometry/filtered) so the EKF can run first as an observer and later as the active local odometry source."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction


def setup(context):
    # TODO: Start robot_localization ekf_node as ekf_filter_node using config/ekf.yaml.
    # Use `use_sim_time` true, forward the `publish_tf` argument, and remap
    # `odometry/filtered` to the requested `output_odom_topic`.
    raise NotImplementedError('LAB2: implement fusion.launch.py')


def generate_launch_description():
    defaults = {'use_sim_time': 'true', 'publish_tf': 'false', 'output_odom_topic': '/odometry/filtered'}
    return LaunchDescription([DeclareLaunchArgument(k, default_value=v) for k, v in defaults.items()]
                             + [OpaqueFunction(function=setup)])
