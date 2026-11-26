#!/usr/bin/env python3
"""Lab 2 route-action assignment. This file is just a template.

The ROS 2 executable is scripts/route_client; this Python module defines the
node it starts. Students implement the four methods marked TODO below.
No external route_support.py is required.
"""
import json
import math
import time
import yaml
import rclpy
import argparse

from pathlib import Path
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.parameter import Parameter

from geometry_msgs.msg import Twist
from nav2_msgs.action import NavigateToPose


class StudentRoute(Node):
    def __init__(self):
        super().__init__('lab2_route_client', parameter_overrides=[
            Parameter('use_sim_time', value=True)])
        self.action = ActionClient(self, NavigateToPose, '/navigate_to_pose')
        self.pending = None
        self.active = None
        self.cancel_requested = False
        self.records = []
        self.safety = {}
        # Last-stage direct-bridge command, used only after cancelling Nav2.
        self.zero_pub = self.create_publisher(Twist, '/cmd_vel', 10)

    def make_goal(self, waypoint):
        """TODO LAB 2: Construct a NavigateToPose goal stamped in map and sim time.
        Convert the waypoint's yaw (radians) to a valid quaternion.
        """
        raise NotImplementedError('LAB2: implement make_goal')

    def navigate(self, waypoint, timeout):
        """TODO LAB 2: Wait for action server; send, monitor, and await one goal.

        Differentiate acceptance from success; handle cancellation, timeout,
        action result status and delayed goal acceptance. Store result/timing
        in self.records; return True ONLY for verified success.
        """
        raise NotImplementedError('LAB2: implement navigate')

    def execute_route(self, waypoints, timeout):
        """TODO LAB 2: Execute waypoints in order; stop on the first failed goal.
        No later goal may be sent after rejection, abort, cancellation or timeout.
        """
        raise NotImplementedError('LAB2: implement execute_route')

    def cancel_and_stop(self):
        """TODO LAB 2: Request Nav2 action cancellation (including pending acceptance).
        Verify acknowledgement and terminal result; do not start the next goal.
        After cancelling, issue a zero Twist on /cmd_vel and verify that the
        simulated robot has stopped. There is NO /control/stop service or
        latched stop when using this direct-Gazebo simulator.
        Store evidence in self.safety.
        """
        raise NotImplementedError('LAB2: implement cancel_and_stop')


def read_route(filename):
    """Provided file/CLI validation; navigation algorithms remain student work."""
    data = yaml.safe_load(filename.read_text(encoding='utf-8'))
    if not isinstance(data, dict) or data.get('frame_id') != 'map':
        raise ValueError('Route must declare frame_id: map.')
    waypoints = data.get('waypoints')
    if not isinstance(waypoints, list) or len(waypoints) < 4:
        raise ValueError('Provide at least four map-frame waypoints.')
    for i, waypoint in enumerate(waypoints, start=1):
        if not isinstance(waypoint, dict) or not all(
            key in waypoint and isinstance(waypoint[key], (int, float))
            and not isinstance(waypoint[key], bool)
            and math.isfinite(waypoint[key]) for key in ('x', 'y', 'yaw')
        ):
            raise ValueError(f'Waypoint {i} must have finite numeric x, y and yaw.')
    return waypoints


def main():
    # TODO LAB 2
    return


if __name__ == '__main__':
    main()
