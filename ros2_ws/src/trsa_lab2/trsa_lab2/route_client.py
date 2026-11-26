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
import rclpy.time

from pathlib import Path

from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.parameter import Parameter

from nav_msgs.msg import Odometry
from action_msgs.msg import GoalStatus
from action_msgs.srv import CancelGoal
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import Twist, PoseStamped

STATUS = {  GoalStatus.STATUS_UNKNOWN:      'UNKNOWN', 
            GoalStatus.STATUS_ACCEPTED:     'ACCEPTED',
            GoalStatus.STATUS_EXECUTING:    'EXECUTING', 
            GoalStatus.STATUS_CANCELING:    'CANCELING',
            GoalStatus.STATUS_SUCCEEDED:    'SUCCEEDED', 
            GoalStatus.STATUS_CANCELED:     'CANCELED',
            GoalStatus.STATUS_ABORTED:      'ABORTED'}

class StudentRoute(Node):
    def __init__(self):
        super().__init__('lab2_route_client', parameter_overrides=[Parameter('use_sim_time', value=True)])
        self.action = ActionClient(self, NavigateToPose, '/navigate_to_pose')
        self.pending = None
        self.active = None
        self.cancel_requested = False
        self.records = []
        self.safety = {}
        self.last_feedback = None
        # Last-stage direct-bridge command, used only after cancelling Nav2.
        self.zero_pub = self.create_publisher(Twist, '/cmd_vel', 10)
    
    # ---------- helpers ----------
    def _wait(self, done, timeout):
        """Spin until done() is True, timeout (wall s) or cancel request."""
        end = time.monotonic() + timeout
        while rclpy.ok() and not done():
            if time.monotonic() > end:
                return False
            rclpy.spin_once(self, timeout_sec=0.1)
        return done()

    def _feedback(self, msg):
        fb = msg.feedback
        self.last_feedback = {
            'distance_remaining': float(fb.distance_remaining),
            'recoveries': int(fb.number_of_recoveries)}

    def make_goal(self, waypoint):
        goal = NavigateToPose.Goal()
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()  # sim time
        pose.pose.position.x = float(waypoint['x'])
        pose.pose.position.y = float(waypoint['y'])
        yaw = float(waypoint['yaw'])
        pose.pose.orientation.z = math.sin(yaw / 2.0)
        pose.pose.orientation.w = math.cos(yaw / 2.0)
        goal.pose = pose
        return goal
        

    def navigate(self, waypoint, timeout):
        rec = {'waypoint': {k: waypoint[k] for k in ('x', 'y', 'yaw')},
               'accepted': False, 'status': None, 'success': False,
               'error_code': None, 'reason': '', 'duration_s': None,
               'recoveries': None}
        self.records.append(rec)
        self.last_feedback = None
        t0 = time.monotonic()

        if not self.action.wait_for_server(timeout_sec=15.0):
            rec['reason'] = 'action server unavailable'
            return False
        if self.cancel_requested:
            rec['reason'] = 'cancelled before sending'
            return False

        self.pending = self.action.send_goal_async(
            self.make_goal(waypoint), feedback_callback=self._feedback)
        if not self._wait(lambda: self.pending.done() or self.cancel_requested, 15.0) \
                or not self.pending.done():
            rec['reason'] = 'goal acceptance timeout or cancelled while pending'
            self.cancel_and_stop()
            rec['duration_s'] = time.monotonic() - t0
            return False

        handle = self.pending.result()
        self.pending = None
        if not handle.accepted:
            rec['reason'] = 'goal rejected'
            rec['status'] = 'REJECTED'
            rec['duration_s'] = time.monotonic() - t0
            return False
        rec['accepted'] = True
        self.active = handle

        result_future = handle.get_result_async()
        finished = self._wait(lambda: result_future.done() or self.cancel_requested,
                              timeout)
        if not result_future.done():
            rec['reason'] = 'cancelled by user' if self.cancel_requested else 'timeout'
            self.cancel_and_stop()
            if result_future.done():
                rec['status'] = STATUS.get(result_future.result().status)
            rec['duration_s'] = time.monotonic() - t0
            if self.last_feedback:
                rec['recoveries'] = self.last_feedback['recoveries']
            return False

        res = result_future.result()
        rec['status'] = STATUS.get(res.status, str(res.status))
        rec['error_code'] = int(res.result.error_code)
        rec['reason'] = res.result.error_msg
        rec['duration_s'] = time.monotonic() - t0
        if self.last_feedback:
            rec['recoveries'] = self.last_feedback['recoveries']
        rec['success'] = res.status == GoalStatus.STATUS_SUCCEEDED
        self.active = None
        return rec['success']


    def execute_route(self, waypoints, timeout):
        for i, wp in enumerate(waypoints, start=1):
            self.get_logger().info(f'Goal {i}/{len(waypoints)}: {wp}')
            if not self.navigate(wp, timeout):
                self.get_logger().error(f'Goal {i} failed; stopping route.')
                return False
        return True

    def cancel_and_stop(self):
        self.cancel_requested = True
        ev = {'cancel_ack': False, 'terminal_status': None,
              'zero_published': False, 'stopped': False, 'max_speed_after': None}

        # Goal still waiting for acceptance: let it resolve, then cancel it.
        if self.pending is not None and self.active is None:
            self._wait(self.pending.done, 5.0)
            if self.pending.done() and self.pending.result().accepted:
                self.active = self.pending.result()
            self.pending = None

        if self.active is not None:
            fut = self.active.cancel_goal_async()
            if self._wait(fut.done, 5.0):
                ev['cancel_ack'] = (fut.result().return_code
                                    == CancelGoal.Response.ERROR_NONE)
            res = self.active.get_result_async()
            if self._wait(res.done, 10.0):
                ev['terminal_status'] = STATUS.get(res.result().status)
            self.active = None

        # Explicit zero command, then verify via /odom twist.
        speeds = []
        sub = self.create_subscription(
            Odometry, '/odom',
            lambda m: speeds.append(math.hypot(m.twist.twist.linear.x,
                                               m.twist.twist.linear.y)
                                    + abs(m.twist.twist.angular.z)), 10)
        for _ in range(10):
            self.zero_pub.publish(Twist())
            rclpy.spin_once(self, timeout_sec=0.05)
        ev['zero_published'] = True
        speeds.clear()
        self._wait(lambda: False, 1.5)          # collect ~1.5 s of odometry
        self.destroy_subscription(sub)
        if speeds:
            ev['max_speed_after'] = max(speeds)
            ev['stopped'] = max(speeds) < 0.02
        self.safety = ev
        self.get_logger().info(f'Stop evidence: {ev}')

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
    ap = argparse.ArgumentParser()
    ap.add_argument('--route', type=Path, default=Path('data/route.yaml'))
    ap.add_argument('--timeout', type=float, default=180.0, help='per-goal, seconds')
    ap.add_argument('--output', type=Path, default=Path('route_result.json'))
    ap.add_argument('--cancel-after', type=float, default=None,
                    help='E4: cancel the active goal after N seconds')
    args = ap.parse_args()
    waypoints = read_route(args.route)

    rclpy.init()
    node = StudentRoute()
    ok = False
    try:
        if args.cancel_after is not None:
            node.create_timer(args.cancel_after,
                              lambda: setattr(node, 'cancel_requested', True))
        ok = node.execute_route(waypoints, args.timeout)
    except KeyboardInterrupt:
        node.get_logger().warn('Interrupted: cancelling.')
        node.cancel_and_stop()
    finally:
        if not node.safety:
            node.cancel_and_stop()
        out = {'route': str(args.route), 'success': ok,
               'records': node.records, 'safety': node.safety}
        args.output.write_text(json.dumps(out, indent=2))
        node.get_logger().info(f'Wrote {args.output}; route success={ok}')
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
