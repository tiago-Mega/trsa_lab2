"""Non-moving readiness check plus opt-in bounded motion and lift tests."""
import argparse
import json
import math
import sys
import time
from collections import defaultdict
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, LaserScan, JointState
from rosgraph_msgs.msg import Clock
from std_msgs.msg import Float64


def yaw(pose):
    q = pose.orientation
    return math.atan2(2*(q.w*q.z + q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))

def angle_delta(a, b):
    return abs(math.atan2(math.sin(a-b), math.cos(a-b)))

class Check(Node):
    def __init__(self):
        super().__init__('trsa_check_' + str(int(time.time()*1000)))
        self.latest = {}; self.received = {}; self.count = defaultdict(int)
        self.first_sim = None; self.sim_time = None
        self.lift_report = None
        self.subscriptions_keep = []
        for typ, name in [(Clock, '/clock'), (Odometry, '/odom'), (Odometry, '/ground_truth/odom'),
                          (Imu, '/imu'), (LaserScan, '/scan'), (JointState, '/joint_states'),
                          (Float64, '/lift/command')]:
            self.subscriptions_keep.append(self.create_subscription(
                typ, name, lambda m, t=name: self.record(t, m), qos_profile_sensor_data))
        self.buffer = Buffer(); self.listener = TransformListener(self.buffer, self)
        self.vel = self.create_publisher(Twist, '/cmd_vel', 10)
        self.lift = self.create_publisher(Float64, '/lift/command', 10)

    def record(self, name, msg):
        self.latest[name] = msg; self.received[name] = time.monotonic(); self.count[name] += 1
        if name == '/clock':
            self.sim_time = msg.clock.sec + msg.clock.nanosec*1e-9
            if self.first_sim is None: self.first_sim = self.sim_time

    def problems(self):
        problems = []
        required = ('/clock', '/odom', '/ground_truth/odom', '/imu', '/scan', '/joint_states')
        for name in required:
            if self.count[name] < 2: problems.append('Waiting for live data: ' + name)
            elif time.monotonic() - self.received[name] > 5: problems.append('Stale data: ' + name)
        if self.sim_time is None or self.sim_time - (self.first_sim or 0) <= 0.05:
            problems.append('Simulation clock is not advancing')
        scan = self.latest.get('/scan')
        if scan is not None:
            if len(scan.ranges) != 360: problems.append('Expected 360 LiDAR samples')
            if scan.header.frame_id != 'laser_frame': problems.append('Wrong scan frame: ' + scan.header.frame_id)
            if not any(math.isfinite(r) and scan.range_min <= r <= scan.range_max for r in scan.ranges):
                problems.append('LiDAR sees no finite in-range warehouse returns')
        imu = self.latest.get('/imu')
        if imu is not None and imu.header.frame_id != 'imu_link': problems.append('Wrong IMU frame: ' + imu.header.frame_id)
        odom = self.latest.get('/odom')
        if odom is not None:
            if (odom.header.frame_id, odom.child_frame_id) != ('odom', 'base_footprint'):
                problems.append('Unexpected odometry frames')
            q = odom.pose.pose.orientation
            if not math.isclose(q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w, 1.0, abs_tol=0.02):
                problems.append('Invalid odometry orientation')
        joints = self.latest.get('/joint_states')
        if joints is not None:
            if not {'left_wheel_joint', 'right_wheel_joint', 'lift_joint'}.issubset(joints.name):
                problems.append('Missing wheel/lift joint states: ' + str(joints.name))
        for target in ('base_footprint', 'base_link', 'laser_frame', 'imu_link', 'platform_link'):
            if not self.buffer.can_transform('odom', target, Time()):
                problems.append('Missing TF: odom -> ' + target)
        return problems

    def spin_for(self, duration, callback=None):
        end = time.monotonic() + duration
        while rclpy.ok() and time.monotonic() < end:
            if callback: callback()
            rclpy.spin_once(self, timeout_sec=0.05)
            time.sleep(0.02)

    def motion_test(self):
        self.spin_for(0.5)
        start_yaw = yaw(self.latest['/ground_truth/odom'].pose.pose)
        start_odom = yaw(self.latest['/odom'].pose.pose)
        start_sim = self.sim_time
        command = Twist(); command.angular.z = 0.3
        end = time.monotonic() + 45
        try:
            while self.sim_time - start_sim < 2.0 and time.monotonic() < end:
                self.vel.publish(command)
                rclpy.spin_once(self, timeout_sec=0.05)
                time.sleep(0.03)
            # No external watchdog is provided. Publish an explicit zero command.
            # Then inspect that the robot settles with no command source active.
            stopped_at = self.sim_time
            self.spin_for(0.5, lambda: self.vel.publish(Twist()))
            end = time.monotonic() + 20
            while self.sim_time - stopped_at < 1.0 and time.monotonic() < end:
                rclpy.spin_once(self, timeout_sec=0.05)
            truth_change = angle_delta(yaw(self.latest['/ground_truth/odom'].pose.pose), start_yaw)
            odom_change = angle_delta(yaw(self.latest['/odom'].pose.pose), start_odom)
            speed = abs(self.latest['/odom'].twist.twist.angular.z)
            if truth_change < 0.08 or odom_change < 0.08:
                raise RuntimeError(f'Rotation failed: truth={truth_change:.3f} rad; odom={odom_change:.3f} rad')
            if speed > 0.05:
                raise RuntimeError(f'Robot still rotating after zero command: odom yaw rate={speed:.3f}')
            return {'ground_truth_yaw_change_rad': truth_change,
                    'wheel_odom_yaw_change_rad': odom_change, 'final_yaw_rate': speed}
        finally:
            self.vel.publish(Twist())

    def lift_position(self):
        msg = self.latest.get('/joint_states')
        if msg is None or 'lift_joint' not in msg.name:
            return None
        index = msg.name.index('lift_joint')
        if index >= len(msg.position) or not math.isfinite(msg.position[index]):
            return None
        return float(msg.position[index])

    def lift_test(self, wall_timeout=120.0):
        """Test both endpoints and an intermediate hold using fresh feedback.

        The ROS target is bridged directly to Gazebo, without any command guard.
        Observing a local ROS command does not prove Gazebo received it; joint
        movement is the relevant endpoint evidence. Timeouts use wall time; stability is measured in simulation time.
        """
        if not math.isfinite(wall_timeout) or wall_timeout <= 0:
            raise ValueError('Lift timeout must be a positive finite number')
        tolerance = 0.005
        hold_sim = 0.2
        self.lift_report = {
            'tolerance_m': tolerance, 'hold_sim_s': hold_sim,
            'stage_timeout_wall_s': wall_timeout, 'steps': []}
        for target in (0.13, 0.07, 0.01):
            start_wall = time.monotonic()
            start_sim = self.sim_time
            initial_joint_count = self.count['/joint_states']
            initial_target_count = self.count['/lift/command']
            start_position = self.lift_position()
            stable_since = None
            stable_samples = 0
            previous_joint_count = initial_joint_count
            step = {'target_m': target, 'start_position_m': start_position, 'passed': False}
            self.lift_report['steps'].append(step)
            while rclpy.ok() and time.monotonic() - start_wall < wall_timeout:
                command = Float64(); command.data = target
                self.lift.publish(command)
                rclpy.spin_once(self, timeout_sec=0.05)
                now = time.monotonic()
                position = self.lift_position()
                requested = self.latest.get('/lift/command')
                requested_value = (float(requested.data) if requested is not None
                                   and math.isfinite(requested.data) else None)
                joint_age = (now - self.received['/joint_states']
                             if '/joint_states' in self.received else None)
                target_age = (now - self.received['/lift/command']
                              if '/lift/command' in self.received else None)
                sim_elapsed = (self.sim_time - start_sim
                               if self.sim_time is not None and start_sim is not None else None)
                step.update({
                    'measured_m': position, 'requested_target_m': requested_value,
                    'joint_feedback_age_wall_s': joint_age,
                    'target_feedback_age_wall_s': target_age,
                    'joint_messages': self.count['/joint_states'] - initial_joint_count,
                    'target_messages': self.count['/lift/command'] - initial_target_count,
                    'command_subscribers': self.lift.get_subscription_count(),
                    'sim_elapsed_s': sim_elapsed, 'wall_elapsed_s': now - start_wall})
                valid = (
                    position is not None and abs(position - target) <= tolerance
                    and step['joint_messages'] > 0 and joint_age is not None and joint_age < 2.0
                    and requested_value is not None and abs(requested_value - target) <= 1e-6
                    and step['target_messages'] > 0 and target_age is not None and target_age < 2.0
                    and sim_elapsed is not None and sim_elapsed > 0)
                if not valid:
                    stable_since = None
                    stable_samples = 0
                elif self.count['/joint_states'] != previous_joint_count:
                    if stable_since is None:
                        stable_since = self.sim_time
                    stable_samples += 1
                    if stable_samples >= 3 and self.sim_time - stable_since >= hold_sim:
                        step['passed'] = True
                        break
                previous_joint_count = self.count['/joint_states']
                time.sleep(0.02)
            if not step['passed']:
                raise RuntimeError(
                    f'Lift did not reach and hold {target} m; '
                    f"measured={step.get('measured_m')}; "
                    f"requested ROS target={step.get('requested_target_m')}; "
                    f"simulation advanced={step.get('sim_elapsed_s')} s. "
                    'See lift.steps for command-path and feedback diagnostics.')
        return self.lift_report

    def release_lift_test(self):
        """Best-effort lower-target cleanup; this does not prove lowering completed."""
        command = Float64(); command.data = 0.01
        for _ in range(3):
            if not rclpy.ok():
                break
            self.lift.publish(command)
            rclpy.spin_once(self, timeout_sec=0.03)



def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--timeout', type=float, default=60, help='Readiness wall-time timeout in seconds')
    parser.add_argument('--motion', action='store_true', help='Turn slowly, send zero velocity and verify stopping; changes robot pose')
    parser.add_argument('--lift', action='store_true', help='Raise/lower lift and verify joint feedback; do not run under a rack')
    parser.add_argument('--lift-timeout', type=float, default=None,
                        help='Wall seconds per lift target; defaults to --timeout')
    args = parser.parse_args()
    if not math.isfinite(args.timeout) or args.timeout <= 0:
        parser.error('--timeout must be positive and finite')
    if args.lift_timeout is not None and (not math.isfinite(args.lift_timeout) or args.lift_timeout <= 0):
        parser.error('--lift-timeout must be positive and finite')
    rclpy.init(args=[]); node = Check()
    report = {'ok': False, 'revision': 'lift-force-pid-fix2',
              'motion_requested': args.motion, 'lift_requested': args.lift}
    try:
        deadline = time.monotonic() + args.timeout
        issues = ['Waiting for data']
        while time.monotonic() < deadline and rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
            issues = node.problems()
            if not issues: break
        if issues: raise RuntimeError('; '.join(issues))
        if args.motion: report['motion'] = node.motion_test()
        if args.lift:
            report['lift'] = node.lift_test(args.lift_timeout if args.lift_timeout is not None else args.timeout)
        report['ok'] = True
    except (RuntimeError, KeyboardInterrupt) as exc:
        report['error'] = str(exc) or 'Interrupted'
    finally:
        # Capture final observations, not the old readiness-only counters.
        report['messages_received'] = dict(node.count)
        report['sim_time'] = node.sim_time
        if node.lift_report is not None:
            report['lift'] = node.lift_report
        if args.motion: node.vel.publish(Twist())
        if args.lift and rclpy.ok(): node.release_lift_test()
        node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()
    print(json.dumps(report, indent=2), flush=True)
    sys.exit(0 if report['ok'] else 1)
