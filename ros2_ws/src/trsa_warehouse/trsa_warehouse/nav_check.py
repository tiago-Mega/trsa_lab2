"""Live readiness checks. Startup gate waits for data/TF, not a fixed sleep."""
import argparse
import json
import math
import time
from collections import defaultdict
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.qos import QoSProfile, DurabilityPolicy, qos_profile_sensor_data
from tf2_ros import Buffer, TransformListener
from nav_msgs.msg import Odometry, OccupancyGrid
from sensor_msgs.msg import LaserScan, Imu
from rosgraph_msgs.msg import Clock
from lifecycle_msgs.srv import GetState
from .navigation_config import NAV_NODES

class NavigationCheck(Node):
    def __init__(self,startup=False,require_ekf=True):
        super().__init__('trsa_nav_check')
        self.startup=startup; self.require_ekf=require_ekf
        self.latest={}; self.count=defaultdict(int); self.wall={}
        self.first_clock=None
        for typ,name in [(Clock,'/clock'),(Odometry,'/wheel/odom'),(Odometry,'/odom'),
                         (Odometry,'/odometry/filtered'),(Imu,'/imu/data'),(LaserScan,'/scan')]:
            self.create_subscription(typ,name,lambda m,t=name:self.record(t,m),qos_profile_sensor_data)
        qos=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(OccupancyGrid,'/map',lambda m:self.record('/map',m),qos)
        self.buffer=Buffer(); self.listener=TransformListener(self.buffer,self)
        self.state_clients={} if startup else {n:self.create_client(GetState,'/'+n+'/get_state') for n in NAV_NODES}
        self.pending={}; self.states={}; self.last_query=0

    def record(self,name,msg):
        self.latest[name]=msg; self.count[name]+=1; self.wall[name]=time.monotonic()
        if name=='/clock' and self.first_clock is None:
            self.first_clock=msg.clock.sec+msg.clock.nanosec*1.e-9

    def poll_states(self):
        for name,future in list(self.pending.items()):
            if future.done():
                try: self.states[name]=future.result().current_state.label
                except Exception as exc: self.states[name]='error: '+str(exc)
                del self.pending[name]
        if time.monotonic()-self.last_query < 1.0: return
        self.last_query=time.monotonic()
        for name,client in self.state_clients.items():
            if name not in self.pending and client.service_is_ready():
                self.pending[name]=client.call_async(GetState.Request())

    def problems(self):
        errors=[]; now=time.monotonic()
        # Direct EKF mode remaps its own output to /odom. Observer mode
        # leaves the filtered output on /odometry/filtered. Both are valid,
        # but exactly one odometry stream must own /odom.
        odom_publishers = self.get_publishers_info_by_topic('/odom')
        ekf_on_odom = (len(odom_publishers)==1 and
                       odom_publishers[0].node_name=='ekf_filter_node')
        for name in ('/clock','/wheel/odom','/odom','/odometry/filtered','/imu/data','/scan'):
            if name=='/odometry/filtered' and (not self.require_ekf or ekf_on_odom):
                continue
            if self.count[name]<2: errors.append('Waiting for '+name)
            elif now-self.wall[name]>8: errors.append('Stale wall-time data: '+name)
        clock=self.latest.get('/clock')
        sim=None if clock is None else clock.clock.sec+clock.clock.nanosec*1.e-9
        if sim is None or sim-(self.first_clock if self.first_clock is not None else sim)<0.1:
            errors.append('Simulation time not advancing')
        grid=self.latest.get('/map')
        if grid is None: errors.append('Waiting for /map (SLAM or map_server)')
        elif grid.info.width==0 or grid.info.height==0 or not any(v==0 for v in grid.data):
            errors.append('Map has no observed free cells yet')
        for target in ('odom','base_footprint','laser_frame','imu_link'):
            if not self.buffer.can_transform('map',target,Time()):
                errors.append('Waiting for TF map -> '+target+' (AMCL requires initial pose)')
        wheel=self.latest.get('/wheel/odom'); imu=self.latest.get('/imu/data')
        if wheel and (wheel.twist.covariance[0]<=0 or wheel.twist.covariance[35]<=0):
            errors.append('Wheel velocity/yaw covariance must be positive')
        if imu and (imu.angular_velocity_covariance[8]<=0 or imu.orientation_covariance[0]!=-1):
            errors.append('IMU must have positive gyro covariance and unavailable absolute orientation')
        if not self.startup:
            self.poll_states()
            for name in NAV_NODES:
                if self.states.get(name)!='active': errors.append(f'{name}: '+self.states.get(name,'waiting for lifecycle service'))
            if len(odom_publishers)!=1:
                errors.append('Expected exactly one /odom publisher')
            elif odom_publishers[0].node_name not in ('wheel_odom_interface','ekf_filter_node'):
                errors.append('Unexpected /odom publisher: ' + odom_publishers[0].node_name)
            # TF is inspected by relationship, not by total /tf publisher count:
            # RSP and SLAM/AMCL legitimately also publish different TF edges.
        return errors


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--timeout',type=float,default=120.0,help='Wall seconds; 0 means wait indefinitely (startup only)')
    parser.add_argument('--startup',action='store_true')
    parser.add_argument('--without-ekf',action='store_true',help='Wheel-only phase: EKF is deliberately not running')
    args,rosargs=parser.parse_known_args()
    if not math.isfinite(args.timeout) or args.timeout<0 or (args.timeout==0 and not args.startup):
        parser.error('--timeout must be positive; 0 is allowed only with --startup')
    rclpy.init(args=rosargs); node=NavigationCheck(args.startup,not args.without_ekf)
    begin=time.monotonic(); last_log=0; problems=['No data']; code=1
    try:
        while rclpy.ok() and (args.timeout==0 or time.monotonic()-begin<args.timeout):
            rclpy.spin_once(node,timeout_sec=0.1)
            problems=node.problems()
            if not problems: code=0; break
            if time.monotonic()-last_log>10:
                node.get_logger().info('; '.join(problems[:5])); last_log=time.monotonic()
        print(json.dumps({'ok':code==0,'revision':'lab2-2026.6','stage':'startup' if args.startup else 'navigation',
                          'wall_elapsed_s':round(time.monotonic()-begin,2),
                          'lifecycle':node.states,'messages_received':dict(node.count),
                          'problems':problems},indent=2))
    except KeyboardInterrupt: code=130
    finally:
        node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()
    raise SystemExit(code)
