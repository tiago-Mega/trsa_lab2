"""Record wheel/EKF/ground-truth poses; report only actual measured comparison."""
import argparse
import csv
import json
import math
from pathlib import Path
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock
from std_msgs.msg import String
from .fusion_evaluation import compare

class Recorder(Node):
    def __init__(self, ekf_topic="/odometry/filtered"):
        super().__init__('trsa_fusion_recorder')
        self.samples={key:[] for key in ('wheel','ekf','truth')}; self.sim=None; self.runtime={}
        self.reset_detected=False; self.previous_clock=None
        for key,topic in [('wheel','/wheel/odom'),('ekf',ekf_topic),('truth','/ground_truth/odom')]:
            self.create_subscription(Odometry,topic,lambda m,k=key:self.record(k,m),qos_profile_sensor_data)
        self.create_subscription(Clock,'/clock',self.clock,qos_profile_sensor_data)
        self.create_subscription(String,'/trsa/runtime_info',self.info,
                                 QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
    def info(self,msg):
        try:self.runtime=json.loads(msg.data)
        except ValueError:pass
    def clock(self,msg):
        now=msg.clock.sec+msg.clock.nanosec*1.e-9
        if self.previous_clock is not None and now<self.previous_clock:self.reset_detected=True
        self.previous_clock=now;self.sim=now
    def record(self,key,msg):
        p=msg.pose.pose.position;q=msg.pose.pose.orientation
        t=msg.header.stamp.sec+msg.header.stamp.nanosec*1.e-9
        yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
        if all(math.isfinite(x) for x in (t,p.x,p.y,yaw)):
            self.samples[key].append((t,p.x,p.y,yaw))


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--duration',type=float,default=30,help='Simulation seconds, not wall seconds')
    parser.add_argument('--wall-timeout',type=float,default=600)
    parser.add_argument('--output',type=Path,help='Filename prefix; existing outputs are never overwritten')
    parser.add_argument('--ekf-topic',choices=['/odometry/filtered','/odom'],default='/odometry/filtered',
                        help='Use /odom when direct EKF mode remaps its output; use /odometry/filtered in observer mode.')
    args,rosargs=parser.parse_known_args()
    if not all(math.isfinite(x) and x>0 for x in (args.duration,args.wall_timeout)):
        parser.error('Durations must be positive and finite')
    prefix=args.output or Path('/home/tmega/trsa_lab2/data/fusion')/time.strftime('run_%Y%m%d_%H%M%S')
    paths={key:Path(str(prefix)+suffix) for key,suffix in [('raw','_samples.csv'),('errors','_errors.csv'),('summary','_summary.json')]}
    if any(p.exists() for p in paths.values()):parser.error('Output already exists; use a new prefix')
    prefix.parent.mkdir(parents=True,exist_ok=True)
    rclpy.init(args=rosargs);node=Recorder(args.ekf_topic);begin=time.monotonic();start=None;code=0;error=None
    print('Recording; drive or send a goal in another terminal. Ground truth is used ONLY for this evaluation.',flush=True)
    try:
        while rclpy.ok():
            rclpy.spin_once(node,timeout_sec=0.1)
            if node.reset_detected:raise RuntimeError('Simulation time reset during recording; start a new run')
            if start is None and node.sim is not None and all(len(v)>=2 for v in node.samples.values()):start=node.sim
            if start is not None and node.sim-start>=args.duration:break
            if time.monotonic()-begin>args.wall_timeout:raise TimeoutError('Recording wall timeout; check sensor streams and simulation speed')
    except KeyboardInterrupt:
        error='Interrupted; saved the measured partial interval'
    except Exception as exc:error=str(exc);code=1
    finally:
        with paths['raw'].open('w',newline='') as f:
            writer=csv.writer(f);writer.writerow(['source','sim_time','x_m','y_m','yaw_rad'])
            for key,values in node.samples.items():
                for sample in values:writer.writerow([key,*sample])
        try:
            rows,summary=compare(node.samples)
            with paths['errors'].open('w',newline='') as f:
                writer=csv.DictWriter(f,fieldnames=list(rows[0]));writer.writeheader();writer.writerows(rows)
        except ValueError as exc:
            summary={'error':str(exc)};code=1
        summary.update(ekf_topic=args.ekf_topic,runtime=node.runtime,complete=error is None,wall_elapsed_s=time.monotonic()-begin,
                       recording_note=error,raw_samples={k:len(v) for k,v in node.samples.items()})
        paths['summary'].write_text(json.dumps(summary,indent=2))
        print(json.dumps(summary,indent=2));print('Saved prefix: '+str(prefix))
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
    raise SystemExit(code)
