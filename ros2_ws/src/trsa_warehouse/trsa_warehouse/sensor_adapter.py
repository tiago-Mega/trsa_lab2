"""Condition simulated wheel/IMU messages for an inspectable EKF experiment."""
import copy
import math
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from .fusion_math import NoiseConfig, SensorErrors, step_pose, diagonal_covariance


class SensorAdapter(Node):
    def __init__(self):
        super().__init__('sensor_adapter')
        defaults=vars(NoiseConfig())
        defaults.update(wheel_v_variance=0.0004, wheel_vy_variance=0.0001,
                        wheel_w_variance=0.0004, gyro_variance=0.000009,
                        accel_variance=0.01)
        for key,value in defaults.items(): self.declare_parameter(key,value)
        vals={key:self.get_parameter(key).value for key in defaults}
        self.errors=SensorErrors(NoiseConfig(**{k:vals[k] for k in vars(NoiseConfig())}))
        self.var={k:float(vals[k]) for k in vals if k.endswith('_variance')}
        if any(not math.isfinite(x) or x<=0 for x in self.var.values()):
            raise ValueError('All sensor variances must be finite and > 0')
        self.wheel_pub=self.create_publisher(Odometry,'/wheel/odom',20)
        self.imu_pub=self.create_publisher(Imu,'/imu/data',20)
        self.create_subscription(Odometry,'/wheel/odom_raw',self.wheel,qos_profile_sensor_data)
        self.create_subscription(Imu,'/imu',self.imu,qos_profile_sensor_data)
        self.last_stamp=None
        self.pose=(0.0,0.0,0.0)
        self.pose_var=[1.e-6,1.e-6,1.e-6]
        self.get_logger().info(f'Sensor profile={self.errors.cfg.profile}; wheel + gyro covariance adapter ready')

    def wheel(self,raw):
        if raw.header.frame_id != 'odom' or raw.child_frame_id != 'base_footprint':
            self.get_logger().error('Rejected wheel odometry: expected odom -> base_footprint')
            return
        stamp=raw.header.stamp.sec+raw.header.stamp.nanosec*1.e-9
        try: v,w=self.errors.wheel(raw.twist.twist.linear.x,raw.twist.twist.angular.z)
        except ValueError: return
        first=self.last_stamp is None or stamp < self.last_stamp
        if first:
            # Local odometry deliberately starts at zero, independent of the
            # robot's world spawn. AMCL/SLAM owns that global relationship.
            self.pose=(0.0,0.0,0.0); self.pose_var=[1.e-6,1.e-6,1.e-6]
            self.last_stamp=stamp
        dt=stamp-self.last_stamp
        if dt==0 and not first: return
        if dt<0: return
        if dt>2.0:
            self.get_logger().warning('Wheel sample gap >2 sim seconds; retaining pose, not integrating the gap')
            dt=0.0
        self.pose=step_pose(*self.pose,v,w,dt)
        self.last_stamp=stamp
        # Conservative diagonal pose uncertainty for plotting/wheel mode only.
        # The default EKF fuses twist, NOT these correlated integrated poses.
        self.pose_var[2] += self.var['wheel_w_variance']*dt*dt
        linear_var=self.var['wheel_v_variance']*dt*dt+(v*dt)**2*self.pose_var[2]
        self.pose_var[0]+=linear_var; self.pose_var[1]+=linear_var
        msg=Odometry(); msg.header=copy.deepcopy(raw.header); msg.child_frame_id='base_footprint'
        msg.pose.pose.position.x,msg.pose.pose.position.y=self.pose[:2]
        msg.pose.pose.orientation.z=math.sin(self.pose[2]/2)
        msg.pose.pose.orientation.w=math.cos(self.pose[2]/2)
        msg.pose.covariance=diagonal_covariance([self.pose_var[0],self.pose_var[1],1000,1000,1000,self.pose_var[2]])
        msg.twist.twist.linear.x=v
        msg.twist.twist.linear.y=0.0  # Explicit differential-drive nonholonomic constraint.
        msg.twist.twist.angular.z=w
        msg.twist.covariance=diagonal_covariance([self.var['wheel_v_variance'],self.var['wheel_vy_variance'],
                                                1000,1000,1000,self.var['wheel_w_variance']])
        self.wheel_pub.publish(msg)

    def imu(self,raw):
        if raw.header.frame_id != 'imu_link':
            self.get_logger().error('Rejected IMU: expected imu_link')
            return
        msg=copy.deepcopy(raw)
        try: msg.angular_velocity.z=self.errors.gyro(raw.angular_velocity.z)
        except ValueError: return
        # Gazebo supplies an ideal absolute attitude. Do not accidentally hand
        # students a ground-truth heading through a seemingly noisy IMU stream.
        msg.orientation.x=msg.orientation.y=msg.orientation.z=0.0
        msg.orientation.w=1.0
        msg.orientation_covariance=[-1.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0]
        msg.angular_velocity_covariance=diagonal_covariance([self.var['gyro_variance']]*3)
        msg.linear_acceleration_covariance=diagonal_covariance([self.var['accel_variance']]*3)
        self.imu_pub.publish(msg)


def main():
    rclpy.init(); node=SensorAdapter()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()
