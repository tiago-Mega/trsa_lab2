"""Small ROS-independent teaching models. No ground truth is used here.

Injected errors are measurement-level approximations, NOT a wheel/terrain contact
model. The reported uncertainty is a configurable hypothesis, not measured truth.
"""
from dataclasses import dataclass
import math
import random


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def step_pose(x, y, yaw, v, w, dt):
    """Integrate a constant body velocity exactly over a positive interval."""
    if not all(math.isfinite(k) for k in (x, y, yaw, v, w, dt)) or dt < 0:
        raise ValueError('Pose, velocity and dt must be finite; dt must be non-negative')
    turn = w * dt
    distance = v * dt
    scale = math.sin(turn / 2) / (turn / 2) if abs(turn) > 1.e-8 else 1.0
    return (x + distance * scale * math.cos(yaw + turn/2),
            y + distance * scale * math.sin(yaw + turn/2), wrap(yaw + turn))


@dataclass
class NoiseConfig:
    profile: str = 'nominal'
    seed: int = 42
    wheel_v_stddev: float = 0.0
    wheel_w_stddev: float = 0.0
    wheel_scale: float = 1.0
    wheel_yaw_bias: float = 0.0
    gyro_stddev: float = 0.0
    gyro_bias: float = 0.0

    def __post_init__(self):
        if self.profile not in ('nominal','wheel_drift','imu_bias','custom'):
            raise ValueError('Unknown sensor profile: ' + self.profile)
        if not all(math.isfinite(v) for v in (self.wheel_v_stddev,self.wheel_w_stddev,
                self.wheel_scale,self.wheel_yaw_bias,self.gyro_stddev,self.gyro_bias)):
            raise ValueError('Non-finite noise parameter')
        if min(self.wheel_v_stddev,self.wheel_w_stddev,self.gyro_stddev) < 0 or self.wheel_scale <= 0:
            raise ValueError('Noise standard deviations must be non-negative and scale positive')
        # Presets do not silently alter the covariance reported to the EKF.
        # Keep profile="custom" when editing every coefficient explicitly.
        if self.profile == 'wheel_drift':
            self.wheel_v_stddev=0.01; self.wheel_w_stddev=0.015
            self.wheel_scale=1.03; self.wheel_yaw_bias=0.03
        elif self.profile == 'imu_bias':
            self.gyro_stddev=0.001; self.gyro_bias=0.035


class SensorErrors:
    def __init__(self, cfg):
        self.cfg=cfg
        # Independent RNGs mean callback interleaving does not mix the streams.
        self.wheel_rng=random.Random(cfg.seed)
        self.imu_rng=random.Random(cfg.seed+1)

    def wheel(self,v,w):
        if not all(math.isfinite(k) for k in (v,w)):
            raise ValueError('Non-finite wheel sample')
        c=self.cfg
        # Stationary encoders should not accumulate a fabricated constant drift.
        moving=abs(v)>0.005 or abs(w)>0.005
        if not moving: return v,w
        return (v*c.wheel_scale+self.wheel_rng.gauss(0,c.wheel_v_stddev),
                w+c.wheel_yaw_bias+self.wheel_rng.gauss(0,c.wheel_w_stddev))

    def gyro(self,w):
        if not math.isfinite(w): raise ValueError('Non-finite gyro sample')
        return w+self.cfg.gyro_bias+self.imu_rng.gauss(0,self.cfg.gyro_stddev)


def diagonal_covariance(values):
    if not all(math.isfinite(v) and v>=0 for v in values):
        raise ValueError('Covariance diagonals must be finite, non-negative VARIANCES')
    n=len(values)
    return [float(values[i]) if i==j else 0.0 for i in range(n) for j in range(n)]


def relative_pose(p, origin):
    """Pose in the initial pose's SE(2) frame; evaluation only."""
    dx,dy=p[0]-origin[0],p[1]-origin[1]
    c,s=math.cos(origin[2]),math.sin(origin[2])
    return c*dx+s*dy, -s*dx+c*dy, wrap(p[2]-origin[2])
