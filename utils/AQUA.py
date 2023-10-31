from utils.PostprocDefault import PostprocDefault
from ahrs.filters import AQUA as ahrsAQUA
import numpy as np
import time
import math
"""
Parameters
acc : numpy.ndarray, default: None
    N-by-3 array with measurements of acceleration in m/s^2
gyr : numpy.ndarray, default: None
    N-by-3 array with measurements of angular velocity in rad/s
mag : numpy.ndarray, default: None
    N-by-3 array with measurements of magnetic field in mT
frequency : float, default: 100.0
    Sampling frequency in Herz
Dt : float, default: 0.01
    Sampling step in seconds. Inverse of sampling frequency. Not required if frequency value is given
alpha : float, default: 0.01
    Gain characterizing cut-off frequency for accelerometer quaternion
beta : float, default: 0.01
    Gain characterizing cut-off frequency for magnetometer quaternion
threshold : float, default: 0.9
    Threshold to discriminate between LERP and SLERP interpolation
adaptive : bool, default: False

var pitch = asin(-2.0*(q.x*q.z - q.w*q.y));
var roll = atan2(2.0*(q.x*q.y + q.w*q.z), q.w*q.w + q.x*q.x - q.y*q.y - q.z*q.z);
"""
class AQUA(PostprocDefault):
    time_old = time.time()
    def deg2rad(self, dat):
        return {'x': np.deg2rad(dat['x']), 'y': np.deg2rad(dat['y']), 'z': np.deg2rad(dat['z'])}
    def dict2arr(self, dat):
        return [dat['x'], dat['y'], dat['z']]
    def __init__(self, alpha= 0.01, threshold= 0.9, adaptive = False) -> None:
        self.aqua= ahrsAQUA(alpha=alpha, threshold=threshold, adaptive=adaptive)
        self.Q = np.tile([1., 0.,0.,0.], (1))
    def apply(self, a, g):
        end = time.time()
        self.aqua.Dt =  end - self.time_old
        self.aqua.frequency = self.aqua.Dt ** (-1)
        self.time_old = end
        # print(self.aqua.frequency)
        self.Q = self.aqua.updateIMU(self.Q, gyr=self.dict2arr(self.deg2rad(g)), acc=self.dict2arr(a))

        ww = self.Q[0]
        xx = self.Q[1]
        yy = self.Q[2]
        zz = self.Q[3]
        t0 = +2.0 * (ww * xx + yy * zz)
        t1 = +1.0 - 2.0 * (xx * xx + yy * yy)
        roll = math.atan2(t0, t1)
    
        t2 = +2.0 * (ww * yy - zz * xx)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch = math.asin(t2)
        
        return {"roll": math.degrees(roll), "pitch": math.degrees(pitch)}
