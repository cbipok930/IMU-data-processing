from utils.PostprocDefault import PostprocDefault
from ahrs.filters import EKF as ahrsEKF
import numpy as np
import time
import math
import ahrs
"""
Parameters
gyr : numpy.ndarray, default: None
    N-by-3 array with measurements of angular velocity in rad/s
acc : numpy.ndarray, default: None
    N-by-3 array with measurements of acceleration in in m/s^2
mag : numpy.ndarray, default: None
    N-by-3 array with measurements of magnetic field in mT
frequency : float, default: 100.0
    Sampling frequency in Herz.
frame : str, default: 'NED'
    Local tangent plane coordinate frame. Valid options are right-handed 'NED' for North-East-Down and 'ENU' for East-North-Up.
q0 : numpy.ndarray, default: None
    Initial orientation, as a versor (normalized quaternion).
magnetic_ref : float or numpy.ndarray
    Local magnetic reference.
noises : numpy.ndarray
    List of noise variances for each type of sensor. Default values: [0.3**2, 0.5**2, 0.8**2].
Dt : float, default: 0.01
   Sampling step in seconds. Inverse of sampling frequency. NOT required if frequency value is given.

var pitch = asin(-2.0*(q.x*q.z - q.w*q.y));
var roll = atan2(2.0*(q.x*q.y + q.w*q.z), q.w*q.w + q.x*q.x - q.y*q.y - q.z*q.z);
"""
class EKF(PostprocDefault):
    time_old = time.time()
    def deg2rad(self, dat):
        return {'x': np.deg2rad(dat['x']), 'y': np.deg2rad(dat['y']), 'z': np.deg2rad(dat['z'])}
    def dict2arr(self, dat):
        return [dat['x'], dat['y'], dat['z']]
    def __init__(self, frame = "ENU", noises = [0.3**2, 0.5**2, 0.8**2]) -> None:
        self.ekf = ahrsEKF(frame= frame, noises = noises)
        self.Q = np.tile([1., 0.,0.,0.], (1))
    def apply(self, a, g):
        end = time.time()
        self.ekf.Dt =  end - self.time_old
        self.ekf.frequency = self.ekf.Dt ** (-1)
        self.time_old = end
        # print(self.aqua.frequency)
        self.Q = self.ekf.update(self.Q, gyr=self.dict2arr(self.deg2rad(g)), acc=self.dict2arr(a))
        rm = ahrs.common.orientation.q2R(self.Q)
        roll = math.atan2(rm[2][1],rm[2][2]);#-math.asin(rm[0][2])
        pitch = math.atan2(-rm[2][0], math.sqrt(rm[2][1]**2 + rm[2][2]**2))#math.atan2(-rm[1][2], rm[2][2])
        return {"roll": math.degrees(roll), "pitch": math.degrees(pitch)}