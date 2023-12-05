from utils.PostprocDefault import PostprocDefault
from ahrs.filters import Madgwick as ahrsMadgwick
import numpy as np
import time
import math
"""
madgwick = Madgwick()
Q = np.tile([1., 0., 0., 0.], (num_samples, 1)) # Allocate for quaternions
 for t in range(1, num_samples):
    Q[t] = madgwick.updateIMU(Q[t-1], gyr=gyro_data[t], acc=acc_data[t])


var pitch = asin(-2.0*(q.x*q.z - q.w*q.y));
var roll = atan2(2.0*(q.x*q.y + q.w*q.z), q.w*q.w + q.x*q.x - q.y*q.y - q.z*q.z);
"""
class Madgwick(PostprocDefault):
    time_old = time.time()
    def deg2rad(self, dat):
        return {'x': np.deg2rad(dat['x']), 'y': np.deg2rad(dat['y']), 'z': np.deg2rad(dat['z'])}
    def dict2arr(self, dat):
        return [dat['x'], dat['y'], dat['z']]
    def __init__(self, gain) -> None:
        self.madgwick = ahrsMadgwick(gain=gain)
        self.Q = np.tile([1., 0.,0.,0.], (1))
    def apply(self, a, g, m):
        end = time.time()
        self.madgwick.Dt =  end - self.time_old
        self.madgwick.frequency = self.madgwick.Dt ** (-1)
        self.time_old = end
        # print(self.madgwick.frequency)
        
        self.Q = self.madgwick.updateIMU(self.Q, gyr=self.dict2arr(self.deg2rad(g)), acc=self.dict2arr(a))

        # pitch = math.asin(-2.0*(self.Q[1]*self.Q[3] - self.Q[0]*self.Q[2]))
        # roll = math.atan2(2.0*(self.Q[1]*self.Q[2] + self.Q[0]*self.Q[3]), self.Q[0]**2 + self.Q[1]**2 - self.Q[2]**2 - self.Q[3]**2)
        ww = self.Q[0]
        xx = self.Q[1]
        yy = self.Q[2]
        zz = self.Q[3]
        t0 = +2.0 * (ww * xx + yy * zz)
        t1 = +1.0 - 2.0 * (xx * xx + yy * yy)
        roll = math.atan2(t0, t1)
    
        # t2 = +2.0 * (ww * yy - zz * xx)
        # t2 = +1.0 if t2 > +1.0 else t2
        # t2 = -1.0 if t2 < -1.0 else t2
        # pitch = math.asin(t2)
        t0 = math.sqrt(1 + 2 * (ww * yy - xx * zz))
        t1 = math.sqrt(1 - 2 * (ww * yy - xx * zz))
        pitch = 2 * math.atan2(t0, t1) - math.pi / 2
        
        return {"roll": math.degrees(roll), "pitch": math.degrees(pitch)}
