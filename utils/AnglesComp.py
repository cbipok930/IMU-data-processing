import numpy as np
# from mpu6050 import mpu6050
import time
import math
from utils.PostprocDefault import PostprocDefault

class AnglesComp(PostprocDefault):
    roll_old = 0
    pitch_old = 0
    time_old = time.time()
    def deg2rad(self, dat):
        return {'x': np.deg2rad(dat['x']), 'y': np.deg2rad(dat['y']), 'z': np.deg2rad(dat['z'])}
    def __init__(self, alpha = 0.5):
        self.alpha = 0.5
    def apply(self, a, g):
        end = time.time()
        t_1 =  end - self.time_old
        self.time_old = end
        pitch = (self.pitch_old + self.deg2rad(g)['y'] * t_1) * self.alpha + (1 - self.alpha) * (math.atan(-a['x'] / math.sqrt(a['y'] ** 2 + a['z'] ** 2)))
        self.pitch_old = pitch
        roll = (self.roll_old + self.deg2rad(g)['x'] * t_1) * self.alpha + (1-self.alpha) * math.atan(a['y'] / math.sqrt(a['x'] ** 2 + a['z'] ** 2))
        self.roll_old = roll
        return {"roll": math.degrees(roll), "pitch": math.degrees(pitch)}