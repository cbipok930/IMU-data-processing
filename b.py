import numpy as np
from mpu6050 import mpu6050
import time
import angles

def mess2list(dat):
    return [dat['x'], dat['y'], dat['z']]

def deg2rad(dat):
    return {'x': np.deg2rad(dat['x']), 'y': np.deg2rad(dat['y']), 'z': np.deg2rad(dat['z'])}
sensor = mpu6050(0x68)
start_0 = time.time()
data_g_old = sensor.get_gyro_data()
data_a_old = sensor.get_accel_data()
t_0 = time.time() - start_0
start_1 = time.time()
while True:
    data_g = sensor.get_gyro_data()
    data_a = sensor.get_accel_data()
    t_1 = time.time() - start_1
    angl = angles.estimate_orientation([mess2list(data_a_old), mess2list(data_a)], [mess2list(deg2rad(data_g_old)), mess2list(deg2rad(data_g))],
                                [t_0, t_1])
    data_g_old = data_g
    data_a_old = data_a
    print(angl)
    t_1 = t_0
    start_1 = time.time()