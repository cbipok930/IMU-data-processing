import numpy as np
from mpu6050 import mpu6050
import time
import math
# import angles

def mess2list(dat):
    return [dat['x'], dat['y'], dat['z']]

def deg2rad(dat):
    return {'x': np.deg2rad(dat['x']), 'y': np.deg2rad(dat['y']), 'z': np.deg2rad(dat['z'])}
sensor = mpu6050(0x68)
# start_0 = time.time()
# data_g_old = sensor.get_gyro_data()
# data_a_old = sensor.get_accel_data()
# t_0 = time.time() - start_0
start_1 = time.time()

pitch_old = 0
pitch = 0
roll_old = 0
roll = 0

acc_coef = 0.1
gyro_coef = 1 - acc_coef

while True:
    data_g = sensor.get_gyro_data()
    data_a = sensor.get_accel_data()
    t_1 = time.time() - start_1
    start_1 = time.time()
    print(t_1)
    pitch = (pitch_old + deg2rad(data_g)['y'] * t_1) * gyro_coef + acc_coef * (math.atan(-data_a['x'] / math.sqrt(data_a['y'] ** 2 + data_a['z'] ** 2)))
    pitch_old = pitch
    roll = (roll_old + deg2rad(data_g)['x'] * t_1) * gyro_coef + acc_coef * math.atan(data_a['y'] / math.sqrt(data_a['x'] ** 2 + data_a['z'] ** 2))
    roll_old = roll
    print(math.degrees(pitch), math.degrees(roll))
    # t_1 = t_0
    # time.sleep(0.1)       