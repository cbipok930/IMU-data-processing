import numpy as np
from mpu6050 import mpu6050
import time
import math
import ahrs
# import quaternion

def ToEulerAngles(q):
    angles = [0, 0, 0]

    #// roll (x-axis rotation)
    sinr_cosp = 2 * (q[0] * q[1] + q[2] * q[3])
    cosr_cosp = 1 - 2 * (q[1] * q[1] + q[2] * q[2])
    angles[0] = math.atan2(sinr_cosp, cosr_cosp)

    # // pitch (y-axis rotation)
    sinp = math.sqrt(1 + 2 * (q[0] * q[2] - q[1] * q[3]))
    cosp = math.sqrt(1 - 2 * (q[0] * q[2] - q[1] * q[3]))
    angles[1] = 2 * math.atan2(sinp, cosp) - math.pi / 2

    # // yaw (z-axis rotation)
    # double siny_cosp = 2 * (q.w * q.z + q.x * q.y);
    # double cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z);
    # angles.yaw = std::atan2(siny_cosp, cosy_cosp);

    return angles

def mess2list(dat):
    return [dat['x'], dat['y'], dat['z']]

def deg2rad(dat):
    return {'x': np.deg2rad(dat['x']), 'y': np.deg2rad(dat['y']), 'z': np.deg2rad(dat['z'])}

def output(angles):
    angles = [round(math.degrees(el)) for el in angles]
    print(f"roll = {angles[0]} pitch = {angles[1]}")
sensor = mpu6050(0x68)
# start_0 = time.time()
# data_g_old = sensor.get_gyro_data()
# data_a_old = sensor.get_accel_data()
# t_0 = time.time() - start_0


pitch_old = 0
pitch = 0
roll_old = 0
roll = 0

start_1 = time.time()
data_g_1 = sensor.get_gyro_data()
data_a_1 = sensor.get_accel_data()
t_1 = time.time() - start_1

start_2 = time.time()
fst = True
q = 0
while True:
    
    data_g_2 = sensor.get_gyro_data()
    data_a_2 = sensor.get_accel_data()
    t_2 = time.time() - start_2
    if fst:
        Q = ahrs.filters.Complementary(np.array([np.array(mess2list(deg2rad(data_g_1))), np.array(mess2list(deg2rad(data_g_2)))]),
                                np.array([np.array(mess2list(data_a_1)), np.array(mess2list(data_a_2))]),
                                    Dt = (t_1 + t_2)/2,
                                    w0=np.array([roll_old, pitch_old, 0]))
        fst = False
    else:
        Q = ahrs.filters.Complementary(np.array([np.array(mess2list(deg2rad(data_g_1))), np.array(mess2list(deg2rad(data_g_2)))]),
                                np.array([np.array(mess2list(data_a_1)), np.array(mess2list(data_a_2))]),
                                    Dt = (t_1 + t_2)/2,
                                    q0 = q[1])
    
    q = Q.Q
    # a1 = ToEulerAngles(q[0])
    a2 = ToEulerAngles(q[1])
    # output(a1)
    output(a2)
    roll_old = a2[0]
    pitch_old = a2[1]
    # t_1 = t_0
    t_1 = t_2
    data_a_1 = data_a_2
    data_g_1 = data_g_2
    start_2 = time.time()
    # time.sleep(0.1) 