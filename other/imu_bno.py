from adafruit_bno08x.i2c import _BNO08X_DEFAULT_ADDRESS, BNO08X_I2C
from adafruit_bno08x import BNO_REPORT_ACCELEROMETER, BNO_REPORT_GYROSCOPE, BNO_REPORT_MAGNETOMETER, BNO_REPORT_GRAVITY, BNO_REPORT_GAME_ROTATION_VECTOR, BNO_REPORT_GEOMAGNETIC_ROTATION_VECTOR, BNO_REPORT_ROTATION_VECTOR
import busio

import time
import ahrs
import math
import numpy as np

class Imu(BNO08X_I2C):
    def __init__(self, i2c_bus, reset=None, address=..., debug=False, features = [BNO_REPORT_ACCELEROMETER, BNO_REPORT_MAGNETOMETER, BNO_REPORT_GYROSCOPE]):
        super().__init__(i2c_bus, reset, address, debug)

        self.initialize()
        for fe in features:
            self.enable_feature(fe)
            # self.enable_feature(BNO_REPORT_ACCELEROMETER)
            # self.enable_feature(BNO_REPORT_MAGNETOMETER)
            # self.enable_feature(BNO_REPORT_GYROSCOPE)
            # self.enable_feature(BNO_REPORT_GRAVITY)
        self.begin_calibration()

        self._filt_pitch = 0
        self._filt_roll = 0
        self._filt_yaw = 0

        self._last_timestamp = time.time()

        #ekf fields
        self.ekf_module = None
        self.ekf_Q = None
        self.ekf_last_timestamp = 0
        #/ekf fields
        #madgwick fields
        self.madgwick_module = None
        self.madgwick_Q = None
        self.madgwick_last_timestamp = 0
        #/madgwick fields
        #quest fields
        self.quest_module = None
        self.quest_Q = None
        self.quest_last_timestamp = 0
        #/quest fields
    
    def get_accel_data(self, g=False):
        if g:
            acx, acy, acz = self.gravity
        else:
            acx, acy, acz = self.acceleration
        return {"x": acx, "y": acy, "z": acz}
    
    def get_gyro_data(self):
        gx, gy, gz = self.gyro
        return {"x": gx, "y": gy, "z": gz}
    
    def get_magnetic_data(self):
        mx, my, mz = self.magnetic
        return {"x": mx, "y": my, "z": mz}
    
    def q2angles(self, Q):
        rm = ahrs.common.orientation.q2R(Q)
        roll = math.atan2(rm[2][1],rm[2][2]);#-math.asin(rm[0][2])
        pitch = math.atan2(-rm[2][0], math.sqrt(rm[2][1]**2 + rm[2][2]**2))#math.atan2(-rm[1][2], rm[2][2])
        yaw = math.atan2(rm[1][0], rm[0][0])
        return roll, pitch, yaw

    def euler_to_quaternion(self):
        (yaw, pitch, roll) = (self._filt_yaw, self._filt_pitch, self._filt_roll)
        qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
        qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
        qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
        qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
        return [qw, qx, qy, qz]

    #0.3**2 0.5**2 0.8**2
    def init_ekf(self, frame = "ENU", noises = [0.3**2, 0.5**2, 0.8**2]):
        self.ekf_module = ahrs.filters.EKF(frame= frame, noises = noises)
        self.ekf_Q = np.tile(self.euler_to_quaternion(), (1))
        self._last_timestamp = time.time()
        self.ekf_last_timestamp = self._last_timestamp
    
    def init_madgwick(self, gain=0.033):
        self.madgwick_module = ahrs.filters.Madgwick(gain=gain)
        self.madgwick_Q = np.tile(self.euler_to_quaternion(), (1))
        self._last_timestamp = time.time()
        self.madgwick_last_timestamp = self._last_timestamp
    
    # def init_quest(self):
    #     self.quest_module = ahrs.filters.QUEST()
    #     self.quest_module.
    
    def get_ekf_angles(self):

        if self.ekf_module is None or self.ekf_last_timestamp != self._last_timestamp:
            self.init_ekf()
        # Get accelerometer data
        accelerometer_data = self.get_accel_data()

        # Get gyroscope data
        gyroscope_data = self.get_gyro_data()

        # Get magnetiic data
        mag_data = self.get_magnetic_data()

        end = time.time()
        self.ekf_module.Dt =  end - self._last_timestamp
        self.ekf_module.frequency = self.ekf_module.Dt ** (-1)
        self._last_timestamp = end
        self.ekf_last_timestamp = end
        
        # def deg2rad(dat):
        #     return {'x': np.deg2rad(dat['x']), 'y': np.deg2rad(dat['y']), 'z': np.deg2rad(dat['z'])}
        def dict2arr(dat):
            return [dat['x'], dat['y'], dat['z']]
        self.ekf_Q = self.ekf_module.update(self.ekf_Q, gyr=dict2arr(gyroscope_data), acc=dict2arr(accelerometer_data), mag=dict2arr(mag_data))
        # self.ekf_Q = self.ekf_module.update(self.ekf_Q,
                                            #  gyr=dict2arr(gyroscope_data),
                                            #    acc=dict2arr(accelerometer_data))
        rm = ahrs.common.orientation.q2R(self.ekf_Q)
        self._filt_roll = math.atan2(rm[2][1],rm[2][2]);#-math.asin(rm[0][2])
        self._filt_pitch = math.atan2(-rm[2][0], math.sqrt(rm[2][1]**2 + rm[2][2]**2))#math.atan2(-rm[1][2], rm[2][2])
        self._filt_yaw = math.atan2(rm[1][0], rm[0][0])

        return (self._filt_roll, self._filt_pitch, self._filt_yaw)
    
    def get_madgwick_angles(self):

        if self.madgwick_module is None or self.madgwick_last_timestamp != self._last_timestamp:
            self.init_madgwick()
        accelerometer_data = self.get_accel_data()
        gyroscope_data = self.get_gyro_data()
        mag_data = self.get_magnetic_data()

        end = time.time()
        self.madgwick_module.Dt =  end - self._last_timestamp
        self.madgwick_module.frequency = self.madgwick_module.Dt ** (-1)
        self._last_timestamp = end
        self.madgwick_last_timestamp = end
        
        def dict2arr(dat):
            return [dat['x'], dat['y'], dat['z']]
        
        self.madgwick_Q = self.madgwick_module.updateMARG(self.madgwick_Q, gyr=dict2arr(gyroscope_data), acc=dict2arr(accelerometer_data), mag=dict2arr(mag_data))
        # self.ekf_Q = self.ekf_module.update(self.ekf_Q, gyr=dict2arr(gyroscope_data), acc=dict2arr(accelerometer_data), mag=dict2arr(mag_data))
        rm = ahrs.common.orientation.q2R(self.madgwick_Q)
        self._filt_roll = math.atan2(rm[2][1],rm[2][2]);#-math.asin(rm[0][2])
        self._filt_pitch = math.atan2(-rm[2][0], math.sqrt(rm[2][1]**2 + rm[2][2]**2))#math.atan2(-rm[1][2], rm[2][2])
        self._filt_yaw = math.atan2(rm[1][0], rm[0][0])

        return (self._filt_roll, self._filt_pitch, self._filt_yaw)
    
    def get_ekf_gravity(self):
        r, p, y = self.get_ekf_angles()
        g = (0, 0, 9.8)
        x1 = -math.cos(y)* math.cos(p)
        x2 = -math.sin(y) * math.cos(p)
        x3 = -math.sin(p)
        y1 = -math.cos(y)*math.sin(p)*math.sin(r)-math.sin(y)*math.cos(r)
        y2 = -math.sin(y)*math.sin(p)*math.sin(r)+math.cos(y)*math.cos(r)
        y3 =  math.cos(p)*math.sin(r)
        z1, z2, z3 = np.cross([x1, x2, x3], [y1, y2, y3])
        g1 = (np.dot([x1, x2, x3], g) / np.linalg.norm([x1, x2, x3]))
        g2 = (np.dot([y1, y2, y3], g) / np.linalg.norm([y1, y2, y3]))
        g3 = (np.dot([z1, z2, z3], g) / np.linalg.norm([z1, z2, z3]))
        return (g1, g2, g3)

def frequency(process_func, count=1000, n=10, init_functions = []):
    i2c = busio.I2C((1, 14), (1, 15))
    device = Imu(i2c, address=0x4b, features=[])
    for f in init_functions:
        f(device)
    freqs = []
    all_intervals = []
    for i in range(n):
        last_timestamp = time.time()
        intervals = []
        for _ in range(count):
            process_func(device)
            end = time.time()
            intervals.append(end - last_timestamp)
            last_timestamp = end
        freqs.append((sum(intervals)/len(intervals))**(-1))
        print(f"Frequency: {freqs[i]} Hz")
        print(f"Max interval: {max(intervals)} s\nMin interval: {min(intervals)} s")
        print(f"Std: {np.std(intervals)} s")
        all_intervals += intervals
    print("SUMMARY")
    print(f"Mean frequency: {sum(freqs)/len(freqs)} Hz")
    print(f"Max: {max(freqs)} Hz\nMin: {min(freqs)} Hz")
    print(f"Std: {np.std(freqs)} Hz")
    print(f"Min interval: {min(all_intervals)} s\nMax interval: {max(all_intervals)}")
    return all_intervals


def show_raw_data():
    i2c = busio.I2C((1, 14), (1, 15))
    device = Imu(i2c, address=0x4b)
    while True:
        accel_data = device.get_accel_data(g=False)
        gyro_data = device.get_gyro_data()
        mag_data = device.get_magnetic_data()

        print(f"Gyro: {gyro_data}\tAccel: {accel_data}\tMagnetic: {mag_data}")
        time.sleep(0.1)

def show_builtin_rpy():
    i2c = busio.I2C((1, 14), (1, 15))
    device = Imu(i2c, address=0x4b, features=[BNO_REPORT_ROTATION_VECTOR])
    device.enable_feature(BNO_REPORT_ROTATION_VECTOR)
    while True:
        x, y, z, w = device.quaternion
        r, p, y = device.q2angles(np.array([w, x, y, z]))
        print(f"roll: {math.degrees(r):3.4f}\tpitch: {math.degrees(p):3.4f}\tyaw: {math.degrees(y):3.4f}")

def show_magnetic_rpy():
    i2c = busio.I2C((1, 14), (1, 15))
    device = Imu(i2c, address=0x4b, features=[BNO_REPORT_GEOMAGNETIC_ROTATION_VECTOR])
    while True:
        x, y, z, w = device.geomagnetic_quaternion
        r, p, y = device.q2angles(np.array([w, x, y, z]))
        print(f"roll: {math.degrees(r):3.4f}\tpitch: {math.degrees(p):3.4f}\tyaw: {math.degrees(y):3.4f}")

def show_filtred_rpy(filter = Imu.get_ekf_angles):
    i2c = busio.I2C((1, 14), (1, 15))
    device = Imu(i2c, address=0x4b)
    # device.init_ekf()
    while True:
        # r, p, y = device.get_ekf_angles()
        r, p, y = filter(device)
        print(f"roll: {math.degrees(r):3.4f}\tpitch: {math.degrees(p):3.4f}\tyaw: {math.degrees(y):3.4f}")

def show_gravity():
    i2c = busio.I2C((1, 14), (1, 15))
    device = Imu(i2c, address=0x4b, features=[BNO_REPORT_GRAVITY, BNO_REPORT_ACCELEROMETER])
    while True:
        print("GRAVITY: ", device.gravity)
        print("ACC: ", device.acceleration)
        time.sleep(0.1)

def show_filtred_gravity():
    i2c = busio.I2C((1, 14), (1, 15))
    device = Imu(i2c, address=0x4b)
    while True:
        g1, g2, g3 = device.get_ekf_gravity()
        print("GRAVITY: ", g1, g2, g3)

    


def __get_raw(dev:Imu):
    accel_data = dev.get_accel_data(g=False)
    gyro_data = dev.get_gyro_data()
    mag_data = dev.get_magnetic_data()
    return [accel_data, gyro_data, mag_data]
def __get_rpy_q_geomagnetic(dev:Imu):
    x, y, z, w = dev.geomagnetic_quaternion
    return np.tile([w, x, y, z], (1))
def __get_rpy_q_builtin(dev:Imu):
    x, y, z, w = dev.quaternion
    return np.tile([w, x, y, z], (1))
def __get_rpy_q_ekf(dev:Imu):
    a, g, m = __get_raw(dev)
    end = time.time()
    dev.ekf_module.Dt =  end - dev._last_timestamp
    dev.ekf_module.frequency = dev.ekf_module.Dt ** (-1)
    dev._last_timestamp = end
    dev.ekf_last_timestamp = end
    def dict2arr(dat):
        return [dat['x'], dat['y'], dat['z']]
    dev.ekf_Q = dev.ekf_module.update(dev.ekf_Q, gyr=dict2arr(g), acc=dict2arr(a), mag=dict2arr(m))
    return dev.ekf_Q

def summary_monitor():
    i2c = busio.I2C((1, 14), (1, 15))
    device = Imu(i2c, address=0x4b)
    device.enable_feature(BNO_REPORT_GEOMAGNETIC_ROTATION_VECTOR)
    device.enable_feature(BNO_REPORT_ROTATION_VECTOR)
    device.init_ekf()
    while True:
        q_geomag = __get_rpy_q_geomagnetic(device)
        q_rpy = __get_rpy_q_builtin(device)
        q_ekf = __get_rpy_q_ekf(device)
        print(f"GEOMAGNETIC_ROTATION_VECTOR: {q_geomag}")
        print(f"ROTATION_VECTOR: {q_rpy}")
        print(f"EKF_ROTATION_VECTOR: {q_ekf}\n")

        print("GEOMAGNETIC_RPY: ", [math.degrees(i) for i in device.q2angles(np.array(q_geomag))])
        print("ROTATION_RPY: ", [math.degrees(i) for i in device.q2angles(np.array(q_rpy))])
        print("EKF_RPY: ", [math.degrees(i) for i in device.q2angles(np.array(q_ekf))], "\n\n")

def write_summary(fp, count=10000):
    i2c = busio.I2C((1, 14), (1, 15))
    device = Imu(i2c, address=0x4b)
    device.enable_feature(BNO_REPORT_GEOMAGNETIC_ROTATION_VECTOR)
    device.enable_feature(BNO_REPORT_ROTATION_VECTOR)
    device.init_ekf()
    data = []
    for _ in range(count):
        q_geomag = __get_rpy_q_geomagnetic(device)
        q_rpy = __get_rpy_q_builtin(device)
        q_ekf = __get_rpy_q_ekf(device)
        data.append({"GEOMAG": list(q_geomag), "RPY": list(q_rpy), "EKF": list(q_ekf)})
    with open(fp, "w") as f:
        import json
        json.dump(data, f)

def write_summary_angles(fp, count=10000):
    i2c = busio.I2C((1, 14), (1, 15))
    device = Imu(i2c, address=0x4b)
    device.enable_feature(BNO_REPORT_GEOMAGNETIC_ROTATION_VECTOR)
    device.enable_feature(BNO_REPORT_ROTATION_VECTOR)
    device.init_ekf()
    data = []
    def to_angles(device, q):
        return [math.degrees(i) for i in device.q2angles(np.array(q))]
    for _ in range(count):
        q_geomag = __get_rpy_q_geomagnetic(device)
        q_rpy = __get_rpy_q_builtin(device)
        q_ekf = __get_rpy_q_ekf(device)
        data.append({"GEOMAG": to_angles(device, q_geomag), "RPY": to_angles(device, q_rpy), "EKF": to_angles(device, q_ekf)})
    with open(fp, "w") as f:
        import json
        json.dump(data, f)
   


if __name__ == "__main__":
    # intervals = frequency(__get_rpy_q_builtin, init_functions=[lambda x: x.enable_feature(BNO_REPORT_ROTATION_VECTOR)])
    # intervals = frequency(__get_raw)
    # intervals = frequency(__get_rpy_q_ekf, init_functions=[Imu.init_ekf])
    # intervals = frequency(__get_rpy_q_geomagnetic, init_functions=[lambda x: x.enable_feature(BNO_REPORT_GEOMAGNETIC_ROTATION_VECTOR)])
    # with open("./data/intervals_geomagnetic_rpy.json", "w") as f:
    #     import json
    #     json.dump(intervals, f)
    # show_magnetic_rpy()
    # show_builtin_rpy()
    show_filtred_rpy()
    # summary_monitor()
    # show_filtred_gravity()
    # write_summary_angles("./data/test2.json")
    

