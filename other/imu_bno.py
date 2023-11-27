from adafruit_bno08x.i2c import _BNO08X_DEFAULT_ADDRESS, BNO08X_I2C
from adafruit_bno08x import BNO_REPORT_ACCELEROMETER, BNO_REPORT_GYROSCOPE, BNO_REPORT_MAGNETOMETER, BNO_REPORT_GRAVITY, BNO_REPORT_GAME_ROTATION_VECTOR, BNO_REPORT_GEOMAGNETIC_ROTATION_VECTOR, BNO_REPORT_ROTATION_VECTOR
import busio

import time
import ahrs
import math
import numpy as np

class Imu(BNO08X_I2C):
    def __init__(self, i2c_bus, reset=None, address=..., debug=False):
        super().__init__(i2c_bus, reset, address, debug)

        self.initialize()
        self.enable_feature(BNO_REPORT_ACCELEROMETER)
        self.enable_feature(BNO_REPORT_MAGNETOMETER)
        self.enable_feature(BNO_REPORT_GYROSCOPE)
        self.enable_feature(BNO_REPORT_GRAVITY)
        self.begin_calibration()

        self._filt_pitch = 0
        self._filt_roll = 0
        self._filt_yaw = 0

        self._last_timestamp = time.time()

        #ekf fields
        self.ekf_module = None
        self.ekf_Q = None
        self.ekf_last_timestamp = self._last_timestamp
        #/ekf fields
    
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

    def init_ekf(self, frame = "ENU", noises = [0.3**2, 0.5**2, 0.8**2]):
        self.ekf_module = ahrs.filters.EKF(frame= frame, noises = noises)
        self.ekf_Q = np.tile(self.euler_to_quaternion(), (1))
        self.ekf_last_timestamp = self._last_timestamp
    
    def get_ekf_angles(self):

        if self.ekf_module == None or self.ekf_last_timestamp != self._last_timestamp:
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
    device = Imu(i2c, address=0x4b)
    device.enable_feature(BNO_REPORT_ROTATION_VECTOR)
    while True:
        x, y, z, w = device.quaternion
        r, p, y = device.q2angles(np.array([w, x, y, z]))
        print(f"roll: {math.degrees(r):3.4f}\tpitch: {math.degrees(p):3.4f}\tyaw: {math.degrees(y):3.4f}")

def show_filtred_rpy():
    i2c = busio.I2C((1, 14), (1, 15))
    device = Imu(i2c, address=0x4b)
    device.init_ekf()
    while True:
        r, p, y = device.get_ekf_angles()
        print(f"roll: {math.degrees(r):3.4f}\tpitch: {math.degrees(p):3.4f}\tyaw: {math.degrees(y):3.4f}")


if __name__ == "__main__":
    show_filtred_rpy()